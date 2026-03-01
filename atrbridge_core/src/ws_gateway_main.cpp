#include "atrbridge_core/core.hpp"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

namespace
{

std::atomic<bool> g_run{true};

void on_sigint(int)
{
  g_run.store(false);
}

std::optional<double> parse_number(const std::string & json, const std::string & key)
{
  const std::string token = "\"" + key + "\"";
  const size_t key_pos = json.find(token);
  if (key_pos == std::string::npos) {
    return std::nullopt;
  }
  size_t colon = json.find(':', key_pos + token.size());
  if (colon == std::string::npos) {
    return std::nullopt;
  }
  ++colon;
  while (colon < json.size() && (json[colon] == ' ' || json[colon] == '\t')) {
    ++colon;
  }
  size_t end = colon;
  while (end < json.size() &&
    (json[end] == '-' || json[end] == '+' || json[end] == '.' ||
    (json[end] >= '0' && json[end] <= '9')))
  {
    ++end;
  }
  if (end == colon) {
    return std::nullopt;
  }
  return std::stod(json.substr(colon, end - colon));
}

bool has_type(const std::string & json, const std::string & t)
{
  return json.find("\"type\":\"" + t + "\"") != std::string::npos ||
         json.find("\"type\": \"" + t + "\"") != std::string::npos;
}

std::string wrap(const std::string & type, const std::string & payload)
{
  std::ostringstream ss;
  ss << "{\"type\":\"" << type << "\"," << payload << "}";
  return ss.str();
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string robot_ip = (argc > 1) ? argv[1] : "192.168.0.78";
  const int tcp_port = (argc > 2) ? std::atoi(argv[2]) : 9000;
  const int lidar_udp_port = (argc > 3) ? std::atoi(argv[3]) : 5601;
  const int ws_port = (argc > 4) ? std::atoi(argv[4]) : 8080;
  const std::string robot_id = (argc > 5) ? argv[5] : "tb_01";

  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);

  atrbridge_core::AtrBridgeCore core;
  if (!core.start(robot_ip, tcp_port, lidar_udp_port)) {
    std::cerr << "atrbridge_ws_gateway: failed to start core\n";
    return 1;
  }

  struct Latest
  {
    atrbridge_core::ImuSample imu{};
    atrbridge_core::IrSample ir{};
    atrbridge_core::TofSample tof{};
    int64_t imu_us{0};
    int64_t ir_us{0};
    int64_t tof_us{0};
  };

  std::mutex latest_mtx;
  Latest latest{};
  core.set_imu_callback([&](const atrbridge_core::ImuSample & s) {
    std::lock_guard<std::mutex> lk(latest_mtx);
    latest.imu = s;
    latest.imu_us++;
  });
  core.set_ir_callback([&](const atrbridge_core::IrSample & s) {
    std::lock_guard<std::mutex> lk(latest_mtx);
    latest.ir = s;
    latest.ir_us++;
  });
  core.set_tof_callback([&](const atrbridge_core::TofSample & s) {
    std::lock_guard<std::mutex> lk(latest_mtx);
    latest.tof = s;
    latest.tof_us++;
  });

  net::io_context ioc;
  tcp::acceptor acceptor(ioc, {tcp::v4(), static_cast<unsigned short>(ws_port)});
  beast::error_code ec;
  acceptor.non_blocking(true, ec);
  std::cout << "atrbridge_ws_gateway listening on ws://0.0.0.0:" << ws_port << "/ws\n";

  std::thread status_thread([&]() {
    bool prev_connected = false;
    bool set_id_sent = false;
    while (g_run.load()) {
      const auto caps = core.get_capabilities();
      const bool connected = caps.runtime.tcp_connected;
      if (connected != prev_connected) {
        std::cout << "[core] tcp_connected=" << (connected ? "true" : "false") << std::endl;
        prev_connected = connected;
      }
      if (!connected) {
        set_id_sent = false;
      } else if (!set_id_sent) {
        if (core.send_set_id(robot_id)) {
          set_id_sent = true;
          std::cout << "[core] SET_ID sent: " << robot_id << std::endl;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  });

  while (g_run.load()) {
    tcp::socket socket(ioc);
    acceptor.accept(socket, ec);
    if (ec) {
      if (ec == net::error::would_block || ec == net::error::try_again) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      continue;
    }

    websocket::stream<tcp::socket> ws(std::move(socket));
    ws.accept(ec);
    if (ec) {
      continue;
    }

    while (g_run.load()) {
      beast::flat_buffer buffer;
      ws.read(buffer, ec);
      if (ec) {
        break;
      }
      const std::string req = beast::buffers_to_string(buffer.data());

      if (has_type(req, "get_capabilities")) {
        const std::string payload = "\"capabilities\":" + core.get_capabilities_json();
        ws.write(net::buffer(wrap("capabilities", payload)), ec);
        if (ec) {
          break;
        }
        continue;
      }

      if (has_type(req, "cmd_vel")) {
        const double vx = parse_number(req, "vx").value_or(0.0);
        const double wz = parse_number(req, "wz").value_or(0.0);
        const bool ok = core.send_cmd_vel(static_cast<float>(vx), static_cast<float>(wz));
        ws.write(net::buffer(wrap("cmd_ack", std::string("\"ok\":") + (ok ? "true" : "false"))), ec);
        if (ec) {
          break;
        }
        continue;
      }

      if (has_type(req, "estop")) {
        const bool ok = core.send_cmd_vel(0.0f, 0.0f);
        ws.write(net::buffer(wrap("estop_ack", std::string("\"ok\":") + (ok ? "true" : "false"))), ec);
        if (ec) {
          break;
        }
        continue;
      }

      if (has_type(req, "poll")) {
        Latest snap{};
        {
          std::lock_guard<std::mutex> lk(latest_mtx);
          snap = latest;
        }
        std::ostringstream payload;
        payload << "\"imu\":{\"ax\":" << snap.imu.ax << ",\"ay\":" << snap.imu.ay << ",\"az\":" << snap.imu.az
                << ",\"gx\":" << snap.imu.gx << ",\"gy\":" << snap.imu.gy << ",\"gz\":" << snap.imu.gz << "},"
                << "\"ir\":{\"left\":" << snap.ir.left << ",\"right\":" << snap.ir.right << "},"
                << "\"tof\":{\"range_m\":" << snap.tof.range_m << "}";
        ws.write(net::buffer(wrap("telemetry", payload.str())), ec);
        if (ec) {
          break;
        }
        continue;
      }

      ws.write(net::buffer(wrap("error", "\"message\":\"unsupported_type\"")), ec);
      if (ec) {
        break;
      }
    }
  }

  if (status_thread.joinable()) {
    status_thread.join();
  }
  core.stop();
  return 0;
}
