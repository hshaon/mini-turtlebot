#include "atrbridge_core/core.hpp"

#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

namespace atrbridge_core
{

namespace
{

struct __attribute__((packed)) TelemetryCfgWire
{
  uint8_t enable_imu;
  float imu_rate_hz;
  uint8_t enable_ir;
  float ir_rate_hz;
  uint8_t enable_motor;
  float motor_rate_hz;
  uint8_t enable_batt;
  float batt_rate_hz;
  uint8_t enable_tof;
  float tof_rate_hz;
  uint8_t enable_lidar;
  float lidar_rate_hz;
  uint16_t lidar_udp_port;
};

struct __attribute__((packed)) LidarUdpHdr
{
  uint16_t magic;
  uint16_t seq;
  uint16_t count;
  uint16_t range_min_mm;
  uint16_t range_max_mm;
};

constexpr uint16_t kLidarMagic = 0x4C44;

}  // namespace

AtrBridgeCore::AtrBridgeCore()
{
  caps_.modules.lidar = true;
}

AtrBridgeCore::~AtrBridgeCore()
{
  stop();
}

bool AtrBridgeCore::start(const std::string & robot_ip, int tcp_port, int lidar_udp_port)
{
  stop();
  {
    std::lock_guard<std::mutex> lk(mtx_);
    caps_.runtime.tcp_port = tcp_port;
    caps_.runtime.lidar_udp_port = lidar_udp_port;
    caps_.runtime.tcp_connected = false;
    caps_.runtime.lidar_udp_active = false;
  }

  const bool tcp_ok = tcp_.start(
    robot_ip, tcp_port, [this](const Frame & f) {
      handle_frame_(f);
    });
  if (!tcp_ok) {
    return false;
  }
  start_lidar_udp_(lidar_udp_port);
  return true;
}

void AtrBridgeCore::stop()
{
  stop_lidar_udp_();
  tcp_.stop();
  std::lock_guard<std::mutex> lk(mtx_);
  caps_.runtime.tcp_connected = false;
}

bool AtrBridgeCore::send_cmd_vel(float vx, float wz)
{
  float v[6] = {vx, 0.0f, 0.0f, 0.0f, 0.0f, wz};
  const uint32_t s = seq_.fetch_add(1);
  auto bytes = encode_frame(
    MsgType::CMD_VEL, 0, s, reinterpret_cast<const uint8_t *>(v),
    static_cast<uint16_t>(sizeof(v)));
  return tcp_.send_bytes(bytes.data(), bytes.size());
}

bool AtrBridgeCore::send_set_id(const std::string & robot_id)
{
  const uint32_t s = seq_.fetch_add(1);
  auto bytes = encode_frame(
    MsgType::SET_ID, 0, s, reinterpret_cast<const uint8_t *>(robot_id.data()),
    static_cast<uint16_t>(robot_id.size()));
  if (tcp_.send_bytes(bytes.data(), bytes.size())) {
    std::lock_guard<std::mutex> lk(mtx_);
    caps_.runtime.robot_id = robot_id;
    return true;
  }
  return false;
}

bool AtrBridgeCore::send_telemetry_config(const TelemetryConfig & cfg)
{
  TelemetryCfgWire w{};
  w.enable_imu = cfg.enable_imu ? 1 : 0;
  w.imu_rate_hz = cfg.imu_rate_hz;
  w.enable_ir = cfg.enable_ir ? 1 : 0;
  w.ir_rate_hz = cfg.ir_rate_hz;
  w.enable_motor = cfg.enable_motor_state ? 1 : 0;
  w.motor_rate_hz = cfg.motor_state_rate_hz;
  w.enable_batt = cfg.enable_battery ? 1 : 0;
  w.batt_rate_hz = cfg.battery_rate_hz;
  w.enable_tof = cfg.enable_tof ? 1 : 0;
  w.tof_rate_hz = cfg.tof_rate_hz;
  w.enable_lidar = cfg.enable_lidar ? 1 : 0;
  w.lidar_rate_hz = cfg.lidar_rate_hz;
  w.lidar_udp_port = cfg.lidar_udp_port;

  const uint32_t s = seq_.fetch_add(1);
  auto bytes = encode_frame(
    MsgType::SET_TELEMETRY, 0, s, reinterpret_cast<const uint8_t *>(&w),
    static_cast<uint16_t>(sizeof(w)));
  return tcp_.send_bytes(bytes.data(), bytes.size());
}

void AtrBridgeCore::set_imu_callback(ImuCallback cb)
{
  std::lock_guard<std::mutex> lk(mtx_);
  imu_cb_ = std::move(cb);
}

void AtrBridgeCore::set_ir_callback(IrCallback cb)
{
  std::lock_guard<std::mutex> lk(mtx_);
  ir_cb_ = std::move(cb);
}

void AtrBridgeCore::set_tof_callback(TofCallback cb)
{
  std::lock_guard<std::mutex> lk(mtx_);
  tof_cb_ = std::move(cb);
}

void AtrBridgeCore::set_lidar_callback(LidarCallback cb)
{
  std::lock_guard<std::mutex> lk(mtx_);
  lidar_cb_ = std::move(cb);
}

void AtrBridgeCore::set_ack_callback(AckCallback cb)
{
  std::lock_guard<std::mutex> lk(mtx_);
  ack_cb_ = std::move(cb);
}

Capabilities AtrBridgeCore::get_capabilities() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  Capabilities c = caps_;
  c.runtime.tcp_connected = tcp_.is_connected();
  return c;
}

std::string AtrBridgeCore::get_capabilities_json() const
{
  const Capabilities c = get_capabilities();
  std::ostringstream ss;
  ss << "{";
  ss << "\"modules\":{";
  ss << "\"base\":" << (c.modules.base ? "true" : "false") << ",";
  ss << "\"imu\":" << (c.modules.imu ? "true" : "false") << ",";
  ss << "\"ir\":" << (c.modules.ir ? "true" : "false") << ",";
  ss << "\"tof\":" << (c.modules.tof ? "true" : "false") << ",";
  ss << "\"lidar\":" << (c.modules.lidar ? "true" : "false") << ",";
  ss << "\"camera\":" << (c.modules.camera ? "true" : "false") << ",";
  ss << "\"arm\":" << (c.modules.arm ? "true" : "false");
  ss << "},";
  ss << "\"limits\":{";
  ss << "\"max_vx_mps\":" << c.limits.max_vx_mps << ",";
  ss << "\"max_wz_rps\":" << c.limits.max_wz_rps << ",";
  ss << "\"watchdog_cmd_timeout_ms\":" << c.limits.watchdog_cmd_timeout_ms;
  ss << "},";
  ss << "\"runtime\":{";
  ss << "\"tcp_connected\":" << (c.runtime.tcp_connected ? "true" : "false") << ",";
  ss << "\"lidar_udp_active\":" << (c.runtime.lidar_udp_active ? "true" : "false") << ",";
  ss << "\"rx_frames\":" << c.runtime.rx_frames << ",";
  ss << "\"rx_drops\":" << c.runtime.rx_drops << ",";
  ss << "\"rx_lidar_packets\":" << c.runtime.rx_lidar_packets << ",";
  ss << "\"last_tcp_rx_us\":" << c.runtime.last_tcp_rx_us << ",";
  ss << "\"last_lidar_rx_us\":" << c.runtime.last_lidar_rx_us << ",";
  ss << "\"tcp_port\":" << c.runtime.tcp_port << ",";
  ss << "\"lidar_udp_port\":" << c.runtime.lidar_udp_port << ",";
  ss << "\"robot_id\":\"" << c.runtime.robot_id << "\"";
  ss << "}";
  ss << "}";
  return ss.str();
}

void AtrBridgeCore::handle_frame_(const Frame & f)
{
  ImuCallback imu_cb;
  IrCallback ir_cb;
  TofCallback tof_cb;
  AckCallback ack_cb;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    caps_.runtime.tcp_connected = tcp_.is_connected();
    caps_.runtime.rx_frames++;
    caps_.runtime.last_tcp_rx_us = now_us_();
    imu_cb = imu_cb_;
    ir_cb = ir_cb_;
    tof_cb = tof_cb_;
    ack_cb = ack_cb_;
  }

  if (f.type == MsgType::ACK) {
    if (ack_cb) {
      ack_cb(f.seq);
    }
    return;
  }

  if (f.type == MsgType::IMU) {
    if (f.payload.size() != 6 * sizeof(float)) {
      std::lock_guard<std::mutex> lk(mtx_);
      caps_.runtime.rx_drops++;
      return;
    }
    ImuSample s{};
    float v[6];
    std::memcpy(v, f.payload.data(), sizeof(v));
    s.ax = v[0];
    s.ay = v[1];
    s.az = v[2];
    s.gx = v[3];
    s.gy = v[4];
    s.gz = v[5];
    if (imu_cb) {
      imu_cb(s);
    }
    return;
  }

  if (f.type == MsgType::IR) {
    if (f.payload.size() != 2 * sizeof(float)) {
      std::lock_guard<std::mutex> lk(mtx_);
      caps_.runtime.rx_drops++;
      return;
    }
    IrSample s{};
    float v[2];
    std::memcpy(v, f.payload.data(), sizeof(v));
    s.left = v[0];
    s.right = v[1];
    if (ir_cb) {
      ir_cb(s);
    }
    return;
  }

  if (f.type == MsgType::TOF) {
    if (f.payload.size() != sizeof(float)) {
      std::lock_guard<std::mutex> lk(mtx_);
      caps_.runtime.rx_drops++;
      return;
    }
    TofSample s{};
    std::memcpy(&s.range_m, f.payload.data(), sizeof(float));
    if (tof_cb) {
      tof_cb(s);
    }
    return;
  }
}

void AtrBridgeCore::start_lidar_udp_(int port)
{
  stop_lidar_udp_();

  lidar_sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (lidar_sock_ < 0) {
    return;
  }

  int one = 1;
  (void)::setsockopt(lidar_sock_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  struct sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (::bind(lidar_sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(lidar_sock_);
    lidar_sock_ = -1;
    return;
  }

  lidar_run_.store(true);
  {
    std::lock_guard<std::mutex> lk(mtx_);
    caps_.runtime.lidar_udp_active = true;
  }
  lidar_thread_ = std::thread([this]() {lidar_udp_thread_();});
}

void AtrBridgeCore::stop_lidar_udp_()
{
  lidar_run_.store(false);
  if (lidar_sock_ >= 0) {
    ::shutdown(lidar_sock_, SHUT_RDWR);
    ::close(lidar_sock_);
    lidar_sock_ = -1;
  }
  if (lidar_thread_.joinable()) {
    lidar_thread_.join();
  }
  std::lock_guard<std::mutex> lk(mtx_);
  caps_.runtime.lidar_udp_active = false;
}

void AtrBridgeCore::lidar_udp_thread_()
{
  std::vector<uint8_t> buf(4096);
  while (lidar_run_.load()) {
    const ssize_t n = ::recvfrom(lidar_sock_, buf.data(), buf.size(), 0, nullptr, nullptr);
    if (n <= 0) {
      continue;
    }
    if (n < static_cast<ssize_t>(sizeof(LidarUdpHdr))) {
      std::lock_guard<std::mutex> lk(mtx_);
      caps_.runtime.rx_drops++;
      continue;
    }

    LidarUdpHdr hdr{};
    std::memcpy(&hdr, buf.data(), sizeof(hdr));
    if (hdr.magic != kLidarMagic) {
      std::lock_guard<std::mutex> lk(mtx_);
      caps_.runtime.rx_drops++;
      continue;
    }

    const size_t needed = sizeof(LidarUdpHdr) + static_cast<size_t>(hdr.count) * 2U;
    if (static_cast<size_t>(n) < needed) {
      std::lock_guard<std::mutex> lk(mtx_);
      caps_.runtime.rx_drops++;
      continue;
    }

    LidarSample s{};
    s.seq = hdr.seq;
    s.count = hdr.count;
    s.range_min_mm = hdr.range_min_mm;
    s.range_max_mm = hdr.range_max_mm;
    s.ranges_mm.resize(hdr.count);
    for (size_t i = 0; i < hdr.count; ++i) {
      const size_t o = sizeof(LidarUdpHdr) + i * 2U;
      s.ranges_mm[i] = static_cast<uint16_t>(buf[o]) |
        (static_cast<uint16_t>(buf[o + 1]) << 8);
    }

    LidarCallback lidar_cb;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      caps_.runtime.rx_lidar_packets++;
      caps_.runtime.last_lidar_rx_us = now_us_();
      lidar_cb = lidar_cb_;
    }
    if (lidar_cb) {
      lidar_cb(s);
    }
  }
}

int64_t AtrBridgeCore::now_us_() const
{
  using namespace std::chrono;
  return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

}  // namespace atrbridge_core

