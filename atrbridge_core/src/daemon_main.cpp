#include "atrbridge_core/core.hpp"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>

namespace
{
std::atomic<bool> g_run{true};

void on_sigint(int)
{
  g_run.store(false);
}

const char * arg_or_default(int argc, char ** argv, int idx, const char * def)
{
  if (idx < argc) {
    return argv[idx];
  }
  return def;
}
}  // namespace

int main(int argc, char ** argv)
{
  const std::string robot_ip = arg_or_default(argc, argv, 1, "192.168.0.78");
  const int tcp_port = std::atoi(arg_or_default(argc, argv, 2, "9000"));
  const int lidar_udp_port = std::atoi(arg_or_default(argc, argv, 3, "5601"));

  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);

  atrbridge_core::AtrBridgeCore core;
  if (!core.start(robot_ip, tcp_port, lidar_udp_port)) {
    std::cerr << "atrbridge_core_daemon: failed to start core\n";
    return 1;
  }

  core.send_set_id("tb_01");

  std::cout << "atrbridge_core_daemon started: robot_ip=" << robot_ip
            << " tcp_port=" << tcp_port
            << " lidar_udp_port=" << lidar_udp_port << "\n";

  while (g_run.load()) {
    std::cout << core.get_capabilities_json() << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  core.stop();
  return 0;
}

