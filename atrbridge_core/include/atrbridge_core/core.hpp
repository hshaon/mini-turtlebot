#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "atrbridge_core/framing.hpp"
#include "atrbridge_core/transport_tcp.hpp"

namespace atrbridge_core
{

struct TelemetryConfig
{
  bool enable_imu{true};
  float imu_rate_hz{50.0f};
  bool enable_ir{true};
  float ir_rate_hz{10.0f};
  bool enable_motor_state{false};
  float motor_state_rate_hz{0.0f};
  bool enable_battery{false};
  float battery_rate_hz{0.0f};
  bool enable_tof{true};
  float tof_rate_hz{10.0f};
  bool enable_lidar{false};
  float lidar_rate_hz{5.0f};
  uint16_t lidar_udp_port{5601};
};

struct ImuSample
{
  float ax{0.0f};
  float ay{0.0f};
  float az{0.0f};
  float gx{0.0f};
  float gy{0.0f};
  float gz{0.0f};
};

struct IrSample
{
  float left{0.0f};
  float right{0.0f};
};

struct TofSample
{
  float range_m{0.0f};
};

struct LidarSample
{
  uint16_t seq{0};
  uint16_t count{0};
  uint16_t range_min_mm{0};
  uint16_t range_max_mm{0};
  std::vector<uint16_t> ranges_mm;
};

struct Capabilities
{
  struct Modules
  {
    bool base{true};
    bool imu{true};
    bool ir{true};
    bool tof{true};
    bool lidar{false};
    bool camera{false};
    bool arm{false};
  } modules;

  struct Limits
  {
    float max_vx_mps{0.25f};
    float max_wz_rps{1.0f};
    int watchdog_cmd_timeout_ms{500};
  } limits;

  struct Runtime
  {
    bool tcp_connected{false};
    bool lidar_udp_active{false};
    uint64_t rx_frames{0};
    uint64_t rx_drops{0};
    uint64_t rx_lidar_packets{0};
    int64_t last_tcp_rx_us{0};
    int64_t last_lidar_rx_us{0};
    int tcp_port{9000};
    int lidar_udp_port{5601};
    std::string robot_id{"tb_01"};
  } runtime;
};

class AtrBridgeCore
{
public:
  using ImuCallback = std::function<void(const ImuSample &)>;
  using IrCallback = std::function<void(const IrSample &)>;
  using TofCallback = std::function<void(const TofSample &)>;
  using LidarCallback = std::function<void(const LidarSample &)>;
  using AckCallback = std::function<void(uint32_t seq)>;

  AtrBridgeCore();
  ~AtrBridgeCore();

  bool start(const std::string & robot_ip, int tcp_port, int lidar_udp_port);
  void stop();

  bool send_cmd_vel(float vx, float wz);
  bool send_set_id(const std::string & robot_id);
  bool send_telemetry_config(const TelemetryConfig & cfg);

  void set_imu_callback(ImuCallback cb);
  void set_ir_callback(IrCallback cb);
  void set_tof_callback(TofCallback cb);
  void set_lidar_callback(LidarCallback cb);
  void set_ack_callback(AckCallback cb);

  Capabilities get_capabilities() const;
  std::string get_capabilities_json() const;

private:
  void handle_frame_(const Frame & f);
  void start_lidar_udp_(int port);
  void stop_lidar_udp_();
  void lidar_udp_thread_();
  int64_t now_us_() const;

  mutable std::mutex mtx_;
  Capabilities caps_;
  TcpTransport tcp_;
  std::atomic<uint32_t> seq_{1};

  int lidar_sock_{-1};
  std::thread lidar_thread_;
  std::atomic<bool> lidar_run_{false};

  ImuCallback imu_cb_;
  IrCallback ir_cb_;
  TofCallback tof_cb_;
  LidarCallback lidar_cb_;
  AckCallback ack_cb_;
};

}  // namespace atrbridge_core

