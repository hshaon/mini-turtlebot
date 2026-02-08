#pragma once

#include <memory>
#include <string>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tb_interfaces/srv/set_telemetry.hpp"
#include "tb_interfaces/srv/get_link_stats.hpp"
#include "tb_bridge_cpp/framing.hpp"
#include "tb_bridge_cpp/transport_tcp.hpp"


namespace tb_bridge_cpp
{

struct TelemetryConfig
{
  bool enable_imu{false};
  float imu_rate_hz{0.0f};

  bool enable_ir{false};
  float ir_rate_hz{0.0f};

  bool enable_motor_state{false};
  float motor_state_rate_hz{0.0f};

  bool enable_battery{false};
  float battery_rate_hz{0.0f};

  bool enable_lidar360{false};
  float lidar360_rate_hz{0.0f};

  std::string profile_name;
};

struct LinkStats
{
  std::atomic<uint32_t> reconnect_count{0};

  std::atomic<float> rtt_ms_p50{0.0f};
  std::atomic<float> rtt_ms_p95{0.0f};
  std::atomic<float> rtt_ms_p99{0.0f};

  std::atomic<uint32_t> tx_dropped{0};
  std::atomic<uint32_t> rx_dropped{0};
  std::atomic<uint32_t> tx_queue_depth{0};
  std::atomic<uint32_t> rx_queue_depth{0};

  std::string transport_mode{"stub"};
  std::string status{"not_connected"};
};

class BridgeNode : public rclcpp::Node
{
public:
  explicit BridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ROS callbacks
  void on_cmd_vel(const geometry_msgs::msg::Twist & msg);

  void on_set_telemetry(
    const std::shared_ptr<tb_interfaces::srv::SetTelemetry::Request> req,
    std::shared_ptr<tb_interfaces::srv::SetTelemetry::Response> resp);

  void on_get_link_stats(
    const std::shared_ptr<tb_interfaces::srv::GetLinkStats::Request> req,
    std::shared_ptr<tb_interfaces::srv::GetLinkStats::Response> resp);

  // Helpers
  std::string current_namespace() const;
  std::string make_ws_url() const;

  // Parameters
  std::string robot_id_;
  std::string ws_url_;
  std::string robot_ip_;
  int port_{9000};
  std::string path_{"/ws"};

  std::string transport_mode_{"ws_jsonl"};
  int cmd_timeout_ms_{500};
  double send_rate_hz_{20.0};

  // State
  TelemetryConfig telemetry_;
  LinkStats stats_;

  // Transport
  std::unique_ptr<TcpTransport> tcp_;
  std::atomic<uint32_t> seq_{1};


  // ROS entities
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Service<tb_interfaces::srv::SetTelemetry>::SharedPtr srv_set_telemetry_;
  rclcpp::Service<tb_interfaces::srv::GetLinkStats>::SharedPtr srv_get_link_stats_;
};

}  // namespace tb_bridge_cpp

