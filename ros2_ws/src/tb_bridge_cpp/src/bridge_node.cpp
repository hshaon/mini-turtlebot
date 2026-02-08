#include "tb_bridge_cpp/bridge_node.hpp"

#include <sstream>
#include <unordered_map>
#include <chrono>
#include <mutex>

rclcpp::TimerBase::SharedPtr hb_timer_;
std::mutex rtt_mtx_;
std::unordered_map<uint32_t, std::chrono::steady_clock::time_point> pending_;

namespace tb_bridge_cpp
{

BridgeNode::BridgeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("tb_bridge_node", options)
{
  // ---- Declare params (swarm-friendly) ----
  robot_id_ = this->declare_parameter<std::string>("robot_id", "tb_01");
  ws_url_   = this->declare_parameter<std::string>("ws_url", "");
  robot_ip_ = this->declare_parameter<std::string>("robot_ip", "192.168.0.78");
  port_     = this->declare_parameter<int>("port", 9000);
  path_     = this->declare_parameter<std::string>("path", "/ws");

  transport_mode_ = this->declare_parameter<std::string>("transport_mode", "ws_jsonl");
  cmd_timeout_ms_ = this->declare_parameter<int>("cmd_timeout_ms", 500);
  send_rate_hz_   = this->declare_parameter<double>("send_rate_hz", 20.0);

  // Update stats visible via service
  stats_.transport_mode = transport_mode_;
  stats_.status = "starting";

  // ---- Topics in namespace ----
  // Using relative topic names lets ROS namespace remapping handle /tb_01 automatically.
  sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    rclcpp::QoS(10),
    [this](const geometry_msgs::msg::Twist & msg) { this->on_cmd_vel(msg); });

  // ---- Services in namespace ----
  srv_set_telemetry_ = this->create_service<tb_interfaces::srv::SetTelemetry>(
    "bridge/set_telemetry",
    [this](
      const std::shared_ptr<tb_interfaces::srv::SetTelemetry::Request> req,
      std::shared_ptr<tb_interfaces::srv::SetTelemetry::Response> resp)
    {
      this->on_set_telemetry(req, resp);
    });

  srv_get_link_stats_ = this->create_service<tb_interfaces::srv::GetLinkStats>(
    "bridge/get_link_stats",
    [this](
      const std::shared_ptr<tb_interfaces::srv::GetLinkStats::Request> req,
      std::shared_ptr<tb_interfaces::srv::GetLinkStats::Response> resp)
    {
      this->on_get_link_stats(req, resp);
    });

  if (transport_mode_ == "tcp_bin") {
    tcp_ = std::make_unique<TcpTransport>();
    stats_.transport_mode = "tcp_bin";
    stats_.status = "connecting";

  tcp_->start(robot_ip_, port_, [this](const Frame& f) {
    if (f.type == MsgType::ACK) {
      auto now = std::chrono::steady_clock::now();
      std::chrono::steady_clock::time_point t0;
      bool found = false;

      {
        std::lock_guard<std::mutex> lk(rtt_mtx_);
        auto it = pending_.find(f.seq);
        if (it != pending_.end()) {
          t0 = it->second;
          pending_.erase(it);
          found = true;
        }
      }

      if (found) {
        auto rtt_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(now - t0).count();
        // TODO: feed into p50/p95/p99 estimator
        // For now just store as p50 to prove it works
        stats_.rtt_ms_p50.store((float)rtt_ms);
      }
    }
  });

    RCLCPP_INFO(this->get_logger(), "TCP transport started: %s:%d", robot_ip_.c_str(), port_);
  } else {
    // Still stub for ws_jsonl
    stats_.transport_mode = transport_mode_;
    stats_.status = "ready_stub";
    RCLCPP_INFO(this->get_logger(), "target_url=%s", make_ws_url().c_str());
  }

  hb_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      if (!(tcp_ && tcp_->is_connected())) return;

      uint32_t s = seq_.fetch_add(1);
      auto now = std::chrono::steady_clock::now();

      {
        std::lock_guard<std::mutex> lk(rtt_mtx_);
        pending_[s] = now;
        // prevent unbounded growth
        if (pending_.size() > 2000) pending_.erase(pending_.begin());
      }

      auto bytes = encode_frame(MsgType::HEARTBEAT, 0, s, nullptr, 0);
      if (!tcp_->send_bytes(bytes.data(), bytes.size())) {
        stats_.tx_dropped.fetch_add(1);
      }
    }
  );

  // Log resolved URL for sanity
  const auto url = make_ws_url();
  RCLCPP_INFO(this->get_logger(), "robot_id=%s namespace=%s", robot_id_.c_str(), current_namespace().c_str());
  RCLCPP_INFO(this->get_logger(), "transport_mode=%s", transport_mode_.c_str());
  RCLCPP_INFO(this->get_logger(), "target_url=%s", url.c_str());

  stats_.status = "ready_stub";
}

void BridgeNode::on_cmd_vel(const geometry_msgs::msg::Twist & msg)
{
  // Payload: 6 floats (vx, vy, vz, wx, wy, wz) as float32 LE
  // For turtlebot you likely use vx and wz; keep full for extensibility.
  float v[6] = {
    static_cast<float>(msg.linear.x),
    static_cast<float>(msg.linear.y),
    static_cast<float>(msg.linear.z),
    static_cast<float>(msg.angular.x),
    static_cast<float>(msg.angular.y),
    static_cast<float>(msg.angular.z),
  };

  uint32_t s = seq_.fetch_add(1);

  auto bytes = encode_frame(
    MsgType::CMD_VEL,
    0,
    s,
    reinterpret_cast<const uint8_t*>(v),
    static_cast<uint16_t>(sizeof(v))
  );

  if (tcp_ && tcp_->is_connected()) {
    bool ok = tcp_->send_bytes(bytes.data(), bytes.size());
    if (!ok) {
      stats_.tx_dropped.fetch_add(1);
    }
  } else {
    stats_.tx_dropped.fetch_add(1);
  }
}


void BridgeNode::on_set_telemetry(
  const std::shared_ptr<tb_interfaces::srv::SetTelemetry::Request> req,
  std::shared_ptr<tb_interfaces::srv::SetTelemetry::Response> resp)
{
  // 1) Update local state from ROS request
  telemetry_.enable_imu = req->enable_imu;
  telemetry_.imu_rate_hz = req->imu_rate_hz;

  telemetry_.enable_ir = req->enable_ir;
  telemetry_.ir_rate_hz = req->ir_rate_hz;

  telemetry_.enable_motor_state = req->enable_motor_state;
  telemetry_.motor_state_rate_hz = req->motor_state_rate_hz;

  telemetry_.enable_battery = req->enable_battery;
  telemetry_.battery_rate_hz = req->battery_rate_hz;

  telemetry_.enable_lidar360 = req->enable_lidar360;
  telemetry_.lidar360_rate_hz = req->lidar360_rate_hz;

  telemetry_.profile_name = req->profile_name;

  // 2) Build binary config payload and send to robot (TCP)
  struct __attribute__((packed)) Cfg {
    uint8_t enable_imu;
    float   imu_rate_hz;
    uint8_t enable_ir;
    float   ir_rate_hz;
    uint8_t enable_motor;
    float   motor_rate_hz;
    uint8_t enable_batt;
    float   batt_rate_hz;
    uint8_t enable_lidar;
    float   lidar_rate_hz;
  } cfg;

  cfg.enable_imu    = telemetry_.enable_imu ? 1 : 0;
  cfg.imu_rate_hz   = telemetry_.imu_rate_hz;
  cfg.enable_ir     = telemetry_.enable_ir ? 1 : 0;
  cfg.ir_rate_hz    = telemetry_.ir_rate_hz;
  cfg.enable_motor  = telemetry_.enable_motor_state ? 1 : 0;
  cfg.motor_rate_hz = telemetry_.motor_state_rate_hz;
  cfg.enable_batt   = telemetry_.enable_battery ? 1 : 0;
  cfg.batt_rate_hz  = telemetry_.battery_rate_hz;
  cfg.enable_lidar  = telemetry_.enable_lidar360 ? 1 : 0;
  cfg.lidar_rate_hz = telemetry_.lidar360_rate_hz;

  uint32_t s = seq_.fetch_add(1);

  auto bytes = encode_frame(
    MsgType::SET_TELEMETRY,
    0,
    s,
    reinterpret_cast<const uint8_t*>(&cfg),
    static_cast<uint16_t>(sizeof(cfg))
  );

  bool sent_ok = false;
  if (tcp_ && tcp_->is_connected()) {
    sent_ok = tcp_->send_bytes(bytes.data(), bytes.size());
  }

  if (!sent_ok) {
    stats_.tx_dropped.fetch_add(1);
  }

  // 3) Respond to ROS service caller
  resp->accepted = true;
  resp->status = sent_ok ? "applied (binary config sent)" : "applied (not connected; config not sent)";
}


void BridgeNode::on_get_link_stats(
  const std::shared_ptr<tb_interfaces::srv::GetLinkStats::Request> req,
  std::shared_ptr<tb_interfaces::srv::GetLinkStats::Response> resp)
{
  (void)req;
  resp->reconnect_count = stats_.reconnect_count.load();
  resp->rtt_ms_p50 = stats_.rtt_ms_p50.load();
  resp->rtt_ms_p95 = stats_.rtt_ms_p95.load();
  resp->rtt_ms_p99 = stats_.rtt_ms_p99.load();

  resp->tx_dropped = stats_.tx_dropped.load();
  resp->rx_dropped = stats_.rx_dropped.load();
  resp->tx_queue_depth = stats_.tx_queue_depth.load();
  resp->rx_queue_depth = stats_.rx_queue_depth.load();

  resp->transport_mode = stats_.transport_mode;
  resp->status = stats_.status;
}

std::string BridgeNode::current_namespace() const
{
  // Returns node namespace like "/tb_01" if launched with __ns remap
  return this->get_namespace();
}

std::string BridgeNode::make_ws_url() const
{
  if (!ws_url_.empty()) {
    return ws_url_;
  }
  std::ostringstream ss;
  ss << "ws://" << robot_ip_ << ":" << port_ << path_;
  return ss.str();
}

}  // namespace tb_bridge_cpp

