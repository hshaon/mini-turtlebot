#include "tb_bridge_cpp/bridge_node.hpp"

#include <sstream>
#include <unordered_map>
#include <chrono>
#include <mutex>
#include <vector>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <limits>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "rcl_interfaces/msg/set_parameters_result.hpp"

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

  telemetry_.enable_imu = this->declare_parameter<bool>("enable_imu", false);
  telemetry_.imu_rate_hz = this->declare_parameter<double>("imu_rate_hz", 50.0);
  telemetry_.enable_ir = this->declare_parameter<bool>("enable_ir", false);
  telemetry_.ir_rate_hz = this->declare_parameter<double>("ir_rate_hz", 10.0);
  telemetry_.enable_tof = this->declare_parameter<bool>("enable_tof", false);
  telemetry_.tof_rate_hz = this->declare_parameter<double>("tof_rate_hz", 10.0);
  telemetry_.enable_lidar = this->declare_parameter<bool>("enable_lidar", false);
  telemetry_.lidar_rate_hz = this->declare_parameter<double>("lidar_rate_hz", 1.0);

  lidar_transport_ = this->declare_parameter<std::string>("lidar_transport", "udp");
  lidar_udp_port_ = this->declare_parameter<int>("lidar_udp_port", 5601);
  if (lidar_udp_port_ < 1) lidar_udp_port_ = 1;
  if (lidar_udp_port_ > 65535) lidar_udp_port_ = 65535;
  telemetry_.lidar_udp_port = static_cast<uint16_t>(lidar_udp_port_);

  bench_enable_ = this->declare_parameter<bool>("bench_enable", true);
  bench_hz_ = this->declare_parameter<double>("bench_heartbeat_hz", 1.0);
  bench_window_ = this->declare_parameter<int>("bench_window_size", 200);
  if (bench_window_ < 20) bench_window_ = 20;
  if (bench_window_ > 2000) bench_window_ = 2000;

  // Update stats visible via service
  stats_.transport_mode = transport_mode_;
  stats_.status = "starting";

  // ---- Topics in namespace ----
  // Using relative topic names lets ROS namespace remapping handle /tb_01 automatically.
  sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    rclcpp::QoS(10),
    [this](const geometry_msgs::msg::Twist & msg) { this->on_cmd_vel(msg); });

  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::QoS(20));
  pub_ir_ = this->create_publisher<tb_interfaces::msg::IRState>("ir/state", rclcpp::QoS(10));
  pub_tof_ = this->create_publisher<sensor_msgs::msg::Range>("tof/range", rclcpp::QoS(10));
  pub_lidar_ = this->create_publisher<sensor_msgs::msg::LaserScan>("lidar/scan", rclcpp::QoS(10));

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
        handle_ack_(f.seq);
        return;
      }

      if (f.type == MsgType::IMU) {
        if (f.payload.size() != 6 * sizeof(float)) {
          stats_.rx_dropped.fetch_add(1);
          return;
        }
        float v[6];
        std::memcpy(v, f.payload.data(), sizeof(v));

        sensor_msgs::msg::Imu msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";

        msg.linear_acceleration.x = v[0];
        msg.linear_acceleration.y = v[1];
        msg.linear_acceleration.z = v[2];

        msg.angular_velocity.x = v[3];
        msg.angular_velocity.y = v[4];
        msg.angular_velocity.z = v[5];

        msg.orientation_covariance[0] = -1.0;

        pub_imu_->publish(msg);
        return;
      }

      if (f.type == MsgType::IR) {
        if (f.payload.size() != 2 * sizeof(float)) {
          stats_.rx_dropped.fetch_add(1);
          return;
        }
        float v[2];
        std::memcpy(v, f.payload.data(), sizeof(v));

        tb_interfaces::msg::IRState msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "ir_link";
        msg.left = v[0];
        msg.right = v[1];

        pub_ir_->publish(msg);
        return;
      }

      if (f.type == MsgType::TOF) {
        if (f.payload.size() != sizeof(float)) {
          stats_.rx_dropped.fetch_add(1);
          return;
        }
        float range_m = 0.0f;
        std::memcpy(&range_m, f.payload.data(), sizeof(range_m));

        sensor_msgs::msg::Range msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "tof_link";
        msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
        msg.field_of_view = 0.44f;
        msg.min_range = 0.03f;
        msg.max_range = 2.0f;
        msg.range = range_m;

        pub_tof_->publish(msg);
        return;
      }
    });

    RCLCPP_INFO(this->get_logger(), "TCP transport started: %s:%d", robot_ip_.c_str(), port_);
  } else {
    // Still stub for ws_jsonl
    stats_.transport_mode = transport_mode_;
    stats_.status = "ready_stub";
    RCLCPP_INFO(this->get_logger(), "target_url=%s", make_ws_url().c_str());
  }

  param_cb_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& params) {
      bool new_enable = bench_enable_;
      double new_hz = bench_hz_;
      int new_window = bench_window_;
      bool telemetry_changed = false;
      bool lidar_udp_changed = false;
      for (const auto& p : params) {
        if (p.get_name() == "bench_enable") new_enable = p.as_bool();
        if (p.get_name() == "bench_heartbeat_hz") new_hz = p.as_double();
        if (p.get_name() == "bench_window_size") new_window = p.as_int();

        if (p.get_name() == "enable_imu") { telemetry_.enable_imu = p.as_bool(); telemetry_changed = true; }
        if (p.get_name() == "imu_rate_hz") { telemetry_.imu_rate_hz = p.as_double(); telemetry_changed = true; }
        if (p.get_name() == "enable_ir") { telemetry_.enable_ir = p.as_bool(); telemetry_changed = true; }
        if (p.get_name() == "ir_rate_hz") { telemetry_.ir_rate_hz = p.as_double(); telemetry_changed = true; }
        if (p.get_name() == "enable_tof") { telemetry_.enable_tof = p.as_bool(); telemetry_changed = true; }
        if (p.get_name() == "tof_rate_hz") { telemetry_.tof_rate_hz = p.as_double(); telemetry_changed = true; }
        if (p.get_name() == "enable_lidar") { telemetry_.enable_lidar = p.as_bool(); telemetry_changed = true; }
        if (p.get_name() == "lidar_rate_hz") { telemetry_.lidar_rate_hz = p.as_double(); telemetry_changed = true; }
        if (p.get_name() == "lidar_transport") { lidar_transport_ = p.as_string(); lidar_udp_changed = true; }
        if (p.get_name() == "lidar_udp_port") {
          int port = p.as_int();
          if (port < 1) port = 1;
          if (port > 65535) port = 65535;
          lidar_udp_port_ = port;
          telemetry_.lidar_udp_port = static_cast<uint16_t>(lidar_udp_port_);
          lidar_udp_changed = true;
          telemetry_changed = true;
        }
      }
      if (new_window < 20) new_window = 20;
      if (new_window > 2000) new_window = 2000;

      {
        std::lock_guard<std::mutex> lk(bench_mtx_);
        bench_enable_ = new_enable;
        bench_hz_ = new_hz;
        bench_window_ = new_window;
      }

      start_or_stop_benchmark_timer_();

      if (lidar_udp_changed) {
        start_or_stop_lidar_udp_();
      }

      if (telemetry_changed && !telemetry_param_update_) {
        send_telemetry_config_();
      }

      rcl_interfaces::msg::SetParametersResult r;
      r.successful = true;
      r.reason = "ok";
      return r;
    }
  );

  start_or_stop_benchmark_timer_();
  start_or_stop_lidar_udp_();

  if (transport_mode_ == "tcp_bin") {
    set_id_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        if (!(tcp_ && tcp_->is_connected())) {
          set_id_sent_ = false;
          return;
        }
        if (set_id_sent_) return;
        send_set_id_();
        set_id_sent_ = true;
      }
    );
  }

  // Log resolved URL for sanity
  const auto url = make_ws_url();
  RCLCPP_INFO(this->get_logger(), "robot_id=%s namespace=%s", robot_id_.c_str(), current_namespace().c_str());
  RCLCPP_INFO(this->get_logger(), "transport_mode=%s", transport_mode_.c_str());
  RCLCPP_INFO(this->get_logger(), "target_url=%s", url.c_str());

  stats_.status = "ready_stub";
}

BridgeNode::~BridgeNode()
{
  lidar_udp_run_.store(false);
  if (lidar_udp_sock_ >= 0) {
    ::shutdown(lidar_udp_sock_, SHUT_RDWR);
    ::close(lidar_udp_sock_);
    lidar_udp_sock_ = -1;
  }
  if (lidar_udp_thread_.joinable()) {
    lidar_udp_thread_.join();
  }
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

  telemetry_.enable_tof = req->enable_tof;
  telemetry_.tof_rate_hz = req->tof_rate_hz;

  telemetry_.enable_lidar = req->enable_lidar;
  telemetry_.lidar_rate_hz = req->lidar_rate_hz;
  telemetry_.lidar_udp_port = req->lidar_udp_port;
  if (telemetry_.lidar_udp_port == 0) {
    telemetry_.lidar_udp_port = 5601;
  }
  lidar_udp_port_ = telemetry_.lidar_udp_port;

  telemetry_.profile_name = req->profile_name;

  // 2) Build binary config payload and send to robot (TCP)
  bool sent_ok = send_telemetry_config_();

  // 3) Update params to reflect current config (best-effort)
  std::vector<rclcpp::Parameter> ps;
  ps.emplace_back("enable_imu", telemetry_.enable_imu);
  ps.emplace_back("imu_rate_hz", telemetry_.imu_rate_hz);
  ps.emplace_back("enable_ir", telemetry_.enable_ir);
  ps.emplace_back("ir_rate_hz", telemetry_.ir_rate_hz);
  ps.emplace_back("enable_tof", telemetry_.enable_tof);
  ps.emplace_back("tof_rate_hz", telemetry_.tof_rate_hz);
  ps.emplace_back("enable_lidar", telemetry_.enable_lidar);
  ps.emplace_back("lidar_rate_hz", telemetry_.lidar_rate_hz);
  ps.emplace_back("lidar_udp_port", static_cast<int>(telemetry_.lidar_udp_port));
  telemetry_param_update_ = true;
  (void)this->set_parameters(ps);
  telemetry_param_update_ = false;

  // 4) Respond to ROS service caller
  resp->accepted = true;
  resp->status = sent_ok ? "applied (binary config sent)" : "applied (not connected; config not sent)";
}

bool BridgeNode::send_telemetry_config_()
{
  // Build binary config payload and send to robot (TCP)
  struct __attribute__((packed)) Cfg {
    uint8_t enable_imu;
    float   imu_rate_hz;
    uint8_t enable_ir;
    float   ir_rate_hz;
    uint8_t enable_motor;
    float   motor_rate_hz;
    uint8_t enable_batt;
    float   batt_rate_hz;
    uint8_t enable_tof;
    float   tof_rate_hz;
    uint8_t enable_lidar;
    float   lidar_rate_hz;
    uint16_t lidar_udp_port;
  } cfg;

  cfg.enable_imu    = telemetry_.enable_imu ? 1 : 0;
  cfg.imu_rate_hz   = telemetry_.imu_rate_hz;
  cfg.enable_ir     = telemetry_.enable_ir ? 1 : 0;
  cfg.ir_rate_hz    = telemetry_.ir_rate_hz;
  cfg.enable_motor  = telemetry_.enable_motor_state ? 1 : 0;
  cfg.motor_rate_hz = telemetry_.motor_state_rate_hz;
  cfg.enable_batt   = telemetry_.enable_battery ? 1 : 0;
  cfg.batt_rate_hz  = telemetry_.battery_rate_hz;
  cfg.enable_tof    = telemetry_.enable_tof ? 1 : 0;
  cfg.tof_rate_hz   = telemetry_.tof_rate_hz;
  cfg.enable_lidar  = telemetry_.enable_lidar ? 1 : 0;
  cfg.lidar_rate_hz = telemetry_.lidar_rate_hz;
  cfg.lidar_udp_port = telemetry_.lidar_udp_port;

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

  return sent_ok;
}

void BridgeNode::start_or_stop_lidar_udp_()
{
  if (lidar_transport_ != "udp" || lidar_udp_port_ <= 0) {
    lidar_udp_run_.store(false);
    if (lidar_udp_sock_ >= 0) {
      ::shutdown(lidar_udp_sock_, SHUT_RDWR);
      ::close(lidar_udp_sock_);
      lidar_udp_sock_ = -1;
    }
    lidar_udp_port_bound_ = 0;
    if (lidar_udp_thread_.joinable()) {
      lidar_udp_thread_.join();
    }
    return;
  }

  if (lidar_udp_run_.load() && lidar_udp_port_bound_ == lidar_udp_port_) {
    return;
  }
  if (lidar_udp_run_.load()) {
    lidar_udp_run_.store(false);
    if (lidar_udp_sock_ >= 0) {
      ::shutdown(lidar_udp_sock_, SHUT_RDWR);
      ::close(lidar_udp_sock_);
      lidar_udp_sock_ = -1;
    }
    if (lidar_udp_thread_.joinable()) {
      lidar_udp_thread_.join();
    }
  }

  int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    RCLCPP_WARN(this->get_logger(), "failed to create lidar UDP socket");
    return;
  }

  sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(lidar_udp_port_));
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    RCLCPP_WARN(this->get_logger(), "failed to bind lidar UDP port %d", lidar_udp_port_);
    ::close(sock);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "lidar UDP listening on 0.0.0.0:%d", lidar_udp_port_);

  lidar_udp_sock_ = sock;
  lidar_udp_port_bound_ = static_cast<uint16_t>(lidar_udp_port_);
  lidar_udp_run_.store(true);

  lidar_udp_thread_ = std::thread([this]() {
    auto rd16 = [](const uint8_t* p) -> uint16_t {
      return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
    };

    constexpr uint16_t kMagic = 0x4C44; // 'LD'
    constexpr float kTwoPi = 6.28318530718f;

    std::vector<uint8_t> buf;
    buf.resize(2048);
    uint32_t pkt_ok = 0;
    uint32_t pkt_any = 0;
    uint32_t pkt_bad = 0;
    auto last_log = std::chrono::steady_clock::now();

    while (lidar_udp_run_.load()) {
      sockaddr_in src;
      socklen_t slen = sizeof(src);
      ssize_t n = ::recvfrom(lidar_udp_sock_, buf.data(), buf.size(), 0,
                             reinterpret_cast<sockaddr*>(&src), &slen);
      if (n <= 0) {
        continue;
      }
      pkt_any++;
      if (n < 10) {
        pkt_bad++;
        continue;
      }

      uint16_t magic = rd16(buf.data());
      if (magic != kMagic) {
        pkt_bad++;
        continue;
      }
      uint16_t count = rd16(buf.data() + 4);
      uint16_t range_min_mm = rd16(buf.data() + 6);
      uint16_t range_max_mm = rd16(buf.data() + 8);

      size_t expected = 10 + static_cast<size_t>(count) * 2;
      if (static_cast<size_t>(n) < expected || count == 0) {
        pkt_bad++;
        continue;
      }

      sensor_msgs::msg::LaserScan msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = "lidar_link";

      float angle_min = 0.0f;
      float angle_increment = kTwoPi / static_cast<float>(count);
      msg.angle_min = angle_min;
      msg.angle_increment = angle_increment;
      msg.angle_max = angle_min + angle_increment * static_cast<float>(count - 1);

      msg.range_min = static_cast<float>(range_min_mm) / 1000.0f;
      msg.range_max = static_cast<float>(range_max_mm) / 1000.0f;

      msg.ranges.resize(count);
      for (size_t i = 0; i < count; ++i) {
        uint16_t mm = rd16(buf.data() + 10 + i * 2);
        if (mm == 0) {
          msg.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        } else {
          msg.ranges[i] = static_cast<float>(mm) / 1000.0f;
        }
      }

      pub_lidar_->publish(msg);
      pkt_ok++;

      last_log = std::chrono::steady_clock::now();
    }
  });
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

void BridgeNode::send_set_id_()
{
  if (!(tcp_ && tcp_->is_connected())) return;

  const std::string id = robot_id_;
  const uint16_t len = static_cast<uint16_t>(std::min<size_t>(id.size(), 15));

  uint32_t s = seq_.fetch_add(1);
  auto bytes = encode_frame(
    MsgType::SET_ID,
    0,
    s,
    reinterpret_cast<const uint8_t*>(id.data()),
    len
  );

  if (!tcp_->send_bytes(bytes.data(), bytes.size())) {
    stats_.tx_dropped.fetch_add(1);
  }
}

void BridgeNode::start_or_stop_benchmark_timer_()
{
  // stop existing timer
  hb_timer_.reset();

  bool enable = false;
  double hz = 0.0;
  {
    std::lock_guard<std::mutex> lk(bench_mtx_);
    enable = bench_enable_;
    hz = bench_hz_;
  }

  if (transport_mode_ != "tcp_bin" || !enable || hz <= 0.0) {
    std::lock_guard<std::mutex> lk(bench_mtx_);
    pending_.clear();
    rtt_window_.clear();
    stats_.rtt_ms_p50.store(0.0f);
    stats_.rtt_ms_p95.store(0.0f);
    stats_.rtt_ms_p99.store(0.0f);
    return;
  }

  auto period = std::chrono::duration<double>(1.0 / hz);

  hb_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() {
      {
        std::lock_guard<std::mutex> lk(bench_mtx_);
        if (!bench_enable_) return;
      }
      if (!(tcp_ && tcp_->is_connected())) return;

      uint32_t s = seq_.fetch_add(1);
      auto now = std::chrono::steady_clock::now();

      {
        std::lock_guard<std::mutex> lk(bench_mtx_);
        pending_[s] = now;
        // keep pending bounded
        size_t max_pending = 4u * static_cast<size_t>(bench_window_);
        if (pending_.size() > max_pending) pending_.erase(pending_.begin());
      }

      auto bytes = encode_frame(MsgType::HEARTBEAT, 0, s, nullptr, 0);
      if (!tcp_->send_bytes(bytes.data(), bytes.size())) {
        stats_.tx_dropped.fetch_add(1);
      }
    }
  );
}

void BridgeNode::handle_ack_(uint32_t seq)
{
  auto now = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lk(bench_mtx_);
  if (!bench_enable_) return;
  auto it = pending_.find(seq);
  if (it == pending_.end()) return;

  auto t0 = it->second;
  pending_.erase(it);

  float rtt_ms = static_cast<float>(
    std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(now - t0).count());

  rtt_window_.push_back(rtt_ms);
  while (static_cast<int>(rtt_window_.size()) > bench_window_) rtt_window_.pop_front();

  update_rtt_stats_locked_();
}

void BridgeNode::update_rtt_stats_locked_()
{
  if (rtt_window_.empty()) return;

  // Copy + sort for percentiles (window is small)
  std::vector<float> v(rtt_window_.begin(), rtt_window_.end());
  std::sort(v.begin(), v.end());

  auto pct = [&](double p) -> float {
    double idx = p * static_cast<double>(v.size() - 1);
    size_t i0 = static_cast<size_t>(idx);
    size_t i1 = std::min(i0 + 1, v.size() - 1);
    double frac = idx - static_cast<double>(i0);
    return static_cast<float>(v[i0] * (1.0 - frac) + v[i1] * frac);
  };

  stats_.rtt_ms_p50.store(pct(0.50));
  stats_.rtt_ms_p95.store(pct(0.95));
  stats_.rtt_ms_p99.store(pct(0.99));
}

}  // namespace tb_bridge_cpp
