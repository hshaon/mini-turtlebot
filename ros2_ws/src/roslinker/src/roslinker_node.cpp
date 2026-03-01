#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "tb_interfaces/msg/ir_state.hpp"
#include "tb_interfaces/srv/get_capabilities.hpp"
#include "tb_interfaces/srv/set_telemetry.hpp"

#include "atrbridge_core/core.hpp"

class RosLinkerNode : public rclcpp::Node
{
public:
  RosLinkerNode()
  : Node("roslinker")
  {
    robot_id_ = declare_parameter<std::string>("robot_id", "tb_01");
    robot_ip_ = declare_parameter<std::string>("robot_ip", "192.168.0.78");
    tcp_port_ = declare_parameter<int>("port", 9000);
    lidar_udp_port_ = declare_parameter<int>("lidar_udp_port", 5601);

    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist & msg) {
        core_.send_cmd_vel(static_cast<float>(msg.linear.x), static_cast<float>(msg.angular.z));
      });

    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::QoS(20));
    pub_ir_ = create_publisher<tb_interfaces::msg::IRState>("ir/state", rclcpp::QoS(10));
    pub_tof_ = create_publisher<sensor_msgs::msg::Range>("tof/range", rclcpp::QoS(10));
    pub_lidar_ = create_publisher<sensor_msgs::msg::LaserScan>("lidar/scan", rclcpp::QoS(10));

    srv_set_telemetry_ = create_service<tb_interfaces::srv::SetTelemetry>(
      "bridge/set_telemetry",
      [this](
        const std::shared_ptr<tb_interfaces::srv::SetTelemetry::Request> req,
        std::shared_ptr<tb_interfaces::srv::SetTelemetry::Response> resp) {
        atrbridge_core::TelemetryConfig cfg;
        cfg.enable_imu = req->enable_imu;
        cfg.imu_rate_hz = static_cast<float>(req->imu_rate_hz);
        cfg.enable_ir = req->enable_ir;
        cfg.ir_rate_hz = static_cast<float>(req->ir_rate_hz);
        cfg.enable_motor_state = req->enable_motor_state;
        cfg.motor_state_rate_hz = static_cast<float>(req->motor_state_rate_hz);
        cfg.enable_battery = req->enable_battery;
        cfg.battery_rate_hz = static_cast<float>(req->battery_rate_hz);
        cfg.enable_tof = req->enable_tof;
        cfg.tof_rate_hz = static_cast<float>(req->tof_rate_hz);
        cfg.enable_lidar = req->enable_lidar;
        cfg.lidar_rate_hz = static_cast<float>(req->lidar_rate_hz);
        cfg.lidar_udp_port = req->lidar_udp_port;
        const bool ok = core_.send_telemetry_config(cfg);
        resp->accepted = true;
        resp->status = ok ? "applied" : "not_connected";
      });

    srv_get_caps_ = create_service<tb_interfaces::srv::GetCapabilities>(
      "bridge/get_capabilities",
      [this](
        const std::shared_ptr<tb_interfaces::srv::GetCapabilities::Request>,
        std::shared_ptr<tb_interfaces::srv::GetCapabilities::Response> resp) {
        resp->success = true;
        resp->capabilities_json = core_.get_capabilities_json();
        resp->message = "ok";
      });

    core_.set_imu_callback([this](const atrbridge_core::ImuSample & s) {
      sensor_msgs::msg::Imu msg;
      msg.header.stamp = now();
      msg.header.frame_id = "imu_link";
      msg.linear_acceleration.x = s.ax;
      msg.linear_acceleration.y = s.ay;
      msg.linear_acceleration.z = s.az;
      msg.angular_velocity.x = s.gx;
      msg.angular_velocity.y = s.gy;
      msg.angular_velocity.z = s.gz;
      msg.orientation_covariance[0] = -1.0;
      pub_imu_->publish(msg);
    });

    core_.set_ir_callback([this](const atrbridge_core::IrSample & s) {
      tb_interfaces::msg::IRState msg;
      msg.header.stamp = now();
      msg.header.frame_id = "ir_link";
      msg.left = s.left;
      msg.right = s.right;
      pub_ir_->publish(msg);
    });

    core_.set_tof_callback([this](const atrbridge_core::TofSample & s) {
      sensor_msgs::msg::Range msg;
      msg.header.stamp = now();
      msg.header.frame_id = "tof_link";
      msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
      msg.field_of_view = 0.44f;
      msg.min_range = 0.03f;
      msg.max_range = 2.0f;
      msg.range = s.range_m;
      pub_tof_->publish(msg);
    });

    core_.set_lidar_callback([this](const atrbridge_core::LidarSample & s) {
      if (s.count == 0 || s.ranges_mm.empty()) {
        return;
      }
      sensor_msgs::msg::LaserScan msg;
      msg.header.stamp = now();
      msg.header.frame_id = "laser_frame";
      constexpr float kPi = 3.14159265358979323846f;
      msg.angle_min = -kPi;
      msg.angle_max = kPi;
      msg.angle_increment = (msg.angle_max - msg.angle_min) / static_cast<float>(s.count);
      msg.time_increment = 0.0f;
      msg.scan_time = 0.0f;
      msg.range_min = static_cast<float>(s.range_min_mm) / 1000.0f;
      msg.range_max = static_cast<float>(s.range_max_mm) / 1000.0f;
      msg.ranges.resize(s.count);
      for (size_t i = 0; i < s.count; ++i) {
        msg.ranges[i] = static_cast<float>(s.ranges_mm[i]) / 1000.0f;
      }
      pub_lidar_->publish(msg);
    });

    if (!core_.start(robot_ip_, tcp_port_, lidar_udp_port_)) {
      RCLCPP_FATAL(get_logger(), "Failed to start atrbridge_core");
      throw std::runtime_error("atrbridge_core start failed");
    }

    // Retry SET_ID until TCP is connected, and resend after reconnect.
    set_id_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        const auto caps = core_.get_capabilities();
        if (!caps.runtime.tcp_connected) {
          set_id_sent_ = false;
          return;
        }
        if (set_id_sent_) {
          return;
        }
        if (core_.send_set_id(robot_id_)) {
          set_id_sent_ = true;
          RCLCPP_INFO(get_logger(), "SET_ID sent: %s", robot_id_.c_str());
        }
      });

    RCLCPP_INFO(
      get_logger(), "roslinker started robot_id=%s ip=%s port=%d lidar_udp=%d",
      robot_id_.c_str(), robot_ip_.c_str(), tcp_port_, lidar_udp_port_);
  }

  ~RosLinkerNode() override
  {
    core_.stop();
  }

private:
  std::string robot_id_;
  std::string robot_ip_;
  int tcp_port_{9000};
  int lidar_udp_port_{5601};

  atrbridge_core::AtrBridgeCore core_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<tb_interfaces::msg::IRState>::SharedPtr pub_ir_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_tof_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_lidar_;
  rclcpp::Service<tb_interfaces::srv::SetTelemetry>::SharedPtr srv_set_telemetry_;
  rclcpp::Service<tb_interfaces::srv::GetCapabilities>::SharedPtr srv_get_caps_;
  rclcpp::TimerBase::SharedPtr set_id_timer_;
  bool set_id_sent_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RosLinkerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
