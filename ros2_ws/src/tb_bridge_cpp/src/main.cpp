#include "rclcpp/rclcpp.hpp"
#include "tb_bridge_cpp/bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tb_bridge_cpp::BridgeNode>());
  rclcpp::shutdown();
  return 0;
}

