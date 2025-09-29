#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("keyboard_teleop");
  RCLCPP_INFO(node->get_logger(), "Keyboard teleop node (stub implementation)");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}