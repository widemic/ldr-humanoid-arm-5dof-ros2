#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("contact_manipulation");
  RCLCPP_INFO(node->get_logger(), "Contact manipulation node (stub implementation)");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}