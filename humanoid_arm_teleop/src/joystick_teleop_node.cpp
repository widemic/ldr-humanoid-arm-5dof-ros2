#include <rclcpp/rclcpp.hpp>
#include "humanoid_arm_teleop/joystick_teleop.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<humanoid_arm_teleop::JoystickTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}