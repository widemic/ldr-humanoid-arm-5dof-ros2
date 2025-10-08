#ifndef HUMANOID_ARM_TELEOP__JOYSTICK_TELEOP_HPP_
#define HUMANOID_ARM_TELEOP__JOYSTICK_TELEOP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace humanoid_arm_teleop
{

class JoystickTeleop : public rclcpp::Node
{
public:
  JoystickTeleop();

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publishJointCommands();

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Publishers
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  double publish_rate_;
  std::vector<double> joint_velocities_;
  std::vector<double> current_positions_;
  std::vector<double> target_positions_;
  bool deadman_pressed_;
  bool joint_state_received_;

  // Joint names for proper ordering
  std::vector<std::string> joint_names_;
};

}  // namespace humanoid_arm_teleop

#endif  // HUMANOID_ARM_TELEOP__JOYSTICK_TELEOP_HPP_