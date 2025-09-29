#include "humanoid_arm_teleop/joystick_teleop.hpp"

namespace humanoid_arm_teleop
{

JoystickTeleop::JoystickTeleop() : Node("joystick_teleop")
{
  // Parameters
  publish_rate_ = this->declare_parameter("publish_rate", 50.0);

  // Initialize joint commands (5 joints)
  joint_commands_.resize(5, 0.0);
  deadman_pressed_ = false;

  // Subscribers
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&JoystickTeleop::joyCallback, this, std::placeholders::_1));

  // Publishers
  joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory_controller/joint_trajectory", 10);

  joint_position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/arm_position_controller/commands", 10);

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
    std::bind(&JoystickTeleop::publishJointCommands, this));

  RCLCPP_INFO(this->get_logger(), "Joystick teleop node started");
}

void JoystickTeleop::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->buttons.size() < 5 || msg->axes.size() < 4)
  {
    RCLCPP_WARN(this->get_logger(), "Insufficient joystick buttons/axes");
    return;
  }

  // Deadman switch (button 4 - L1)
  deadman_pressed_ = msg->buttons[4];

  if (!deadman_pressed_)
  {
    // Zero all commands if deadman not pressed
    std::fill(joint_commands_.begin(), joint_commands_.end(), 0.0);
    return;
  }

  // Emergency stop (button 5 - R1)
  if (msg->buttons[5])
  {
    std::fill(joint_commands_.begin(), joint_commands_.end(), 0.0);
    RCLCPP_WARN(this->get_logger(), "Emergency stop activated!");
    return;
  }

  // Individual joint control mode
  // Map joystick axes to joint velocities
  double velocity_scale = 1.0;

  // Left stick - base rotation and shoulder pitch
  joint_commands_[0] = msg->axes[0] * velocity_scale;  // Base rotation
  joint_commands_[1] = msg->axes[1] * velocity_scale;  // Shoulder pitch

  // Right stick - elbow and wrist pitch
  joint_commands_[2] = msg->axes[2] * velocity_scale * 0.8;  // Elbow pitch
  joint_commands_[3] = msg->axes[3] * velocity_scale * 0.8;  // Wrist pitch

  // Triggers for wrist roll
  double wrist_roll_cmd = 0.0;
  if (msg->buttons.size() > 7)
  {
    if (msg->buttons[6]) wrist_roll_cmd += 0.6;  // R2
    if (msg->buttons[7]) wrist_roll_cmd -= 0.6;  // L2
  }
  joint_commands_[4] = wrist_roll_cmd;  // Wrist roll
}

void JoystickTeleop::publishJointCommands()
{
  if (!deadman_pressed_)
  {
    return;
  }

  // Publish as joint position commands
  auto position_msg = std_msgs::msg::Float64MultiArray();
  position_msg.data = joint_commands_;
  joint_position_pub_->publish(position_msg);

  // Also publish as joint trajectory (for trajectory controller)
  auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
  trajectory_msg.header.stamp = this->now();
  trajectory_msg.joint_names = {
    "base_rotation_joint",
    "shoulder_pitch_joint",
    "elbow_pitch_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint"
  };

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = joint_commands_;
  point.time_from_start = rclcpp::Duration::from_nanoseconds(100000000); // 0.1 seconds

  trajectory_msg.points.push_back(point);
  joint_trajectory_pub_->publish(trajectory_msg);
}

}  // namespace humanoid_arm_teleop