#include "humanoid_arm_teleop/joystick_teleop.hpp"

namespace humanoid_arm_teleop
{

JoystickTeleop::JoystickTeleop() : Node("joystick_teleop")
{
  // Parameters
  publish_rate_ = this->declare_parameter("publish_rate", 50.0);

  // Initialize vectors (5 joints)
  joint_names_ = {
    "base_rotation_joint",
    "shoulder_pitch_joint",
    "elbow_pitch_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint"
  };

  joint_velocities_.resize(5, 0.0);
  current_positions_.resize(5, 0.0);
  target_positions_.resize(5, 0.0);
  deadman_pressed_ = false;
  joint_state_received_ = false;

  // Subscribers
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&JoystickTeleop::joyCallback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&JoystickTeleop::jointStateCallback, this, std::placeholders::_1));

  // Publishers
  joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory_controller/joint_trajectory", 10);

  joint_position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/arm_position_controller/commands", 10);

  // Timer for command publishing
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
    std::bind(&JoystickTeleop::publishJointCommands, this));

  RCLCPP_INFO(this->get_logger(), "Joystick teleop node started");
  RCLCPP_INFO(this->get_logger(), "Hold L1 (button 4) to enable control");
  RCLCPP_INFO(this->get_logger(), "Press R1 (button 5) for emergency stop");
}

void JoystickTeleop::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Update current joint positions from robot state
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
    if (it != msg->name.end())
    {
      size_t index = std::distance(msg->name.begin(), it);
      current_positions_[i] = msg->position[index];
    }
  }

  // Initialize target positions on first message
  if (!joint_state_received_)
  {
    target_positions_ = current_positions_;
    joint_state_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Received initial joint states");
  }
}

void JoystickTeleop::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->buttons.size() < 5 || msg->axes.size() < 4)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Insufficient joystick buttons/axes");
    return;
  }

  // Deadman switch (button 4 - L1)
  bool previous_deadman = deadman_pressed_;
  deadman_pressed_ = msg->buttons[4];

  if (deadman_pressed_ && !previous_deadman)
  {
    RCLCPP_INFO(this->get_logger(), "Deadman switch activated - control enabled");
    // Reset target to current position when deadman is pressed
    target_positions_ = current_positions_;
  }
  else if (!deadman_pressed_ && previous_deadman)
  {
    RCLCPP_INFO(this->get_logger(), "Deadman switch released - control disabled");
  }

  if (!deadman_pressed_)
  {
    // Zero all velocities if deadman not pressed
    std::fill(joint_velocities_.begin(), joint_velocities_.end(), 0.0);
    return;
  }

  // Emergency stop (button 5 - R1)
  if (msg->buttons[5])
  {
    std::fill(joint_velocities_.begin(), joint_velocities_.end(), 0.0);
    target_positions_ = current_positions_;
    RCLCPP_WARN(this->get_logger(), "Emergency stop activated!");
    return;
  }

  // Individual joint velocity control
  // Velocity scaling factors (rad/s per full joystick deflection)
  double base_velocity_scale = 1.0;
  double shoulder_velocity_scale = 1.0;
  double elbow_velocity_scale = 0.8;
  double wrist_velocity_scale = 0.8;
  double roll_velocity_scale = 0.6;

  // Apply deadzone
  double deadzone = 0.1;
  auto apply_deadzone = [deadzone](double value) {
    return (std::abs(value) < deadzone) ? 0.0 : value;
  };

  // Left stick - base rotation and shoulder pitch
  joint_velocities_[0] = apply_deadzone(msg->axes[0]) * base_velocity_scale;
  joint_velocities_[1] = -apply_deadzone(msg->axes[1]) * shoulder_velocity_scale;  // Inverted Y

  // Right stick - elbow and wrist pitch
  joint_velocities_[2] = apply_deadzone(msg->axes[2]) * elbow_velocity_scale;
  joint_velocities_[3] = -apply_deadzone(msg->axes[3]) * wrist_velocity_scale;  // Inverted Y

  // Triggers for wrist roll (R2/L2 are buttons on DualSense)
  double wrist_roll_vel = 0.0;
  if (msg->buttons.size() > 7)
  {
    if (msg->buttons[6]) wrist_roll_vel += roll_velocity_scale;  // R2
    if (msg->buttons[7]) wrist_roll_vel -= roll_velocity_scale;  // L2
  }
  joint_velocities_[4] = wrist_roll_vel;
}

void JoystickTeleop::publishJointCommands()
{
  if (!joint_state_received_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Waiting for joint states...");
    return;
  }

  if (!deadman_pressed_)
  {
    return;
  }

  // Integrate velocities to update target positions
  double dt = 1.0 / publish_rate_;
  for (size_t i = 0; i < 5; ++i)
  {
    target_positions_[i] += joint_velocities_[i] * dt;
  }

  // Apply joint limits (from URDF)
  // base_rotation_joint: -3.14 to 3.14
  target_positions_[0] = std::clamp(target_positions_[0], -3.14, 3.14);
  // shoulder_pitch_joint: -0.55 to 3.1
  target_positions_[1] = std::clamp(target_positions_[1], -0.55, 3.1);
  // elbow_pitch_joint: -3.14 to 3.14
  target_positions_[2] = std::clamp(target_positions_[2], -3.14, 3.14);
  // wrist_pitch_joint: -0.31 to 2.8
  target_positions_[3] = std::clamp(target_positions_[3], -0.31, 2.8);
  // wrist_roll_joint: -3.14 to 3.14
  target_positions_[4] = std::clamp(target_positions_[4], -3.14, 3.14);

  // Publish as joint trajectory (for trajectory controller)
  auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
  trajectory_msg.header.stamp = this->now();
  trajectory_msg.joint_names = joint_names_;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = target_positions_;
  point.velocities = joint_velocities_;
  point.time_from_start = rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(dt * 1e9));

  trajectory_msg.points.push_back(point);
  joint_trajectory_pub_->publish(trajectory_msg);

  // Also publish to position controller (if active)
  auto position_msg = std_msgs::msg::Float64MultiArray();
  position_msg.data = target_positions_;
  joint_position_pub_->publish(position_msg);
}

}  // namespace humanoid_arm_teleop
