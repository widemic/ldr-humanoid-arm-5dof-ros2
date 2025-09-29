#ifndef HUMANOID_ARM_CONTROL__ROBSTRIDE_SYSTEM_HPP_
#define HUMANOID_ARM_CONTROL__ROBSTRIDE_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"

#include "humanoid_arm_control/visibility_control.h"

namespace humanoid_arm_control
{
class RobstrideSystemHardware : public hardware_interface::SystemInterface
{
public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters
  bool use_fake_hardware_;
  double slowdown_;
  std::string can_interface_;
  std::vector<int> motor_ids_;
  std::vector<std::string> motor_types_;

  // Robstride motor specifications
  struct MotorSpecs {
    int motor_id;
    std::string motor_type;
    double gear_ratio;
    double max_torque;
    double max_speed;
    int encoder_resolution;
    int can_id;
  };
  std::vector<MotorSpecs> motor_specs_;

  // Store the command and state values
  std::vector<double> hw_commands_position_;
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_commands_effort_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_effort_;

  // Contact force sensor states
  std::vector<double> forearm_force_states_;
  std::vector<double> forearm_torque_states_;
  std::vector<double> wrist_force_states_;
  std::vector<double> wrist_torque_states_;
  std::vector<double> tcp_force_states_;
  std::vector<double> tcp_torque_states_;

  // IMU states
  std::vector<double> imu_orientation_;
  std::vector<double> imu_angular_velocity_;
  std::vector<double> imu_linear_acceleration_;

  // CAN interface members (for real hardware)
  // TODO: Add CAN interface implementation
  bool initializeCAN();
  bool configureCAN();
  bool sendPositionCommand(int motor_id, double position);
  bool sendVelocityCommand(int motor_id, double velocity);
  bool readMotorState(int motor_id, double& position, double& velocity, double& effort);
  void shutdownCAN();

  // Fake hardware simulation
  void simulateMotorDynamics(const rclcpp::Duration& period);
  void simulateContactForces();
  void simulateIMU();
};

}  // namespace humanoid_arm_control

#endif  // HUMANOID_ARM_CONTROL__ROBSTRIDE_SYSTEM_HPP_