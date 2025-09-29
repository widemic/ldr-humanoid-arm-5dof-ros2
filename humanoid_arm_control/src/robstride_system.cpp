#include "humanoid_arm_control/robstride_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace humanoid_arm_control
{
hardware_interface::CallbackReturn RobstrideSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters
  use_fake_hardware_ = info_.hardware_parameters.at("use_fake_hardware") == "true";
  slowdown_ = std::stod(info_.hardware_parameters.at("slowdown"));

  if (!use_fake_hardware_)
  {
    can_interface_ = info_.hardware_parameters.at("can_interface");
    // Parse motor IDs and types from parameters
    // TODO: Implement parameter parsing for real hardware
  }

  // Initialize state and command vectors
  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Initialize sensor states
  forearm_force_states_.resize(6, 0.0);  // Fx, Fy, Fz, Tx, Ty, Tz
  forearm_torque_states_.resize(6, 0.0);
  wrist_force_states_.resize(6, 0.0);
  wrist_torque_states_.resize(6, 0.0);
  tcp_force_states_.resize(6, 0.0);
  tcp_torque_states_.resize(6, 0.0);

  // Initialize IMU states
  imu_orientation_.resize(4, 0.0);       // Quaternion (x, y, z, w)
  imu_angular_velocity_.resize(3, 0.0);  // rad/s
  imu_linear_acceleration_.resize(3, 0.0); // m/s²

  // Set initial IMU orientation (identity quaternion)
  imu_orientation_[3] = 1.0; // w = 1

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RobstrideSystemHardware has exactly 3 state interfaces
    // and 3 command interfaces on each joint
    if (joint.command_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobstrideSystemHardware"),
        "Joint '%s' has %zu command interfaces. 3 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobstrideSystemHardware"),
        "Joint '%s' has %zu state interfaces. 3 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Configure hardware interface
  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystemHardware"), "Configuring...");

  // Initialize hardware state
  for (std::size_t i = 0; i < hw_states_position_.size(); i++)
  {
    hw_states_position_[i] = 0.0;
    hw_states_velocity_[i] = 0.0;
    hw_states_effort_[i] = 0.0;
    hw_commands_position_[i] = 0.0;
    hw_commands_velocity_[i] = 0.0;
    hw_commands_effort_[i] = 0.0;
  }

  if (!use_fake_hardware_)
  {
    // Initialize CAN interface
    if (!initializeCAN())
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobstrideSystemHardware"), "Failed to initialize CAN interface");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!configureCAN())
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobstrideSystemHardware"), "Failed to configure CAN interface");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystemHardware"), "Configuration successful");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobstrideSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Joint state interfaces
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));
  }

  // Force-torque sensor state interfaces
  std::vector<std::string> sensor_names = {"forearm_force_sensor", "wrist_force_sensor", "tcp_force_sensor"};
  std::vector<std::vector<double>*> force_states = {&forearm_force_states_, &wrist_force_states_, &tcp_force_states_};

  for (std::size_t sensor_idx = 0; sensor_idx < sensor_names.size(); sensor_idx++)
  {
    std::vector<std::string> force_torque_interfaces = {
      "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"
    };

    for (std::size_t i = 0; i < force_torque_interfaces.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor_names[sensor_idx], force_torque_interfaces[i], &((*force_states[sensor_idx])[i])));
    }
  }

  // IMU sensor state interfaces
  std::vector<std::string> imu_interfaces = {
    "orientation.x", "orientation.y", "orientation.z", "orientation.w",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"
  };

  for (std::size_t i = 0; i < 4; i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "arm_imu", imu_interfaces[i], &imu_orientation_[i]));
  }

  for (std::size_t i = 0; i < 3; i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "arm_imu", imu_interfaces[i + 4], &imu_angular_velocity_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "arm_imu", imu_interfaces[i + 7], &imu_linear_acceleration_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobstrideSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RobstrideSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystemHardware"), "Activating...");

  // Set initial command values
  for (std::size_t i = 0; i < hw_states_position_.size(); i++)
  {
    if (std::isnan(hw_commands_position_[i]))
    {
      hw_commands_position_[i] = hw_states_position_[i];
    }
    if (std::isnan(hw_commands_velocity_[i]))
    {
      hw_commands_velocity_[i] = 0.0;
    }
    if (std::isnan(hw_commands_effort_[i]))
    {
      hw_commands_effort_[i] = 0.0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystemHardware"), "Activation successful");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystemHardware"), "Deactivating...");

  if (!use_fake_hardware_)
  {
    shutdownCAN();
  }

  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystemHardware"), "Deactivation successful");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobstrideSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (use_fake_hardware_)
  {
    // Simulate motor dynamics
    simulateMotorDynamics(period);
    simulateContactForces();
    simulateIMU();
  }
  else
  {
    // Read from real hardware
    for (std::size_t i = 0; i < hw_states_position_.size(); i++)
    {
      double position, velocity, effort;
      if (readMotorState(static_cast<int>(i + 1), position, velocity, effort))
      {
        hw_states_position_[i] = position;
        hw_states_velocity_[i] = velocity;
        hw_states_effort_[i] = effort;
      }
    }

    // TODO: Read sensor data from real hardware
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobstrideSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (use_fake_hardware_)
  {
    // For fake hardware, commands are applied in the read() method during simulation
    return hardware_interface::return_type::OK;
  }

  // Send commands to real hardware
  for (std::size_t i = 0; i < hw_commands_position_.size(); i++)
  {
    if (!std::isnan(hw_commands_position_[i]))
    {
      sendPositionCommand(static_cast<int>(i + 1), hw_commands_position_[i]);
    }

    if (!std::isnan(hw_commands_velocity_[i]))
    {
      sendVelocityCommand(static_cast<int>(i + 1), hw_commands_velocity_[i]);
    }
  }

  return hardware_interface::return_type::OK;
}

// Private helper methods for hardware simulation

void RobstrideSystemHardware::simulateMotorDynamics(const rclcpp::Duration& period)
{
  // Simple simulation: move towards commanded position with velocity limits
  double dt = period.seconds();

  for (std::size_t i = 0; i < hw_states_position_.size(); i++)
  {
    if (!std::isnan(hw_commands_position_[i]))
    {
      // Position control mode
      double position_error = hw_commands_position_[i] - hw_states_position_[i];
      double max_velocity = 2.0; // rad/s

      if (i >= 2) max_velocity = 1.5; // Slower for wrist joints
      if (i == 4) max_velocity = 0.8; // Slowest for wrist roll

      max_velocity /= slowdown_;

      double velocity_command = std::copysign(std::min(std::abs(position_error) / dt, max_velocity), position_error);
      hw_states_velocity_[i] = velocity_command;
      hw_states_position_[i] += velocity_command * dt;

      // Simulate some effort based on velocity
      hw_states_effort_[i] = velocity_command * 0.1; // Simple model
    }
    else if (!std::isnan(hw_commands_velocity_[i]))
    {
      // Velocity control mode
      hw_states_velocity_[i] = hw_commands_velocity_[i] / slowdown_;
      hw_states_position_[i] += hw_states_velocity_[i] * dt;
      hw_states_effort_[i] = hw_states_velocity_[i] * 0.1;
    }
  }
}

void RobstrideSystemHardware::simulateContactForces()
{
  // Simple simulation of contact forces
  // In real implementation, these would come from Gazebo or real sensors

  // Add some noise to simulate sensor readings
  static double time_counter = 0.0;
  time_counter += 0.01; // Assume 100Hz update rate

  for (std::size_t i = 0; i < 6; i++)
  {
    // Simulate small random forces/torques
    forearm_force_states_[i] = 0.1 * sin(time_counter + i) + 0.05 * sin(3 * time_counter + i);
    wrist_force_states_[i] = 0.08 * sin(1.5 * time_counter + i) + 0.03 * cos(2 * time_counter + i);
    tcp_force_states_[i] = 0.12 * cos(time_counter + i) + 0.04 * sin(4 * time_counter + i);
  }
}

void RobstrideSystemHardware::simulateIMU()
{
  // Simple IMU simulation
  static double time_counter = 0.0;
  time_counter += 0.01;

  // Simulate small orientation changes
  double roll = 0.05 * sin(0.5 * time_counter);
  double pitch = 0.03 * cos(0.7 * time_counter);
  double yaw = 0.02 * sin(0.3 * time_counter);

  // Convert to quaternion (simplified)
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);

  imu_orientation_[3] = cr * cp * cy + sr * sp * sy; // w
  imu_orientation_[0] = sr * cp * cy - cr * sp * sy; // x
  imu_orientation_[1] = cr * sp * cy + sr * cp * sy; // y
  imu_orientation_[2] = cr * cp * sy - sr * sp * cy; // z

  // Simulate angular velocities
  imu_angular_velocity_[0] = 0.1 * sin(2 * time_counter);
  imu_angular_velocity_[1] = 0.08 * cos(1.5 * time_counter);
  imu_angular_velocity_[2] = 0.05 * sin(3 * time_counter);

  // Simulate linear accelerations (with gravity)
  imu_linear_acceleration_[0] = 0.5 * sin(time_counter);
  imu_linear_acceleration_[1] = 0.3 * cos(1.2 * time_counter);
  imu_linear_acceleration_[2] = -9.81 + 0.2 * sin(2.5 * time_counter); // Gravity + noise
}

// CAN interface methods (stubs for real hardware)

bool RobstrideSystemHardware::initializeCAN()
{
  // TODO: Implement CAN interface initialization
  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystemHardware"), "CAN interface initialization (stub)");
  return true;
}

bool RobstrideSystemHardware::configureCAN()
{
  // TODO: Implement CAN configuration
  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystemHardware"), "CAN configuration (stub)");
  return true;
}

bool RobstrideSystemHardware::sendPositionCommand(int motor_id, double position)
{
  // TODO: Implement position command via CAN
  RCLCPP_DEBUG(rclcpp::get_logger("RobstrideSystemHardware"),
    "Sending position command to motor %d: %f rad", motor_id, position);
  return true;
}

bool RobstrideSystemHardware::sendVelocityCommand(int motor_id, double velocity)
{
  // TODO: Implement velocity command via CAN
  RCLCPP_DEBUG(rclcpp::get_logger("RobstrideSystemHardware"),
    "Sending velocity command to motor %d: %f rad/s", motor_id, velocity);
  return true;
}

bool RobstrideSystemHardware::readMotorState(int motor_id, double& position, double& velocity, double& effort)
{
  // TODO: Implement motor state reading via CAN
  RCLCPP_DEBUG(rclcpp::get_logger("RobstrideSystemHardware"),
    "Reading state from motor %d", motor_id);

  // Placeholder values
  position = 0.0;
  velocity = 0.0;
  effort = 0.0;

  return true;
}

void RobstrideSystemHardware::shutdownCAN()
{
  // TODO: Implement CAN shutdown
  RCLCPP_INFO(rclcpp::get_logger("RobstrideSystemHardware"), "CAN interface shutdown (stub)");
}

}  // namespace humanoid_arm_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  humanoid_arm_control::RobstrideSystemHardware, hardware_interface::SystemInterface)