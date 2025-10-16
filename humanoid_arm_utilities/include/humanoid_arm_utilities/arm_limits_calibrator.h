#ifndef ARM_LIMITS_CALIBRATOR_H
#define ARM_LIMITS_CALIBRATOR_H

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <limits>
#include <memory>

class ArmLimitsCalibrator : public rclcpp::Node
{
public:
    explicit ArmLimitsCalibrator();
    
    // Public control functions
    void startCalibration();
    void stopCalibration();
    bool isCalibrationComplete() const { return calibration_complete_; }

private:
    // Data structures
    struct JointLimit {
        double min;
        double max;
    };
    
    struct WorkspaceLimit {
        double min_x, max_x;
        double min_y, max_y;
        double min_z, max_z;
    };

    // ROS2 callbacks
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    // Calibration sequence control
    void calibrationSequence();
    void executeCurrentStep();
    void nextStep();
    void finishCalibration();
    
    // Joint testing functions
    void testBaseRotation();
    void testShoulderPitch();
    void testElbowPitch();
    void testWristPitch();
    void testWristRoll();
    void testWorkspaceCombinations();
    
    // Utility functions
    void initializeTestSequence();
    void sendJointCommand(const std::vector<double>& joint_angles);
    bool checkMovementCompletion(const std::vector<double>& target_angles);
    void showCurrentState();
    void updateWorkspaceLimits();
    void printCalibrationResults();
    void saveCalibrationResults();
    void saveLimitsHeader();
    
    // Kinematics functions
    geometry_msgs::msg::Point forwardKinematics(const std::vector<double>& joint_angles);
    
    // Conversion helpers
    double degToRad(double degrees) { return degrees * M_PI / 180.0; }
    double radToDeg(double radians) { return radians * 180.0 / M_PI; }
    
    // ROS2 members
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr calibration_timer_;
    
    // Calibration state
    std::vector<std::string> joint_names_;
    std::vector<double> current_joint_positions_;
    std::vector<double> current_target_angles_;
    std::vector<JointLimit> joint_limits_;
    WorkspaceLimit workspace_limits_;
    
    // Arm dimensions
    double L1_, L2_;
    double shoulder_x_, shoulder_y_, shoulder_z_;
    
    // Calibration control
    int current_step_;
    int current_sub_step_;
    bool joint_states_received_;
    bool calibration_complete_;
    bool waiting_for_movement_;
    rclcpp::Time movement_start_time_;
};

#endif // ARM_LIMITS_CALIBRATOR_H