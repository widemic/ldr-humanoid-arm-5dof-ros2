#include "../include/humanoid_arm_utilities/arm_limits_calibrator.h"

ArmLimitsCalibrator::ArmLimitsCalibrator() 
    : Node("arm_limits_calibrator")
{
    // Publisher for joint commands
    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 10);
    
    // Subscriber for joint states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&ArmLimitsCalibrator::jointStateCallback, this, std::placeholders::_1));
    
    // Joint names
    joint_names_ = {
        "base_rotation_joint",
        "shoulder_pitch_joint", 
        "elbow_pitch_joint",
        "wrist_pitch_joint",
        "wrist_roll_joint"
    };
    
    // Initialize limits
    current_joint_positions_.resize(joint_names_.size(), 0.0);
    joint_limits_.resize(joint_names_.size());
    for (auto& limit : joint_limits_) {
        limit.min = std::numeric_limits<double>::max();
        limit.max = std::numeric_limits<double>::lowest();
    }
    
    workspace_limits_.min_x = std::numeric_limits<double>::max();
    workspace_limits_.max_x = std::numeric_limits<double>::lowest();
    workspace_limits_.min_y = std::numeric_limits<double>::max();
    workspace_limits_.max_y = std::numeric_limits<double>::lowest();
    workspace_limits_.min_z = std::numeric_limits<double>::max();
    workspace_limits_.max_z = std::numeric_limits<double>::lowest();
    
    joint_states_received_ = false;
    calibration_complete_ = false;
    current_step_ = -1; // -1 for initialization
    current_sub_step_ = 0;
    movement_start_time_ = this->now();
    waiting_for_movement_ = false;
    
    // Arm dimensions
    L1_ = 0.299;
    L2_ = 0.430;
    shoulder_x_ = -0.3827;
    shoulder_y_ = 0.0;
    shoulder_z_ = 1.0987;
    
    std::cout << "ARM LIMITS CALIBRATOR - Determining real joint limits" << std::endl;
    std::cout << "====================================================" << std::endl;
    
    // Timer for calibration sequence
    calibration_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ArmLimitsCalibrator::calibrationSequence, this));
}

void ArmLimitsCalibrator::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (size_t j = 0; j < joint_names_.size(); ++j) {
            if (msg->name[i] == joint_names_[j]) {
                current_joint_positions_[j] = msg->position[i];
                
                // Update joint limits
                if (msg->position[i] < joint_limits_[j].min) {
                    joint_limits_[j].min = msg->position[i];
                }
                if (msg->position[i] > joint_limits_[j].max) {
                    joint_limits_[j].max = msg->position[i];
                }
                
                break;
            }
        }
    }
    joint_states_received_ = true;
    
    // Update workspace limits
    updateWorkspaceLimits();
}

void ArmLimitsCalibrator::updateWorkspaceLimits()
{
    geometry_msgs::msg::Point current_pos = forwardKinematics(current_joint_positions_);
    
    workspace_limits_.min_x = std::min(workspace_limits_.min_x, current_pos.x);
    workspace_limits_.max_x = std::max(workspace_limits_.max_x, current_pos.x);
    workspace_limits_.min_y = std::min(workspace_limits_.min_y, current_pos.y);
    workspace_limits_.max_y = std::max(workspace_limits_.max_y, current_pos.y);
    workspace_limits_.min_z = std::min(workspace_limits_.min_z, current_pos.z);
    workspace_limits_.max_z = std::max(workspace_limits_.max_z, current_pos.z);
}

geometry_msgs::msg::Point ArmLimitsCalibrator::forwardKinematics(const std::vector<double>& joint_angles)
{
    geometry_msgs::msg::Point position;
    
    double base = joint_angles[0];
    double shoulder = joint_angles[1];
    double elbow = joint_angles[2];
    
    double x_rel = L1_ * cos(shoulder) + L2_ * cos(shoulder + elbow);
    double z_rel = L1_ * sin(shoulder) + L2_ * sin(shoulder + elbow);
    
    position.x = shoulder_x_ + x_rel * cos(base);
    position.y = shoulder_y_ + x_rel * sin(base);
    position.z = shoulder_z_ + z_rel;
    
    return position;
}

void ArmLimitsCalibrator::calibrationSequence()
{
    if (!joint_states_received_) {
        static int wait_count = 0;
        if (wait_count++ % 4 == 0) {
            std::cout << "Waiting for joint states from Gazebo..." << std::endl;
        }
        return;
    }
    
    static bool first_run = true;
    if (first_run) {
        first_run = false;
        std::cout << "Connected to Gazebo! Starting calibration..." << std::endl;
        showCurrentState();
        current_step_ = -1;
        waiting_for_movement_ = false;
    }
    
    if (calibration_complete_) {
        return;
    }
    
    // If waiting for movement, check if it's complete
    if (waiting_for_movement_) {
        auto current_time = this->now();
        auto elapsed = (current_time - movement_start_time_).seconds();
        bool movement_complete = checkMovementCompletion(current_target_angles_);
        
        if (movement_complete || elapsed > 15.0) { // 15 second timeout
            if (movement_complete) {
                std::cout << "Movement " << current_step_ << "." << current_sub_step_ 
                          << " completed in " << elapsed << " seconds" << std::endl;
            } else {
                std::cout << "Movement " << current_step_ << "." << current_sub_step_ 
                          << " timeout after " << elapsed << " seconds - continuing..." << std::endl;
            }
            
            showCurrentState();
            waiting_for_movement_ = false;
            
            // Short pause between movements
            rclcpp::sleep_for(std::chrono::seconds(1));
            
            // Move to next sub-step
            current_sub_step_++;
        } else {
            // Show progress every 5 seconds
            static double last_progress_time = 0;
            if (elapsed - last_progress_time > 5.0) {
                std::cout << "Movement " << current_step_ << "." << current_sub_step_ 
                          << " in progress... (" << elapsed << "s elapsed)" << std::endl;
                last_progress_time = elapsed;
            }
            return;
        }
    }
    
    // Execute next step
    executeCurrentStep();
}

void ArmLimitsCalibrator::executeCurrentStep()
{
    if (current_step_ == -1) {
        // Step 0: Set initial position
        std::cout << "\nSTEP 0: Setting initial position (hand down beside body)" << std::endl;
        std::vector<double> initial_position = {
            degToRad(0.0),     // base_rotation_joint: 0 degrees
            degToRad(-13.0),   // shoulder_pitch_joint: -13 degrees  
            degToRad(0.0),     // elbow_pitch_joint: 0 degrees
            degToRad(67.0),    // wrist_pitch_joint: 67 degrees
            degToRad(0.0)      // wrist_roll_joint: 0 degrees
        };
        sendJointCommand(initial_position);
        current_target_angles_ = initial_position;
        current_step_ = 0;
        current_sub_step_ = 0;
        movement_start_time_ = this->now();
        waiting_for_movement_ = true;
        return;
    }
    
    switch(current_step_) {
        case 0: // Test base_rotation_joint
            testBaseRotation();
            break;
        case 1: // Test shoulder_pitch_joint
            testShoulderPitch();
            break;
        case 2: // Test elbow_pitch_joint
            testElbowPitch();
            break;
        case 3: // Test wrist_pitch_joint
            testWristPitch();
            break;
        case 4: // Test wrist_roll_joint
            testWristRoll();
            break;
        case 5: // Test workspace combinations
            testWorkspaceCombinations();
            break;
        case 6: // Finish calibration
            finishCalibration();
            return;
        default:
            break;
    }
}

void ArmLimitsCalibrator::testBaseRotation()
{
    std::vector<double> test_angles_deg = {-180, -135, -90, -45, -9, 0, 9, 45, 90, 135, 180};
    
    if (current_sub_step_ < test_angles_deg.size()) {
        double target_deg = test_angles_deg[current_sub_step_];
        std::cout << "\nSTEP 1." << (current_sub_step_ + 1) << "/" << test_angles_deg.size() 
                  << ": Testing base_rotation: " << target_deg << " degrees" << std::endl;
        
        std::vector<double> target_angles = {
            degToRad(target_deg),      // base_rotation
            degToRad(-13.0),           // shoulder_pitch (-13°)
            degToRad(0.0),             // elbow_pitch (0°)
            degToRad(67.0),            // wrist_pitch (67°)
            degToRad(0.0)              // wrist_roll (0°)
        };
        
        std::cout << "Command: base_rotation = " << target_deg << " degrees" << std::endl;
        
        sendJointCommand(target_angles);
        current_target_angles_ = target_angles;
        movement_start_time_ = this->now();
        waiting_for_movement_ = true;
    } else {
        std::cout << "\nBase rotation testing complete (" << test_angles_deg.size() << " positions tested)" << std::endl;
        nextStep();
    }
}

void ArmLimitsCalibrator::testShoulderPitch()
{
    std::vector<double> test_angles_deg = {-32, -25, -20, -15, -13, -10, -5, 0, 10, 20, 30, 45, 60, 75, 90, 120, 150, 178};
    
    if (current_sub_step_ < test_angles_deg.size()) {
        double target_deg = test_angles_deg[current_sub_step_];
        std::cout << "\nSTEP 2." << (current_sub_step_ + 1) << "/" << test_angles_deg.size() 
                  << ": Testing shoulder_pitch: " << target_deg << " degrees" << std::endl;
        
        std::vector<double> target_angles = {
            degToRad(0.0),             // base_rotation (0°)
            degToRad(target_deg),      // shoulder_pitch
            degToRad(0.0),             // elbow_pitch (0°)
            degToRad(67.0),            // wrist_pitch (67°)
            degToRad(0.0)              // wrist_roll (0°)
        };
        
        std::cout << "Command: shoulder_pitch = " << target_deg << " degrees" << std::endl;
        
        sendJointCommand(target_angles);
        current_target_angles_ = target_angles;
        movement_start_time_ = this->now();
        waiting_for_movement_ = true;
    } else {
        std::cout << "\nShoulder pitch testing complete (" << test_angles_deg.size() << " positions tested)" << std::endl;
        nextStep();
    }
}

void ArmLimitsCalibrator::testElbowPitch()
{
    std::vector<double> test_angles_deg = {-180, -150, -120, -90, -60, -45, -30, -15, 0, 15, 30, 45, 60, 90, 120, 150, 180};
    
    if (current_sub_step_ < test_angles_deg.size()) {
        double target_deg = test_angles_deg[current_sub_step_];
        std::cout << "\nSTEP 3." << (current_sub_step_ + 1) << "/" << test_angles_deg.size() 
                  << ": Testing elbow_pitch: " << target_deg << " degrees" << std::endl;
        
        std::vector<double> target_angles = {
            degToRad(0.0),             // base_rotation (0°)
            degToRad(-13.0),           // shoulder_pitch (-13°)
            degToRad(target_deg),      // elbow_pitch
            degToRad(67.0),            // wrist_pitch (67°)
            degToRad(0.0)              // wrist_roll (0°)
        };
        
        std::cout << "Command: elbow_pitch = " << target_deg << " degrees" << std::endl;
        
        sendJointCommand(target_angles);
        current_target_angles_ = target_angles;
        movement_start_time_ = this->now();
        waiting_for_movement_ = true;
    } else {
        std::cout << "\nElbow pitch testing complete (" << test_angles_deg.size() << " positions tested)" << std::endl;
        nextStep();
    }
}

void ArmLimitsCalibrator::testWristPitch()
{
    std::vector<double> test_angles_deg = {-18, -10, 0, 10, 20, 30, 40, 50, 60, 67, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160};
    
    if (current_sub_step_ < test_angles_deg.size()) {
        double target_deg = test_angles_deg[current_sub_step_];
        std::cout << "\nSTEP 4." << (current_sub_step_ + 1) << "/" << test_angles_deg.size() 
                  << ": Testing wrist_pitch: " << target_deg << " degrees" << std::endl;
        
        std::vector<double> target_angles = {
            degToRad(0.0),             // base_rotation (0°)
            degToRad(-13.0),           // shoulder_pitch (-13°)
            degToRad(0.0),             // elbow_pitch (0°)
            degToRad(target_deg),      // wrist_pitch
            degToRad(0.0)              // wrist_roll (0°)
        };
        
        std::cout << "Command: wrist_pitch = " << target_deg << " degrees" << std::endl;
        
        sendJointCommand(target_angles);
        current_target_angles_ = target_angles;
        movement_start_time_ = this->now();
        waiting_for_movement_ = true;
    } else {
        std::cout << "\nWrist pitch testing complete (" << test_angles_deg.size() << " positions tested)" << std::endl;
        nextStep();
    }
}

void ArmLimitsCalibrator::testWristRoll()
{
    std::vector<double> test_angles_deg = {-180, -135, -90, -45, -30, -15, 0, 15, 30, 45, 90, 135, 180};
    
    if (current_sub_step_ < test_angles_deg.size()) {
        double target_deg = test_angles_deg[current_sub_step_];
        std::cout << "\nSTEP 5." << (current_sub_step_ + 1) << "/" << test_angles_deg.size() 
                  << ": Testing wrist_roll: " << target_deg << " degrees" << std::endl;
        
        std::vector<double> target_angles = {
            degToRad(0.0),             // base_rotation (0°)
            degToRad(-13.0),           // shoulder_pitch (-13°)
            degToRad(0.0),             // elbow_pitch (0°)
            degToRad(67.0),            // wrist_pitch (67°)
            degToRad(target_deg)       // wrist_roll
        };
        
        std::cout << "Command: wrist_roll = " << target_deg << " degrees" << std::endl;
        
        sendJointCommand(target_angles);
        current_target_angles_ = target_angles;
        movement_start_time_ = this->now();
        waiting_for_movement_ = true;
    } else {
        std::cout << "\nWrist roll testing complete (" << test_angles_deg.size() << " positions tested)" << std::endl;
        nextStep();
    }
}

void ArmLimitsCalibrator::testWorkspaceCombinations()
{
    std::vector<std::vector<double>> test_combinations = {
        // base, shoulder, elbow, wrist, wrist_roll (in degrees)
        {0, -13, 0, 67, 0},        // Initial position
        {9, -13, 0, 67, 0},        // Forward-right
        {-9, -13, 0, 67, 0},       // Forward-left
        {0, 30, -45, 15, 0},       // Up-forward
        {45, 30, -45, 15, 0},      // Up-right
        {-45, 30, -45, 15, 0},     // Up-left
        {0, -20, -60, 80, 0},      // Down-forward
        {60, -20, -60, 80, 0},     // Down-right
        {-60, -20, -60, 80, 0},    // Down-left
        {90, -13, 0, 67, 0},       // Right maximum
        {-90, -13, 0, 67, 0},      // Left maximum
        {0, 150, -30, -30, 0},     // Up maximum
        {0, -32, -120, 150, 0},    // Down maximum
    };
    
    if (current_sub_step_ < test_combinations.size()) {
        auto& combo = test_combinations[current_sub_step_];
        std::cout << "\nSTEP 6." << (current_sub_step_ + 1) << "/" << test_combinations.size() 
                  << ": Testing workspace combination" << std::endl;
        
        std::vector<double> target_angles = {
            degToRad(combo[0]),  // base
            degToRad(combo[1]),  // shoulder
            degToRad(combo[2]),  // elbow
            degToRad(combo[3]),  // wrist
            degToRad(combo[4])   // wrist_roll
        };
        
        std::cout << "Command: base=" << combo[0] << "°, shoulder=" << combo[1] 
                  << "°, elbow=" << combo[2] << "°, wrist=" << combo[3] << "°" << std::endl;
        
        sendJointCommand(target_angles);
        current_target_angles_ = target_angles;
        movement_start_time_ = this->now();
        waiting_for_movement_ = true;
    } else {
        std::cout << "\nWorkspace testing complete (" << test_combinations.size() << " combinations tested)" << std::endl;
        nextStep();
    }
}

void ArmLimitsCalibrator::nextStep()
{
    current_step_++;
    current_sub_step_ = 0;
    waiting_for_movement_ = false;
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "MOVING TO STEP " << current_step_ << std::endl;
    std::cout << "========================================" << std::endl;
    
    // Longer pause between main steps
    rclcpp::sleep_for(std::chrono::seconds(2));
}

void ArmLimitsCalibrator::showCurrentState()
{
    std::cout << "Current joint states:" << std::endl;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        std::cout << "   " << joint_names_[i] << ": " << current_joint_positions_[i] 
                  << " rad (" << radToDeg(current_joint_positions_[i]) << "°)" << std::endl;
    }
}

void ArmLimitsCalibrator::finishCalibration()
{
    calibration_complete_ = true;
    calibration_timer_->cancel();
    
    std::cout << "\nCALIBRATION COMPLETE!" << std::endl;
    std::cout << "====================================================" << std::endl;
    
    printCalibrationResults();
    saveCalibrationResults();
    saveLimitsHeader();
    
    std::cout << "\nResults saved to 'arm_limits.txt' and 'arm_limits.h'" << std::endl;
    std::cout << "Shutting down in 10 seconds..." << std::endl;
    
    rclcpp::sleep_for(std::chrono::seconds(10));
    rclcpp::shutdown();
}

void ArmLimitsCalibrator::printCalibrationResults()
{
    std::cout << "\nJOINT LIMITS FOUND:" << std::endl;
    std::cout << "====================================================" << std::endl;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        std::cout << joint_names_[i] << ":" << std::endl;
        std::cout << "  MIN: " << joint_limits_[i].min << " rad (" << radToDeg(joint_limits_[i].min) << "°)" << std::endl;
        std::cout << "  MAX: " << joint_limits_[i].max << " rad (" << radToDeg(joint_limits_[i].max) << "°)" << std::endl;
        std::cout << "  RANGE: " << (joint_limits_[i].max - joint_limits_[i].min) << " rad (" 
                  << radToDeg(joint_limits_[i].max - joint_limits_[i].min) << "°)" << std::endl;
    }
}

void ArmLimitsCalibrator::saveCalibrationResults()
{
    std::ofstream file("arm_limits.txt");
    file << "ARM JOINT LIMITS - CALIBRATION RESULTS\n";
    file << "======================================\n\n";
    
    file << "JOINT LIMITS:\n";
    file << "-------------\n";
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        file << joint_names_[i] << ":\n";
        file << "  MIN: " << joint_limits_[i].min << " rad (" << radToDeg(joint_limits_[i].min) << "°)\n";
        file << "  MAX: " << joint_limits_[i].max << " rad (" << radToDeg(joint_limits_[i].max) << "°)\n";
        file << "  RANGE: " << (joint_limits_[i].max - joint_limits_[i].min) << " rad (" 
             << radToDeg(joint_limits_[i].max - joint_limits_[i].min) << "°)\n\n";
    }
    
    file.close();
}

void ArmLimitsCalibrator::saveLimitsHeader()
{
    std::ofstream file("arm_limits.h");
    file << "#ifndef ARM_LIMITS_H\n";
    file << "#define ARM_LIMITS_H\n\n";
    
    file << "// Auto-generated arm limits from calibration\n";
    file << "// Generated by ArmLimitsCalibrator\n\n";
    
    file << "#include <vector>\n";
    file << "#include <string>\n\n";
    
    file << "struct JointLimit {\n";
    file << "    double min;\n";
    file << "    double max;\n";
    file << "    std::string name;\n";
    file << "};\n\n";
    
    file << "// Joint names\n";
    file << "const std::vector<std::string> JOINT_NAMES = {\n";
    for (const auto& name : joint_names_) {
        file << "    \"" << name << "\",\n";
    }
    file << "};\n\n";
    
    file << "// Measured joint limits (radians)\n";
    file << "const std::vector<JointLimit> JOINT_LIMITS = {\n";
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        file << "    {" << joint_limits_[i].min << ", " << joint_limits_[i].max << ", \"" << joint_names_[i] << "\"},\n";
    }
    file << "};\n\n";
    
    file << "#endif // ARM_LIMITS_H\n";
    file.close();
}

bool ArmLimitsCalibrator::checkMovementCompletion(const std::vector<double>& target_angles)
{
    double tolerance = 0.2;
    for (size_t i = 0; i < target_angles.size(); ++i) {
        if (fabs(current_joint_positions_[i] - target_angles[i]) > tolerance) {
            return false;
        }
    }
    return true;
}

void ArmLimitsCalibrator::sendJointCommand(const std::vector<double>& joint_angles)
{
    auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
    trajectory_msg.joint_names = joint_names_;
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = joint_angles;
    point.time_from_start = rclcpp::Duration::from_seconds(12.0);
    
    trajectory_msg.points.push_back(point);
    trajectory_pub_->publish(trajectory_msg);
}

void ArmLimitsCalibrator::startCalibration()
{
    // Reset calibration state if needed
    calibration_complete_ = false;
    current_step_ = -1;
    current_sub_step_ = 0;
}

void ArmLimitsCalibrator::stopCalibration()
{
    calibration_complete_ = true;
    if (calibration_timer_) {
        calibration_timer_->cancel();
    }
}