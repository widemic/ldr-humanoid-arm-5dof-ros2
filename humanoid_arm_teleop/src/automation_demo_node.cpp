#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>

class AutomationDemo : public rclcpp::Node
{
public:
    AutomationDemo() : Node("automation_demo")
    {
        // Publisher for trajectory commands
        trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);
        
        // Joint names from your working command
        joint_names_ = {
            "base_rotation_joint",
            "shoulder_pitch_joint", 
            "elbow_pitch_joint",
            "wrist_pitch_joint",
            "wrist_roll_joint"
        };
        
        // Demo parameters
        this->declare_parameter<double>("movement_duration", 2.0);
        movement_duration_ = this->get_parameter("movement_duration").as_double();
        
        RCLCPP_INFO(this->get_logger(), "🎪 HUMANOID ARM AUTOMATION DEMO STARTED!");
        RCLCPP_INFO(this->get_logger(), "=========================================");
        print_instructions();
        
        current_pattern_ = 0;
        is_running_ = false;
        
        // Timer for keyboard input
        input_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&AutomationDemo::input_loop, this));
            
        // Timer for automation patterns
        automation_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&AutomationDemo::automation_loop, this));
    }

private:
    void print_instructions()
    {
        std::cout << "\n=== AUTOMATION PATTERNS ===" << std::endl;
        std::cout << "1️⃣  Wave Pattern" << std::endl;
        std::cout << "2️⃣  Circular Motion" << std::endl;
        std::cout << "3️⃣  Random Exploration" << std::endl;
        std::cout << "4️⃣  Test Sequence" << std::endl;
        std::cout << "5️⃣  Figure Eight" << std::endl;
        std::cout << "0️⃣  Stop Automation" << std::endl;
        std::cout << "r️⃣  Reset to Zero" << std::endl;
        std::cout << "q️⃣  Quit" << std::endl;
        std::cout << "============================" << std::endl;
        std::cout << "Current Pattern: " << pattern_names_[current_pattern_] << std::endl;
        std::cout << "Status: " << (is_running_ ? "RUNNING" : "STOPPED") << std::endl;
    }
    
    bool keyboard_hit()
    {
        struct termios oldt, newt;
        int ch;
        int oldf;

        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

        ch = getchar();

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);

        if (ch != EOF) {
            ungetc(ch, stdin);
            return true;
        }

        return false;
    }
    
    void input_loop()
    {
        if (keyboard_hit()) {
            int key = getchar();
            process_input(key);
        }
    }
    
    void process_input(int key)
    {
        switch(key) {
            case '1':
                current_pattern_ = 0;
                is_running_ = true;
                std::cout << "🔄 Starting: " << pattern_names_[current_pattern_] << std::endl;
                break;
            case '2':
                current_pattern_ = 1;
                is_running_ = true;
                std::cout << "🔄 Starting: " << pattern_names_[current_pattern_] << std::endl;
                break;
            case '3':
                current_pattern_ = 2;
                is_running_ = true;
                std::cout << "🔄 Starting: " << pattern_names_[current_pattern_] << std::endl;
                break;
            case '4':
                current_pattern_ = 3;
                is_running_ = true;
                std::cout << "🔄 Starting: " << pattern_names_[current_pattern_] << std::endl;
                break;
            case '5':
                current_pattern_ = 4;
                is_running_ = true;
                std::cout << "🔄 Starting: " << pattern_names_[current_pattern_] << std::endl;
                break;
            case '0':
                is_running_ = false;
                std::cout << "⏹️  Automation Stopped" << std::endl;
                break;
            case 'r':
                send_reset_position();
                break;
            case 'q':
                std::cout << "👋 Exiting..." << std::endl;
                rclcpp::shutdown();
                return;
            default:
                return;
        }
        
        // Update display
        std::cout << "Current: " << pattern_names_[current_pattern_] 
                  << " | Status: " << (is_running_ ? "RUNNING" : "STOPPED") << std::endl;
    }
    
    void automation_loop()
    {
        if (!is_running_) return;
        
        std::vector<double> positions;
        counter_++;
        
        switch(current_pattern_) {
            case 0: positions = wave_pattern(); break;
            case 1: positions = circular_motion(); break;
            case 2: positions = random_exploration(); break;
            case 3: positions = test_sequence(); break;
            case 4: positions = figure_eight_pattern(); break;
            default: positions = wave_pattern(); break;
        }
        
        send_trajectory(positions);
    }
    
    std::vector<double> wave_pattern()
    {
        double t = counter_ * 0.3;
        return {
            std::sin(t) * 0.5,                    // Base
            std::sin(t + 1.0) * 0.4,              // Shoulder
            std::sin(t + 2.0) * 0.6,              // Elbow
            std::sin(t + 3.0) * 0.3,              // Wrist pitch
            std::sin(t + 4.0) * 0.5               // Wrist roll
        };
    }
    
    std::vector<double> circular_motion()
    {
        double angle = counter_ * M_PI / 6.0;
        return {
            std::sin(angle) * 0.6,                // Base
            std::cos(angle) * 0.4,                // Shoulder
            std::sin(angle + M_PI/2) * 0.5,       // Elbow
            std::cos(angle) * 0.3,                // Wrist pitch
            std::sin(angle) * 0.4                 // Wrist roll
        };
    }
    
    std::vector<double> random_exploration()
    {
        // Pseudo-random but smooth exploration
        double t = counter_ * 0.2;
        return {
            std::sin(t * 1.1) * 0.5,              // Base
            std::cos(t * 1.3) * 0.4,              // Shoulder
            std::sin(t * 1.5) * 0.6,              // Elbow
            std::cos(t * 1.7) * 0.3,              // Wrist pitch
            std::sin(t * 1.9) * 0.4               // Wrist roll
        };
    }
    
    std::vector<double> test_sequence()
    {
        // Test positions including your working command
        std::vector<std::vector<double>> test_positions = {
            {0.0, 0.0, 0.0, 0.0, 0.0},           // Zero position
            {0.5, -0.3, 0.7, -0.2, 0.0},         // Salute
            {0.0, 0.0, -1.0, 0.0, 0.0},          // Your working command!
            {-0.5, 0.3, -0.5, 0.4, 0.5},         // Opposite
            {0.3, -0.4, 0.2, -0.1, 0.5}          // Circle-like
        };
        
        int index = (counter_ / 3) % test_positions.size();
        return test_positions[index];
    }
    
    std::vector<double> figure_eight_pattern()
    {
        double t = counter_ * 0.2;
        return {
            std::sin(t) * 0.6,                    // Base - pattern 8
            std::sin(2 * t) * 0.3,                // Shoulder
            std::cos(t) * 0.5,                    // Elbow
            std::sin(t) * 0.4,                    // Wrist pitch
            std::cos(2 * t) * 0.3                 // Wrist roll
        };
    }
    
    void send_trajectory(const std::vector<double>& positions)
    {
        auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
        
        // Set timestamp to 0 as requested
        trajectory_msg.header.stamp.sec = 0;
        trajectory_msg.header.stamp.nanosec = 0;
        trajectory_msg.header.frame_id = "base_link";
        trajectory_msg.joint_names = joint_names_;
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;
        point.time_from_start = rclcpp::Duration::from_seconds(movement_duration_);
        
        trajectory_msg.points.push_back(point);
        
        // Log the movement
        std::string position_str = "[";
        for (size_t i = 0; i < positions.size(); ++i) {
            position_str += std::to_string(positions[i]).substr(0, 4);
            if (i < positions.size() - 1) position_str += ", ";
        }
        position_str += "]";
        
        RCLCPP_INFO(this->get_logger(), "🤖 %s: %s", 
                   pattern_names_[current_pattern_].c_str(), position_str.c_str());
        
        trajectory_pub_->publish(trajectory_msg);
    }
    
    void send_reset_position()
    {
        std::vector<double> reset_position = {0.0, 0.0, 0.0, 0.0, 0.0};
        std::cout << "🔄 Resetting to zero position" << std::endl;
        send_trajectory(reset_position);
    }
    
    // ROS2 members
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::TimerBase::SharedPtr input_timer_;
    rclcpp::TimerBase::SharedPtr automation_timer_;
    
    // Demo state
    std::vector<std::string> joint_names_;
    std::vector<std::string> pattern_names_ = {
        "Wave Pattern", "Circular Motion", "Random Exploration", 
        "Test Sequence", "Figure Eight"
    };
    
    int current_pattern_ = 0;
    int counter_ = 0;
    bool is_running_ = false;
    double movement_duration_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    std::cout << "🎪 Starting Humanoid Arm Automation Demo..." << std::endl;
    std::cout << "===========================================" << std::endl;
    
    auto node = std::make_shared<AutomationDemo>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
    
    std::cout << "👋 Automation Demo finished!" << std::endl;
    rclcpp::shutdown();
    
    return 0;
}