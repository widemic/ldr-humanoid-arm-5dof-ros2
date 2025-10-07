#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <fcntl.h>
#include <vector>
#include <string>
#include <map>
#include <iomanip>

class KeyboardControl : public rclcpp::Node
{
public:
    KeyboardControl() : Node("keyboard_control")
    {
        trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&KeyboardControl::joint_state_callback, this, std::placeholders::_1));

        // Parameters
        this->declare_parameter<double>("position_step", 0.5); // increased for visible movement
        this->declare_parameter<double>("duration", 2.0);

        position_step_ = this->get_parameter("position_step").as_double();
        duration_ = this->get_parameter("duration").as_double();

        // Actual joint order (as in controller)
        joint_names_ = {
            "base_rotation_joint",
            "shoulder_pitch_joint",
            "elbow_pitch_joint",
            "wrist_pitch_joint",
            "wrist_roll_joint"
        };

        // Joint limits
        joint_limits_ = {
            {"base_rotation_joint", {-3.0, 3.0}},
            {"shoulder_pitch_joint", {-0.5, 3.0}},
            {"elbow_pitch_joint", {-3.0, 3.0}},
            {"wrist_pitch_joint", {-0.25, 2.7}},
            {"wrist_roll_joint", {-3.0, 3.0}}
        };

        // ✅ Correct logical → actual mapping
        // Base=0, Shoulder=2, Elbow=1, WristPitch=3, WristRoll=4
        joint_mapping_ = {
            0, // Base
            2, // Shoulder
            1, // Elbow
            3, // Wrist Pitch
            4  // Wrist Roll
        };

        current_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0};
        target_positions_ = current_positions_;

        RCLCPP_INFO(this->get_logger(), "✅ Keyboard Control Node Started");
        print_instructions();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&KeyboardControl::timer_callback, this));
    }

private:
    void print_instructions()
    {
        std::cout << "\n=== 5DOF HUMANOID ARM - KEYBOARD CONTROL ===\n";
        std::cout << "Controls:\n";
        std::cout << "Base rotation:        q - left | w - right\n";
        std::cout << "Shoulder pitch:       a - up   | s - down\n";
        std::cout << "Elbow pitch:          z - up   | x - down\n";
        std::cout << "Wrist pitch:          e - up   | r - down\n";
        std::cout << "Wrist roll:           c - left | v - right\n";
        std::cout << "Speed:                + increase | - decrease step\n";
        std::cout << "Reset:                0 - zero position\n";
        std::cout << "Home:                 h - home position\n";
        std::cout << "Test:                 t - send visible test move\n";
        std::cout << "Exit:                 CTRL+C\n";
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); i++) {
            for (size_t j = 0; j < joint_names_.size(); j++) {
                if (msg->name[i] == joint_names_[j]) {
                    current_positions_[j] = msg->position[i];
                    break;
                }
            }
        }

        if (!initialized_) {
            target_positions_ = current_positions_;
            initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Initial joint positions captured:");
            for (size_t i = 0; i < joint_names_.size(); i++)
                RCLCPP_INFO(this->get_logger(), "  [%zu] %s = %.3f",
                            i, joint_names_[i].c_str(), current_positions_[i]);
        }
    }

    void timer_callback()
    {
        if (keyboard_hit()) {
            int c = getchar();
            process_key(c);
        }
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

    void process_key(int key)
    {
        if (!initialized_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for joint_states...");
            return;
        }

        int logical_joint = -1;
        double direction = 0.0;

        switch (key) {
            // Base rotation
            case 'q': logical_joint = 0; direction = 1.0; break;
            case 'w': logical_joint = 0; direction = -1.0; break;

            // Shoulder pitch
            case 'a': logical_joint = 1; direction = 1.0; break;
            case 's': logical_joint = 1; direction = -1.0; break;

            // Elbow pitch
            case 'z': logical_joint = 2; direction = 1.0; break;
            case 'x': logical_joint = 2; direction = -1.0; break;

            // Wrist pitch
            case 'e': logical_joint = 3; direction = 1.0; break;
            case 'r': logical_joint = 3; direction = -1.0; break;

            // Wrist roll
            case 'c': logical_joint = 4; direction = 1.0; break;
            case 'v': logical_joint = 4; direction = -1.0; break;

            // Speed up / down
            case '+': case '=':
                position_step_ *= 1.2;
                RCLCPP_INFO(this->get_logger(), "Step size: %.2f", position_step_);
                return;
            case '-': case '_':
                position_step_ /= 1.2;
                RCLCPP_INFO(this->get_logger(), "Step size: %.2f", position_step_);
                return;

            // Reset
            case '0':
                target_positions_ = {0, 0, 0, 0, 0};
                RCLCPP_INFO(this->get_logger(), "Reset all joints");
                send_trajectory();
                return;

            // Home
            case 'h':
                target_positions_ = {0.0, -0.5, 0.5, 0.0, 0.0};
                RCLCPP_INFO(this->get_logger(), "Home position");
                send_trajectory();
                return;

            // Test
            case 't':
                target_positions_ = {0.0, 2.0, -1.0, 0.0, 0.0};
                RCLCPP_INFO(this->get_logger(), "Test trajectory sent");
                send_trajectory();
                return;

            default:
                return;
        }

        // Update joint position
        if (logical_joint >= 0 && logical_joint < 5) {
            int actual_index = joint_mapping_[logical_joint];
            double new_position = target_positions_[actual_index] + direction * position_step_;

            // Clamp within limits
            auto limits = joint_limits_[joint_names_[actual_index]];
            new_position = std::max(limits.first, std::min(limits.second, new_position));

            target_positions_[actual_index] = new_position;

            RCLCPP_INFO(this->get_logger(), "Moved %s to %.3f",
                        joint_names_[actual_index].c_str(), new_position);
            send_trajectory();
        }
    }

    void send_trajectory()
    {
        auto traj = trajectory_msgs::msg::JointTrajectory();
        traj.header.stamp.nanosec = 0;
        traj.header.stamp.sec = 0;
        traj.header.frame_id = "base_link";
        traj.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = target_positions_;
        point.time_from_start = rclcpp::Duration::from_seconds(duration_);
        traj.points.push_back(point);

        trajectory_pub_->publish(traj);

        RCLCPP_INFO(this->get_logger(), "✅ Trajectory published:");
        for (size_t i = 0; i < joint_names_.size(); i++)
            RCLCPP_INFO(this->get_logger(), "  %s: %.3f", joint_names_[i].c_str(), target_positions_[i]);
    }

    // ROS members
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Joint data
    std::vector<std::string> joint_names_;
    std::vector<double> current_positions_;
    std::vector<double> target_positions_;
    std::vector<int> joint_mapping_;
    std::map<std::string, std::pair<double, double>> joint_limits_;
    double position_step_;
    double duration_;
    bool initialized_ = false;
};

void signal_handler(int)
{
    std::cout << "\nExiting keyboard control.\n";
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    auto node = std::make_shared<KeyboardControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
