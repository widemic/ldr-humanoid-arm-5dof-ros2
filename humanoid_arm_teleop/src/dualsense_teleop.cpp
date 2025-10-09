// Simple DualSense teleop for 4-axis position control
// Left stick: base_rotation + shoulder_pitch
// Right stick: elbow_pitch + wrist_pitch

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <chrono>
#include <vector>
#include <string>
#include <algorithm>

using namespace std::chrono_literals;

class DualSenseTeleop : public rclcpp::Node
{
public:
    DualSenseTeleop() : Node("dualsense_teleop")
    {
        // Declare parameters
        this->declare_parameter("publish_rate", 10.0);
        this->declare_parameter("deadzone", 0.1);
        this->declare_parameter("velocity_scale", 0.3);
        this->declare_parameter("deadman_button", 4);  // L1
        this->declare_parameter("emergency_stop_button", 5);  // R1

        // Get parameters
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        deadzone_ = this->get_parameter("deadzone").as_double();
        velocity_scale_ = this->get_parameter("velocity_scale").as_double();
        deadman_button_ = this->get_parameter("deadman_button").as_int();
        emergency_stop_button_ = this->get_parameter("emergency_stop_button").as_int();

        // Joint names (first 4 joints)
        joint_names_ = {
            "base_rotation_joint",
            "shoulder_pitch_joint",
            "elbow_pitch_joint",
            "wrist_pitch_joint"
        };

        // Initialize current positions to zero
        current_positions_.resize(4, 0.0);
        target_positions_.resize(4, 0.0);

        // Subscribers
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&DualSenseTeleop::joyCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&DualSenseTeleop::jointStateCallback, this, std::placeholders::_1));

        // Publisher
        trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        // Timer for publishing commands
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&DualSenseTeleop::publishCommand, this));

        RCLCPP_INFO(this->get_logger(), "DualSense teleop node started");
        RCLCPP_INFO(this->get_logger(), "Hold L1 (button %d) to enable motion", deadman_button_);
        RCLCPP_INFO(this->get_logger(), "Press R1 (button %d) for emergency stop", emergency_stop_button_);
        RCLCPP_INFO(this->get_logger(), "Controls:");
        RCLCPP_INFO(this->get_logger(), "  Left stick X/Y  -> base_rotation / shoulder_pitch");
        RCLCPP_INFO(this->get_logger(), "  Right stick X/Y -> elbow_pitch / wrist_pitch");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        last_joy_msg_ = msg;

        // Check emergency stop
        if (msg->buttons.size() > static_cast<size_t>(emergency_stop_button_) &&
            msg->buttons[emergency_stop_button_] == 1)
        {
            emergency_stop_ = true;
            RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP pressed!");
            return;
        }

        // Check deadman switch
        if (msg->buttons.size() > static_cast<size_t>(deadman_button_) &&
            msg->buttons[deadman_button_] == 1)
        {
            deadman_pressed_ = true;
            emergency_stop_ = false;  // Reset emergency stop when deadman is pressed

            // Get joystick axes with deadzone
            if (msg->axes.size() >= 4)
            {
                double left_x = applyDeadzone(msg->axes[0]);   // base_rotation
                double left_y = applyDeadzone(msg->axes[1]);   // shoulder_pitch
                double right_x = applyDeadzone(msg->axes[2]);  // elbow_pitch
                double right_y = applyDeadzone(msg->axes[3]);  // wrist_pitch

                // Update target positions (integrate velocity)
                double dt = 1.0 / publish_rate_;
                target_positions_[0] += left_x * velocity_scale_ * dt;   // base_rotation
                target_positions_[1] += -left_y * velocity_scale_ * dt;  // shoulder_pitch (inverted)
                target_positions_[2] += right_x * velocity_scale_ * dt;  // elbow_pitch
                target_positions_[3] += -right_y * velocity_scale_ * dt; // wrist_pitch (inverted)

                // Apply joint limits
                clampToLimits();
            }
        }
        else
        {
            deadman_pressed_ = false;
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Update current positions from joint_states
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
            if (it != msg->name.end())
            {
                size_t index = std::distance(msg->name.begin(), it);
                if (index < msg->position.size())
                {
                    current_positions_[i] = msg->position[index];

                    // Initialize target position on first read
                    if (!initialized_)
                    {
                        target_positions_[i] = current_positions_[i];
                    }
                }
            }
        }
        initialized_ = true;
    }

    void publishCommand()
    {
        if (!initialized_ || !deadman_pressed_ || emergency_stop_)
        {
            return;
        }

        // Create trajectory message
        auto traj_msg = trajectory_msgs::msg::JointTrajectory();
        traj_msg.header.stamp = this->now();
        traj_msg.joint_names = joint_names_;

        // Single trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = target_positions_;
        point.time_from_start = rclcpp::Duration::from_seconds(0.1);  // 100ms lookahead

        traj_msg.points.push_back(point);

        // Publish
        trajectory_pub_->publish(traj_msg);
    }

    double applyDeadzone(double value)
    {
        if (std::abs(value) < deadzone_)
        {
            return 0.0;
        }
        // Scale to maintain smooth response
        double sign = (value > 0) ? 1.0 : -1.0;
        return sign * (std::abs(value) - deadzone_) / (1.0 - deadzone_);
    }

    void clampToLimits()
    {
        // Joint limits (from URDF)
        const std::vector<double> lower_limits = {-3.14, -0.55, -3.14, -0.31};
        const std::vector<double> upper_limits = {3.14, 3.1, 3.14, 2.8};

        for (size_t i = 0; i < target_positions_.size(); ++i)
        {
            target_positions_[i] = std::clamp(
                target_positions_[i],
                lower_limits[i],
                upper_limits[i]
            );
        }
    }

    // ROS communication
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    std::vector<std::string> joint_names_;
    std::vector<double> current_positions_;
    std::vector<double> target_positions_;
    sensor_msgs::msg::Joy::SharedPtr last_joy_msg_;

    bool initialized_ = false;
    bool deadman_pressed_ = false;
    bool emergency_stop_ = false;

    // Parameters
    double publish_rate_;
    double deadzone_;
    double velocity_scale_;
    int deadman_button_;
    int emergency_stop_button_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualSenseTeleop>());
    rclcpp::shutdown();
    return 0;
}
