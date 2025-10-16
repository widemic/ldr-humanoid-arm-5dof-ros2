#include "../include/humanoid_arm_utilities/arm_limits_calibrator.h"
#include <iostream>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    std::cout << "ARM LIMITS CALIBRATOR" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "This node will test joint positions to determine real arm limits." << std::endl;
    std::cout << "Total steps: 7 (including initialization)" << std::endl;
    std::cout << "Each movement has 15 second timeout" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Press Enter to start calibration..." << std::endl;
    std::cin.get();
    
    auto node = std::make_shared<ArmLimitsCalibrator>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
    
    rclcpp::shutdown();
    return 0;
}