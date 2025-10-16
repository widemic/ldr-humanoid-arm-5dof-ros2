cmake_minimum_required(VERSION 3.8)
project(humanoid_arm_utilities)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create the calibrator executable
add_executable(arm_limits_calibrator
  src/arm_limits_calibrator_node.cpp
  src/arm_limits_calibrator.cpp
)

target_include_directories(arm_limits_calibrator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(arm_limits_calibrator
  rclcpp
  trajectory_msgs
  sensor_msgs
  geometry_msgs
)

# Install the executable
install(TARGETS
  arm_limits_calibrator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
