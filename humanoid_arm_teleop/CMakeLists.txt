cmake_minimum_required(VERSION 3.8)
project(humanoid_arm_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Joystick teleoperation node
add_executable(joystick_teleop_node
  src/joystick_teleop_node.cpp
  src/joystick_teleop.cpp
)

target_include_directories(joystick_teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(joystick_teleop_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Keyboard teleoperation node
add_executable(keyboard_teleop_node
  src/keyboard_teleop_node.cpp
  src/keyboard_teleop.cpp
)

target_include_directories(keyboard_teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(keyboard_teleop_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Contact-based manipulation helper node
add_executable(contact_manipulation_node
  src/contact_manipulation_node.cpp
  src/contact_manipulation.cpp
)

target_include_directories(contact_manipulation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(contact_manipulation_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# DualSense simple teleoperation node
add_executable(dualsense_teleop_node
  src/dualsense_teleop.cpp
)

target_include_directories(dualsense_teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dualsense_teleop_node
  rclcpp
  sensor_msgs
  trajectory_msgs
)

# Install executables
install(TARGETS
  joystick_teleop_node
  keyboard_teleop_node
  contact_manipulation_node
  dualsense_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()