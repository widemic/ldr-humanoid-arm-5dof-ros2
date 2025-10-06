"""
Reusable launch modules for humanoid arm bringup.

This package provides Pythonic, composable functions for building launch files.
Each module handles a specific concern (robot description, controllers, sensors, etc.)
and can be mixed and matched to create different launch configurations.

Modules:
    robot_description: Load URDF and spawn robot_state_publisher
    controllers: Spawn ros2_control controllers with sequencing
    sensors: Spawn sensor broadcasters (F/T, IMU)
    gazebo: Launch Gazebo simulation and spawn robot
    rviz: Launch RViz2 visualization

Usage:
    from modules import robot_description, controllers, sensors

    # In your launch file:
    robot_desc = robot_description.load_urdf(use_fake_hardware=True)
    rsp = robot_description.get_robot_state_publisher(robot_desc)
    ctrl_nodes = controllers.get_standard_controllers()
    sensor_nodes = sensors.get_all_sensors()

    return LaunchDescription([rsp] + ctrl_nodes + sensor_nodes)
"""

__all__ = [
    'robot_description',
    'controllers',
    'sensors',
    'gazebo',
    'rviz',
]
