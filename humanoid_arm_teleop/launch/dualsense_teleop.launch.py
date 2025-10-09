#!/usr/bin/env python3

"""
DualSense teleop launch file.

Launches joy_node and dualsense_teleop for simple 4-axis control.

Usage:
    # First start the robot (any mode):
    ros2 launch humanoid_arm_bringup gazebo_simple.launch.py

    # Then in another terminal:
    ros2 launch humanoid_arm_teleop dualsense_teleop.launch.py

    # Or specify custom device:
    ros2 launch humanoid_arm_teleop dualsense_teleop.launch.py device:=/dev/input/js1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ========== Launch Arguments ==========
    declared_arguments = [
        DeclareLaunchArgument(
            'device',
            default_value='/dev/input/js0',
            description='Joystick device path'
        ),
    ]

    # ========== Configuration ==========
    device = LaunchConfiguration('device')

    config_file = PathJoinSubstitution([
        FindPackageShare('humanoid_arm_teleop'),
        'config',
        'dualsense_config.yaml'
    ])

    # ========== Nodes ==========

    # Joy node - reads joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': device,
            'deadzone': 0.05,
            'autorepeat_rate': 50.0,
        }],
        output='screen'
    )

    # DualSense teleop node
    dualsense_teleop_node = Node(
        package='humanoid_arm_teleop',
        executable='dualsense_teleop_node',
        name='dualsense_teleop',
        parameters=[config_file],
        output='screen'
    )

    # ========== Launch Description ==========
    nodes_to_start = [
        joy_node,
        dualsense_teleop_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
