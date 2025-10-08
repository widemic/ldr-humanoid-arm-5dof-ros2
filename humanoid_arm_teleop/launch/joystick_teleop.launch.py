#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration('config_filepath')

    # Paths
    pkg_share = FindPackageShare('humanoid_arm_teleop')
    default_config_file = PathJoinSubstitution(
        [pkg_share, 'config', 'joystick_config.yaml']
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'joy_config',
            default_value='ps5',
            description='Joystick type: ps5, ps4, xbox'
        ),
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path'
        ),
        DeclareLaunchArgument(
            'config_filepath',
            default_value=default_config_file,
            description='Path to joystick configuration YAML file'
        ),

        # Joy node (publishes sensor_msgs/Joy from controller input)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.1,
                'autorepeat_rate': 50.0,
            }],
            output='screen'
        ),

        # Joystick teleop node (converts Joy messages to joint commands)
        Node(
            package='humanoid_arm_teleop',
            executable='joystick_teleop_node',
            name='joystick_teleop',
            parameters=[config_filepath],
            output='screen',
            emulate_tty=True
        ),
    ])
