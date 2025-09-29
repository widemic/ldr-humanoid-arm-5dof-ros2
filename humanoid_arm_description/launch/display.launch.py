#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro


def launch_setup(context, *args, **kwargs):
    # Get parameters
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    robot_controller = LaunchConfiguration('robot_controller')

    # Get URDF via xacro
    robot_description_content = xacro.process_file(
        os.path.join(
            get_package_share_directory('humanoid_arm_description'),
            'urdf',
            'humanoid_arm_5dof_ros2_control.urdf.xacro'
        ),
        mappings={
            'use_fake_hardware': use_fake_hardware.perform(context),
            'robot_controller': robot_controller.perform(context)
        }
    )
    robot_description = {'robot_description': robot_description_content.toxml()}

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # Joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz
    rviz_config_file_path = PathJoinSubstitution([
        FindPackageShare('humanoid_arm_description'),
        'config',
        rviz_config_file
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file_path],
        condition=LaunchConfiguration('use_rviz')
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]

    return nodes_to_start


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Start robot with fake hardware mirroring command to its states.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value='humanoid_arm_5dof.rviz',
            description='RViz config file (absolute path) to use when launching rviz.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_controller',
            default_value='joint_trajectory_controller',
            description='Robot controller to start.',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])