#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
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
    initial_joint_controller = LaunchConfiguration('initial_joint_controller')

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

    # Robot controllers configuration
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('humanoid_arm_control'),
        'config',
        'controllers.yaml',
    ])

    # Control node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Robot controller spawner
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[initial_joint_controller, '--controller-manager', '/controller_manager'],
    )

    # Force torque sensor broadcasters
    tcp_ft_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tcp_force_torque_broadcaster', '--controller-manager', '/controller_manager'],
    )

    forearm_ft_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forearm_force_torque_broadcaster', '--controller-manager', '/controller_manager'],
    )

    wrist_ft_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wrist_force_torque_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # IMU broadcaster spawner
    imu_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_imu_broadcaster', '--controller-manager', '/controller_manager'],
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
        condition=IfCondition(use_rviz)
    )

    # Delay rviz start after joint_state_broadcaster
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Delay sensor broadcasters after joint state broadcaster
    delay_ft_broadcasters = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                tcp_ft_broadcaster_spawner,
                forearm_ft_broadcaster_spawner,
                wrist_ft_broadcaster_spawner,
                imu_broadcaster_spawner,
            ],
        )
    )

    nodes_to_start = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_ft_broadcasters,
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

    declared_arguments.append(
        DeclareLaunchArgument(
            'initial_joint_controller',
            default_value='joint_trajectory_controller',
            description='Initially loaded robot controller.',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])