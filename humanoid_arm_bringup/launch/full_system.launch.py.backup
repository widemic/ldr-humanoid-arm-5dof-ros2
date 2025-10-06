#!/usr/bin/env python3

"""
Full system launch: Gazebo simulation + MoveIt2 + RViz
Allows interactive control via MoveIt and see the robot move in Gazebo
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'world_file',
            default_value='empty_world.sdf',
            description='Gazebo world file to load.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Gazebo with GUI.',
        )
    )

    # Get parameters
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    # Get URDF via xacro
    robot_description_content = xacro.process_file(
        os.path.join(
            get_package_share_directory('humanoid_arm_description'),
            'urdf',
            'humanoid_arm_5dof_ros2_control.urdf.xacro'
        ),
        mappings={
            'use_fake_hardware': 'false',  # Use Gazebo
            'robot_controller': 'joint_trajectory_controller'
        }
    )
    robot_description = {'robot_description': robot_description_content.toxml()}

    # Load SRDF (Semantic Robot Description Format)
    robot_description_semantic_path = os.path.join(
        get_package_share_directory('humanoid_arm_moveit_config'),
        'config',
        'humanoid_arm_5dof.srdf'
    )
    with open(robot_description_semantic_path, 'r') as file:
        robot_description_semantic = {'robot_description_semantic': file.read()}

    # Robot controllers configuration
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('humanoid_arm_control'),
        'config',
        'controllers.yaml',
    ])

    # World file path
    world_file_path = PathJoinSubstitution([
        FindPackageShare('humanoid_arm_bringup'),
        'worlds',
        world_file
    ])

    # Gazebo Harmonic (gz-sim) launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world_file_path],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Spawn robot in Gazebo Harmonic
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', 'humanoid_arm_5dof'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Bridge for Gazebo topics
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Robot controller spawner
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Delay spawn after robot state publisher starts publishing
    delay_spawn_after_rsp = TimerAction(
        period=3.0,
        actions=[spawn_entity]
    )

    # Delay joint state broadcaster after spawn
    delay_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner]
    )

    # Delay controller after joint state broadcaster
    delay_robot_controller = TimerAction(
        period=7.0,
        actions=[robot_controller_spawner]
    )

    # MoveIt launch (starts after controllers are loaded)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('humanoid_arm_moveit_config'),
                'launch',
                'move_group_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
        }.items()
    )

    delay_moveit = TimerAction(
        period=9.0,
        actions=[moveit_launch]
    )

    # RViz with MoveIt config
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('humanoid_arm_moveit_config'),
        'config',
        'moveit.rviz'
    ])

    # Load kinematics config for interactive markers
    kinematics_yaml_path = os.path.join(
        get_package_share_directory('humanoid_arm_moveit_config'),
        'config',
        'kinematics.yaml'
    )

    with open(kinematics_yaml_path, 'r') as file:
        kinematics_data = yaml.safe_load(file)

    # Wrap kinematics under robot_description_kinematics namespace
    kinematics_config = {'robot_description_kinematics': kinematics_data}

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            {'use_sim_time': use_sim_time},
            kinematics_config,
        ]
    )

    delay_rviz = TimerAction(
        period=10.0,
        actions=[rviz_node]
    )

    nodes_to_start = [
        gazebo,
        gz_bridge,
        robot_state_publisher_node,
        delay_spawn_after_rsp,
        delay_joint_state_broadcaster,
        delay_robot_controller,
        delay_moveit,
        delay_rviz,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)