#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro


def launch_setup(context, *args, **kwargs):
    # Get parameters
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    verbose = LaunchConfiguration('verbose')
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
            'use_fake_hardware': 'false',  # Use Gazebo hardware interface
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

    # World file path
    world_file_path = PathJoinSubstitution([
        FindPackageShare('humanoid_arm_bringup'),
        'worlds',
        world_file
    ])

    # Gazebo Harmonic launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file_path],
        }.items()
    )

    # gz_ros2_control handles both control node and hardware interface
    # No separate control_node needed with Gazebo Harmonic

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
        arguments=[
            '-name', 'humanoid_arm_5dof',
            '-topic', '/robot_description',
            '-z', '0.5'  # Spawn slightly above ground
        ],
        output='screen'
    )

    # Bridge for joint states and commands
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
        arguments=[initial_joint_controller, '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Force torque sensor broadcasters
    tcp_ft_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tcp_force_torque_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    forearm_ft_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forearm_force_torque_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    wrist_ft_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wrist_force_torque_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # IMU broadcaster spawner
    imu_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_imu_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Contact monitoring node (custom node for contact-based manipulation)
    contact_monitor_node = Node(
        package='humanoid_arm_control',
        executable='contact_monitor',
        name='contact_monitor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Delay rviz start after joint_state_broadcaster
    delay_spawn_after_rsp = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_publisher_node,
            on_exit=[spawn_entity],
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

    # Delay contact monitor after controllers are loaded
    delay_contact_monitor = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[contact_monitor_node],
        )
    )

    nodes_to_start = [
        gazebo,
        gz_bridge,
        robot_state_publisher_node,
        delay_spawn_after_rsp,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_ft_broadcasters,
        delay_contact_monitor,
    ]

    return nodes_to_start


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

    declared_arguments.append(
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Start Gazebo in headless mode.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Start Gazebo in verbose mode.',
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