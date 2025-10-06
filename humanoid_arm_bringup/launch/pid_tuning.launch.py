#!/usr/bin/env python3
"""
PID Tuning Launch File

Launches Gazebo simulation with isolated single-joint control for PID tuning.
Only ONE joint controller is spawned at a time to avoid multi-joint interference.

Usage:
    ros2 launch humanoid_arm_bringup pid_tuning.launch.py
    ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint_to_tune:=shoulder_pitch_joint
    ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint_to_tune:=elbow_pitch_joint use_gui:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'joint_to_tune',
            default_value='base_rotation_joint',
            description='Joint to tune: base_rotation_joint, shoulder_pitch_joint, elbow_pitch_joint, wrist_pitch_joint, wrist_roll_joint'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='empty_world.sdf',
            description='Gazebo world file (empty_world.sdf, contact_manipulation_arena.sdf, etc.)'
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Launch rqt_joint_trajectory_controller GUI for manual control'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
    ]

    # Get launch arguments
    joint_to_tune = LaunchConfiguration('joint_to_tune')
    world_file = LaunchConfiguration('world_file')
    use_gui = LaunchConfiguration('use_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Build URDF from xacro
    urdf_file = os.path.join(
        get_package_share_directory('humanoid_arm_description'),
        'urdf',
        'humanoid_arm_5dof_ros2_control.urdf.xacro'
    )
    robot_description = {
        'robot_description': xacro.process_file(
            urdf_file,
            mappings={'use_fake_hardware': 'false'}
        ).toxml()
    }

    # World file path
    world_file_path = PathJoinSubstitution([
        FindPackageShare('humanoid_arm_bringup'),
        'worlds',
        world_file
    ])

    # ============================================================================
    # NODES
    # ============================================================================

    # Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': ['-r -v4 ', world_file_path]}.items()
    )

    # Clock bridge (Gazebo time → ROS time)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'humanoid_arm_5dof',
            '-z', '0.5'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Controller configuration file path
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('humanoid_arm_control'),
        'config',
        'controllers.yaml'
    ])

    # Single joint controller (dynamically named based on joint_to_tune)
    # Note: This uses OpaqueFunction to access the launch configuration value
    from launch.actions import OpaqueFunction

    def create_controller_spawner(context):
        joint_name = joint_to_tune.perform(context)
        controller_name = f"{joint_name}_position_controller"

        return [Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                controller_name,
                '--controller-manager', '/controller_manager',
                '--ros-args',
                '--params-file', controllers_yaml
            ],
            parameters=[{'use_sim_time': use_sim_time}]
        )]

    single_joint_controller = OpaqueFunction(function=create_controller_spawner)

    # RQt Joint Trajectory Controller GUI
    rqt_gui = Node(
        package='rqt_joint_trajectory_controller',
        executable='rqt_joint_trajectory_controller',
        name='rqt_joint_trajectory_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_gui)
    )

    # ============================================================================
    # TIMING SEQUENCE
    # ============================================================================

    # t=0s: Gazebo, clock bridge, robot_state_publisher start immediately
    # t=3s: Spawn robot in Gazebo
    # t=5s: Start joint state broadcaster
    # t=7s: Start single joint controller
    # t=9s: Start GUI

    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_robot]
    )

    delayed_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster]
    )

    delayed_controller = TimerAction(
        period=7.0,
        actions=[single_joint_controller]
    )

    delayed_gui = TimerAction(
        period=9.0,
        actions=[rqt_gui]
    )

    # ============================================================================
    # LAUNCH DESCRIPTION
    # ============================================================================

    return LaunchDescription(
        declared_arguments + [
            gazebo,
            clock_bridge,
            robot_state_pub,
            delayed_spawn,
            delayed_joint_state_broadcaster,
            delayed_controller,
            delayed_gui,
        ]
    )
