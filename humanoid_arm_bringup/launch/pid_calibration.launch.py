#!/usr/bin/env python3
"""
Simple PID Calibration Launch

Launches:
- Gazebo with physics
- Single joint controller (choose which joint to tune)
- PlotJuggler for visualization

Usage:
    # Tune base rotation (default)
    ros2 launch humanoid_arm_bringup pid_calibration.launch.py

    # Tune specific joint
    ros2 launch humanoid_arm_bringup pid_calibration.launch.py joint:=elbow_pitch_joint

    # Without PlotJuggler
    ros2 launch humanoid_arm_bringup pid_calibration.launch.py use_plotjuggler:=false

Available joints:
    - base_rotation_joint
    - shoulder_pitch_joint
    - elbow_pitch_joint
    - wrist_pitch_joint
    - wrist_roll_joint

After launch:
    1. Use rqt_joint_trajectory_controller GUI to send commands
    2. Watch oscillations in PlotJuggler
    3. Tune PID: ros2 run humanoid_arm_control pid_tuner_gui.py
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
    # Arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'joint',
            default_value='base_rotation_joint',
            description='Joint to calibrate PID for'
        ),
        DeclareLaunchArgument(
            'use_plotjuggler',
            default_value='true',
            description='Launch PlotJuggler for visualization'
        ),
    ]

    joint = LaunchConfiguration('joint')
    use_plotjuggler = LaunchConfiguration('use_plotjuggler')

    # Robot description
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

    # World file
    world_file = PathJoinSubstitution([
        FindPackageShare('humanoid_arm_bringup'),
        'worlds',
        'empty_world.sdf'
    ])

    # ====================
    # GAZEBO
    # ====================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': ['-r -v4 ', world_file]}.items()
    )

    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # ====================
    # ROBOT
    # ====================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', 'humanoid_arm_5dof', '-z', '0.5'],
        output='screen'
    )

    # ====================
    # CONTROLLERS
    # ====================
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )

    # Single joint controller (dynamic name based on joint parameter)
    from launch.actions import OpaqueFunction

    def create_controller_spawner(context):
        joint_name = joint.perform(context)
        controller_name = f"{joint_name}_position_controller"

        return [Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller_name, '--controller-manager', '/controller_manager']
        )]

    single_joint_controller = OpaqueFunction(function=create_controller_spawner)

    # ====================
    # GUI TOOLS
    # ====================

    # RQt Joint Trajectory Controller (for sending commands)
    rqt_controller = Node(
        package='rqt_joint_trajectory_controller',
        executable='rqt_joint_trajectory_controller',
        name='rqt_joint_trajectory_controller',
        output='screen'
    )

    # PlotJuggler (for visualizing oscillations)
    plotjuggler = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        output='screen',
        condition=IfCondition(use_plotjuggler)
    )

    # ====================
    # TIMING
    # ====================
    delay_spawn = TimerAction(period=3.0, actions=[spawn_robot])
    delay_joint_state = TimerAction(period=5.0, actions=[joint_state_broadcaster])
    delay_controller = TimerAction(period=7.0, actions=[single_joint_controller])
    delay_gui = TimerAction(period=9.0, actions=[rqt_controller, plotjuggler])

    return LaunchDescription(
        declared_arguments + [
            gazebo,
            clock_bridge,
            robot_state_publisher,
            delay_spawn,
            delay_joint_state,
            delay_controller,
            delay_gui,
        ]
    )
