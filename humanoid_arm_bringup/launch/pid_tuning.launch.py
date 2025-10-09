#!/usr/bin/env python3

"""
PID Tuning launch file - Gazebo + ros2_control (no MoveIt).

Simplified launch for PID tuning with:
- Gazebo physics simulation
- ros2_control with joint_trajectory_controller
- rqt_joint_trajectory_controller GUI
- Event-based sequencing for reliability

Usage:
    ros2 launch humanoid_arm_bringup pid_tuning.launch.py
    ros2 launch humanoid_arm_bringup pid_tuning.launch.py world_file:=empty_world.sdf
    ros2 launch humanoid_arm_bringup pid_tuning.launch.py gazebo_gui:=false
    ros2 launch humanoid_arm_bringup pid_tuning.launch.py launch_rqt:=false
"""

import sys
from pathlib import Path

# Add modules directory to Python path
sys.path.insert(0, str(Path(__file__).parent))

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

# Import our custom modules
from modules import gazebo, controllers


def generate_launch_description():
    # ========== Launch Arguments ==========
    declared_arguments = [
        DeclareLaunchArgument(
            'world_file',
            default_value='empty_world.sdf',
            description='Gazebo world file (in humanoid_arm_bringup/worlds/)'
        ),
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='true',
            description='Launch Gazebo GUI'
        ),
        DeclareLaunchArgument(
            'launch_rqt',
            default_value='true',
            description='Launch rqt_joint_trajectory_controller GUI'
        ),
    ]

    # ========== Configuration ==========
    world_file = LaunchConfiguration('world_file')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    launch_rqt = LaunchConfiguration('launch_rqt')
    use_sim_time = True

    # ========== Gazebo ==========
    gazebo_nodes = gazebo.get_standard_gazebo_setup(
        world_file=world_file,
        robot_name='humanoid_arm_5dof',
        gui=gazebo_gui
    )
    gazebo_server = gazebo_nodes[0]
    spawn_robot = gazebo_nodes[1]
    clock_bridge = gazebo_nodes[2]

    # Bridge for joint force-torque sensors (real joint torques)
    ft_bridge = gazebo.get_joint_ft_bridges(
        robot_name='humanoid_arm_5dof',
        world_name='humanoid_arm_empty_world'
    )

    # ========== Robot Description ==========
    urdf_path = PathJoinSubstitution([
        FindPackageShare('humanoid_arm_description'),
        'urdf',
        'humanoid_arm_5dof_ros2_control.urdf.xacro'
    ])

    robot_description_content = ParameterValue(
        Command([
            'xacro ', urdf_path,
            ' use_fake_hardware:=false',
            ' robot_controller:=joint_trajectory_controller'
        ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': use_sim_time}
        ]
    )

    # ========== Controllers ==========
    # Load individual joint controllers for isolated PID tuning
    from launch_ros.actions import Node as LaunchNode

    joint_state_broadcaster_spawner = controllers.get_joint_state_broadcaster(use_sim_time)

    # Individual joint controllers (all inactive initially - GUI will enable them)
    individual_controllers = [
        controllers.spawn_controller('base_rotation_joint_position_controller', use_sim_time, inactive=True),
        controllers.spawn_controller('shoulder_pitch_joint_position_controller', use_sim_time, inactive=True),
        controllers.spawn_controller('elbow_pitch_joint_position_controller', use_sim_time, inactive=True),
        controllers.spawn_controller('wrist_pitch_joint_position_controller', use_sim_time, inactive=True),
        controllers.spawn_controller('wrist_roll_joint_position_controller', use_sim_time, inactive=True),
    ]

    # ========== Event-based Sequencing ==========
    # Spawn robot after Gazebo starts (small delay for Gazebo initialization)
    spawn_robot_delayed = TimerAction(
        period=2.0,
        actions=[spawn_robot]
    )

    # Start controllers after robot is spawned
    start_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner] + individual_controllers
        )
    )

    # ========== RQT Joint Trajectory Controller ==========
    # Launch after controllers are ready
    # Note: RQT manages its own node name internally, don't specify 'name' parameter
    rqt_node = Node(
        package='rqt_joint_trajectory_controller',
        executable='rqt_joint_trajectory_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rqt)
    )

    # Start RQT after joint_state_broadcaster is ready
    start_rqt = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rqt_node]
        )
    )

    # ========== Compose Launch Description ==========
    nodes_to_start = [
        gazebo_server,
        clock_bridge,
        ft_bridge,
        robot_state_publisher,
        spawn_robot_delayed,
        start_controllers,
        start_rqt,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
