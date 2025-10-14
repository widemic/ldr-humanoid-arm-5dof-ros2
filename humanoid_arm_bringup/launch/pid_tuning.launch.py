#!/usr/bin/env python3

"""
PID Tuning launch file - Gazebo + ros2_control (no MoveIt).

Isolated single-joint PID tuning with:
- Gazebo physics simulation
- Individual joint controllers (one active at a time)
- Automatic controller activation based on 'joint' argument
- No MoveIt interference

Usage:
    # Tune joint 0 (base_rotation)
    ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint:=0

    # Tune joint 1 (shoulder_pitch)
    ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint:=1

    # Additional options:
    ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint:=0 gazebo_gui:=false
    ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint:=0 world_file:=empty_world.sdf

After launch, in separate terminals:
    ros2 run humanoid_arm_control pid_tuner_gui.py
    python3 humanoid_arm_control/scripts/test_trajectory.py --joint 0 --type step --amplitude 0.5 --reset
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
            default_value='false',
            description='Launch rqt_joint_trajectory_controller GUI (use pid_tuner_gui.py instead)'
        ),
        DeclareLaunchArgument(
            'joint',
            default_value='0',
            description='Joint to tune (0=base_rotation, 1=shoulder_pitch, 2=elbow_pitch, 3=wrist_pitch, 4=wrist_roll)'
        ),
    ]

    # ========== Configuration ==========
    world_file = LaunchConfiguration('world_file')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    launch_rqt = LaunchConfiguration('launch_rqt')
    use_sim_time = True

    # Joint controller mapping
    joint_controllers = [
        'base_rotation_joint_position_controller',
        'shoulder_pitch_joint_position_controller',
        'elbow_pitch_joint_position_controller',
        'wrist_pitch_joint_position_controller',
        'wrist_roll_joint_position_controller',
    ]

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
    from launch.actions import OpaqueFunction

    joint_state_broadcaster_spawner = controllers.get_joint_state_broadcaster(use_sim_time)

    # Individual joint controllers - will be spawned based on 'joint' argument
    def spawn_joint_controllers(context):
        """Spawn controllers based on joint argument"""
        joint_idx = int(context.launch_configurations['joint'])
        nodes = []

        for i, controller_name in enumerate(joint_controllers):
            # Only the selected joint is active, others inactive
            is_inactive = (i != joint_idx)
            nodes.append(
                controllers.spawn_controller(controller_name, use_sim_time, inactive=is_inactive)
            )

        return nodes

    spawn_controllers_action = OpaqueFunction(function=spawn_joint_controllers)

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
            on_exit=[joint_state_broadcaster_spawner, spawn_controllers_action]
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
