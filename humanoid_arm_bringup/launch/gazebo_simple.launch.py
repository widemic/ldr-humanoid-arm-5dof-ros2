#!/usr/bin/env python3

"""
Simple Gazebo launch with ros2_control (effort interface) - Modular style.

Minimal setup for physics simulation with effort control.
Perfect for DualSense teleoperation testing.

Usage:
    ros2 launch humanoid_arm_bringup gazebo_simple.launch.py
    ros2 launch humanoid_arm_bringup gazebo_simple.launch.py world_file:=contact_manipulation_arena.sdf
    ros2 launch humanoid_arm_bringup gazebo_simple.launch.py gui:=false
"""

import sys
from pathlib import Path

# Add modules directory to Python path
sys.path.insert(0, str(Path(__file__).parent))

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

# Import our custom modules
from modules import robot_description, controllers, gazebo


def generate_launch_description():
    # ========== Launch Arguments ==========
    declared_arguments = [
        DeclareLaunchArgument(
            'world_file',
            default_value='empty_world.sdf',
            description='Gazebo world file (in humanoid_arm_bringup/worlds/)'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch Gazebo GUI (false for headless)'
        ),
    ]

    # ========== Configuration ==========
    world_file = LaunchConfiguration('world_file')
    gui = LaunchConfiguration('gui')

    # ========== Gazebo ==========
    gazebo_nodes = gazebo.get_standard_gazebo_setup(
        world_file=world_file,
        robot_name='humanoid_arm_5dof',
        gui=gui
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

    robot_state_pub = robot_description.get_robot_state_publisher(
        robot_description={'robot_description': robot_description_content},
        use_sim_time=True
    )

    # ========== Spawn Robot (Delayed) ==========
    spawn_robot_delayed = TimerAction(
        period=3.0,
        actions=[gazebo_nodes[1]]  # spawn_robot is second in list
    )

    # ========== Controllers (Delayed) ==========
    controller_nodes = controllers.get_standard_controllers(
        primary_controller='joint_trajectory_controller',
        use_sim_time=True,
        sequence=True
    )

    controllers_delayed = TimerAction(
        period=5.0,
        actions=controller_nodes
    )

    # ========== Compose Launch Description ==========
    nodes_to_start = [
        gazebo_nodes[0],  # Gazebo server
        gazebo_nodes[2],  # Clock bridge
        robot_state_pub,
        spawn_robot_delayed,
        controllers_delayed,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
