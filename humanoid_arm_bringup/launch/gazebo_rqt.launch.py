#!/usr/bin/env python3

"""
Gazebo + RQt Joint Trajectory Controller Launch File

Perfect for PID tuning, trajectory testing, and manual joint control.
Launches Gazebo simulation with RQt Joint Trajectory Controller GUI.

Features:
- Gazebo Harmonic physics simulation
- RQt Joint Trajectory Controller for manual joint control
- Real-time trajectory tracking visualization
- Easy to add more RQt plugins for monitoring

Usage:
    # Basic launch
    ros2 launch humanoid_arm_bringup gazebo_rqt.launch.py

    # With specific world
    ros2 launch humanoid_arm_bringup gazebo_rqt.launch.py world_file:=contact_manipulation_arena.sdf

    # Headless (no Gazebo GUI, only RQt)
    ros2 launch humanoid_arm_bringup gazebo_rqt.launch.py gui:=false

    # With additional RQt plot for tracking errors
    ros2 launch humanoid_arm_bringup gazebo_rqt.launch.py enable_plot:=true

Use Cases:
    - PID gain tuning with immediate visual feedback
    - Testing individual joint movements
    - Trajectory validation before MoveIt integration
    - Contact force monitoring during manipulation
"""

import sys
import os
from pathlib import Path

# Add modules directory to Python path
launch_dir = Path(__file__).parent
sys.path.insert(0, str(launch_dir))

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

# Import our modular components
from modules import robot_description, controllers, gazebo, rqt


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
            description='Launch Gazebo GUI (false for headless mode)'
        ),
        DeclareLaunchArgument(
            'enable_plot',
            default_value='false',
            description='Launch RQt Plot for tracking error visualization'
        ),
    ]

    # ========== Configuration ==========
    world_file = LaunchConfiguration('world_file')
    gui = LaunchConfiguration('gui')
    enable_plot = LaunchConfiguration('enable_plot')

    # ========== Gazebo Simulation ==========
    # Get Gazebo server, robot spawn, and clock bridge
    gazebo_nodes = gazebo.get_standard_gazebo_setup(
        world_file=world_file,
        robot_name='humanoid_arm_5dof',
        gui=gui
    )

    # ========== Robot Description ==========
    # Load URDF for Gazebo (real physics with effort control)
    robot_desc = robot_description.load_urdf(
        use_fake_hardware=False,  # Use Gazebo hardware interface
        robot_controller='joint_trajectory_controller'
    )

    robot_state_pub = robot_description.get_robot_state_publisher(
        robot_description=robot_desc,
        use_sim_time=True
    )

    # ========== Spawn Robot in Gazebo ==========
    # Delay spawn until robot_state_publisher is ready
    spawn_robot_delayed = TimerAction(
        period=3.0,
        actions=[gazebo_nodes[1]]  # spawn_robot
    )

    # ========== Controllers ==========
    # Joint state broadcaster + joint trajectory controller
    controller_nodes = controllers.get_standard_controllers(
        primary_controller='joint_trajectory_controller',
        use_sim_time=True,
        sequence=True  # Proper startup sequencing
    )

    # Delay controllers after robot spawn
    controllers_delayed = TimerAction(
        period=5.0,
        actions=controller_nodes
    )

    # ========== RQt Joint Trajectory Controller ==========
    # Main GUI for manual joint control
    rqt_jtc = rqt.get_rqt_joint_trajectory_controller(
        use_sim_time=True,
        controller_name='joint_trajectory_controller'
    )

    # Delay RQt until controllers are loaded
    rqt_jtc_delayed = TimerAction(
        period=7.0,
        actions=[rqt_jtc]
    )

    # ========== Optional: RQt Plot for Tracking Errors ==========
    # Shows real-time plot of joint position tracking
    from launch.actions import ExecuteProcess

    # Create conditional action wrapper
    rqt_plot_delayed = TimerAction(
        period=8.0,
        actions=[
            rqt.get_rqt_plot(
                use_sim_time=True,
                topics=[
                    '/joint_states/position[0]',  # Base rotation actual
                    '/joint_trajectory_controller/controller_state/desired/positions[0]',  # Desired
                    '/joint_trajectory_controller/controller_state/error/positions[0]',  # Error
                ]
            )
        ],
        condition=IfCondition(enable_plot)
    )

    # ========== Compose Launch Description ==========
    nodes_to_start = [
        # Gazebo
        gazebo_nodes[0],  # Gazebo server
        gazebo_nodes[2],  # Clock bridge

        # Robot
        robot_state_pub,
        spawn_robot_delayed,

        # Controllers
        controllers_delayed,

        # RQt Tools
        rqt_jtc_delayed,
        rqt_plot_delayed,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
