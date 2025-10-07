#!/usr/bin/env python3

"""
Gazebo simulation launch file - Physics simulation without MoveIt.

Launches Gazebo Harmonic with the humanoid arm for physics-based testing,
PID tuning, and contact manipulation experiments.

Usage:
    ros2 launch humanoid_arm_bringup gazebo.launch.py
    ros2 launch humanoid_arm_bringup gazebo.launch.py world_file:=contact_manipulation_arena.sdf
    ros2 launch humanoid_arm_bringup gazebo.launch.py gui:=false
"""

import sys
from pathlib import Path

# Add modules directory to Python path
sys.path.insert(0, str(Path(__file__).parent))

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

# Import our custom modules
from modules import robot_description, controllers, sensors, gazebo


def generate_launch_description():
    # ========== Launch Arguments ==========
    declared_arguments = [
        DeclareLaunchArgument(
            'world_file',
            default_value='empty_world.sdf',
            description='Gazebo world file (in humanoid_arm_bringup/worlds/)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time from Gazebo /clock'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch Gazebo GUI (false for headless)'
        ),
        DeclareLaunchArgument(
            'robot_controller',
            default_value='joint_trajectory_controller',
            description='Controller type for URDF configuration'
        ),
        DeclareLaunchArgument(
            'initial_joint_controller',
            default_value='joint_trajectory_controller',
            description='Primary controller to spawn'
        ),
    ]

    # ========== Configuration ==========
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    robot_controller = LaunchConfiguration('robot_controller')
    initial_joint_controller = LaunchConfiguration('initial_joint_controller')

    # ========== Gazebo ==========
    gazebo_nodes = gazebo.get_standard_gazebo_setup(
        world_file=world_file,
        robot_name='humanoid_arm_5dof',
        gui=gui
    )

    # ========== Robot Description ==========
    # Load URDF for Gazebo (use_fake_hardware=False)
    robot_desc = robot_description.load_urdf(
        use_fake_hardware=False,  # Use Gazebo hardware interface
        robot_controller=robot_controller
    )

    # Robot state publisher
    robot_state_pub = robot_description.get_robot_state_publisher(
        robot_description=robot_desc,
        use_sim_time=True
    )

    # ========== Delay Robot Spawn ==========
    # Spawn robot in Gazebo after robot_state_publisher starts
    spawn_robot_delayed = TimerAction(
        period=3.0,
        actions=[gazebo_nodes[1]]  # spawn_robot is second in list
    )

    # ========== Controllers ==========
    # Get joint_state_broadcaster + primary controller
    controller_nodes = controllers.get_standard_controllers(
        primary_controller=initial_joint_controller,
        use_sim_time=True,
        sequence=True
    )

    # Delay controllers after spawn
    controllers_delayed = TimerAction(
        period=5.0,
        actions=controller_nodes
    )

    # ========== Sensors ==========
    # Get all sensor broadcasters
    sensor_nodes = sensors.get_all_sensors(
        use_sim_time=True,
        after_node=controller_nodes[0]  # After joint_state_broadcaster
    )

    # Delay sensors
    sensors_delayed = TimerAction(
        period=7.0,
        actions=[sensor_nodes] if not isinstance(sensor_nodes, list) else sensor_nodes
    )

    # ========== Compose Launch Description ==========
    nodes_to_start = [
        gazebo_nodes[0],  # Gazebo server
        gazebo_nodes[2],  # Clock bridge
        robot_state_pub,
        spawn_robot_delayed,
        controllers_delayed,
        sensors_delayed,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
