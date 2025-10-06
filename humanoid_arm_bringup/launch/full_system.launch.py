#!/usr/bin/env python3

"""
Full system launch file - Gazebo + MoveIt2 + RViz.

Launches complete system: Gazebo simulation with physics, MoveIt2 motion planning,
and RViz visualization with interactive planning.

Usage:
    ros2 launch humanoid_arm_bringup full_system.launch.py
    ros2 launch humanoid_arm_bringup full_system.launch.py world_file:=contact_manipulation_arena.sdf
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

# Import our custom modules
from modules import robot_description, controllers, gazebo, rviz


def generate_launch_description():
    # ========== Launch Arguments ==========
    declared_arguments = [
        DeclareLaunchArgument(
            'world_file',
            default_value='empty_world.sdf',
            description='Gazebo world file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Gazebo GUI'
        ),
    ]

    # ========== Configuration ==========
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    # ========== Gazebo ==========
    gazebo_nodes = gazebo.get_standard_gazebo_setup(
        world_file=world_file,
        gui=gui
    )

    # ========== Robot Description ==========
    robot_desc = robot_description.load_urdf(
        use_fake_hardware=False,
        robot_controller='joint_trajectory_controller'
    )

    robot_state_pub = robot_description.get_robot_state_publisher(
        robot_description=robot_desc,
        use_sim_time=True
    )

    # ========== Load SRDF for MoveIt ==========
    srdf_path = os.path.join(
        get_package_share_directory('humanoid_arm_moveit_config'),
        'config',
        'humanoid_arm_5dof.srdf'
    )
    with open(srdf_path, 'r') as file:
        robot_description_semantic = {'robot_description_semantic': file.read()}

    # ========== Load Kinematics Config ==========
    kinematics_path = os.path.join(
        get_package_share_directory('humanoid_arm_moveit_config'),
        'config',
        'kinematics.yaml'
    )
    with open(kinematics_path, 'r') as file:
        kinematics_data = yaml.safe_load(file)
    kinematics_config = {'robot_description_kinematics': kinematics_data}

    # ========== Spawn Robot ==========
    spawn_robot_delayed = TimerAction(
        period=3.0,
        actions=[gazebo_nodes[1]]  # spawn_robot
    )

    # ========== Controllers ==========
    controller_nodes = controllers.get_standard_controllers(
        primary_controller='joint_trajectory_controller',
        use_sim_time=True,
        sequence=True
    )

    controllers_delayed = TimerAction(
        period=5.0,
        actions=controller_nodes
    )

    # ========== MoveIt2 ==========
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('humanoid_arm_moveit_config'),
                'launch',
                'move_group_sim.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    moveit_delayed = TimerAction(
        period=7.0,
        actions=[moveit_launch]
    )

    # ========== RViz with MoveIt ==========
    rviz_node = rviz.get_rviz_moveit(
        robot_description=robot_desc,
        robot_description_semantic=robot_description_semantic,
        kinematics_config=kinematics_config,
        use_sim_time=True
    )

    rviz_delayed = TimerAction(
        period=9.0,
        actions=[rviz_node]
    )

    # ========== Compose Launch Description ==========
    nodes_to_start = [
        gazebo_nodes[0],  # Gazebo server
        gazebo_nodes[2],  # Clock bridge
        robot_state_pub,
        spawn_robot_delayed,
        controllers_delayed,
        moveit_delayed,
        rviz_delayed,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
