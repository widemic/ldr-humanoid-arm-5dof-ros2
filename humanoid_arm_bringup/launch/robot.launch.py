#!/usr/bin/env python3

"""
Robot launch file - Mock hardware mode with controllers and RViz.

Launches the humanoid arm with fake hardware for development and testing.
No physics simulation, perfect position tracking.

Usage:
    ros2 launch humanoid_arm_bringup robot.launch.py
    ros2 launch humanoid_arm_bringup robot.launch.py use_rviz:=false
    ros2 launch humanoid_arm_bringup robot.launch.py initial_joint_controller:=arm_position_controller
"""

import sys
from pathlib import Path

# Add modules directory to Python path
sys.path.insert(0, str(Path(__file__).parent))

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

# Import our custom modules
from modules import robot_description, controllers, sensors, rviz


def generate_launch_description():
    # ========== Launch Arguments ==========
    declared_arguments = [
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Use mock hardware (true) or real hardware (false)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for visualization'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value='view_robot.rviz',
            description='RViz config file name (in humanoid_arm_bringup/config/)'
        ),
        DeclareLaunchArgument(
            'robot_controller',
            default_value='joint_trajectory_controller',
            description='Controller type for URDF configuration'
        ),
        DeclareLaunchArgument(
            'initial_joint_controller',
            default_value='joint_trajectory_controller',
            description='Primary controller to spawn (joint_trajectory_controller, '
                       'arm_position_controller, or arm_velocity_controller)'
        ),
    ]

    # ========== Configuration ==========
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_rviz_arg = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    robot_controller = LaunchConfiguration('robot_controller')
    initial_joint_controller = LaunchConfiguration('initial_joint_controller')

    # ========== Robot Description ==========
    # Load URDF with fake hardware
    robot_desc = robot_description.load_urdf(
        use_fake_hardware=True,  # Always true for this launch file
        robot_controller=robot_controller
    )

    # Robot state publisher
    robot_state_pub = robot_description.get_robot_state_publisher(
        robot_description=robot_desc,
        use_sim_time=False
    )

    # ros2_control node (for mock hardware)
    control_node = robot_description.get_control_node(
        robot_description=robot_desc,
        use_fake_hardware=True,
        robot_controller=robot_controller
    )

    # ========== Controllers ==========
    # Get joint_state_broadcaster + primary controller with proper sequencing
    controller_nodes = controllers.get_standard_controllers(
        primary_controller=initial_joint_controller,
        use_sim_time=False,
        sequence=True  # Use event handlers for proper startup order
    )

    # ========== Sensors ==========
    # Get all sensor broadcasters, sequenced after joint_state_broadcaster
    sensor_nodes = sensors.get_all_sensors(
        use_sim_time=False,
        after_node=controller_nodes[0]  # joint_state_broadcaster is first
    )

    # ========== RViz ==========
    rviz_node = rviz.get_rviz(
        config_file=rviz_config_file,
        use_sim_time=False
    )

    # Make RViz conditional
    rviz_conditional = rviz_node
    rviz_conditional.condition = IfCondition(use_rviz_arg)

    # Sequence RViz after controllers
    rviz_sequenced = controllers.sequence_after(
        controller_nodes[0],  # joint_state_broadcaster
        rviz_conditional
    )

    # ========== Compose Launch Description ==========
    nodes_to_start = [
        control_node,
        robot_state_pub,
    ]
    nodes_to_start.extend(controller_nodes)
    nodes_to_start.append(sensor_nodes)
    nodes_to_start.append(rviz_sequenced)

    return LaunchDescription(declared_arguments + nodes_to_start)
