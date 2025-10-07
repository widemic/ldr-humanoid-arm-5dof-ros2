#!/usr/bin/env python3
import os
import xacro
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory("humanoid_arm_bringup")
    description_dir = get_package_share_directory("humanoid_arm_description")

    world_path = os.path.join(bringup_dir, "worlds", "empty_world.sdf")

    # URDF
    urdf_path = os.path.join(description_dir, "urdf", "humanoid_arm_5dof_ros2_control.urdf.xacro")
    robot_description_content = xacro.process_file(
        urdf_path,
        mappings={"use_fake_hardware": "false"}
    )
    robot_description = {"robot_description": robot_description_content.toxml()}

    world_arg = DeclareLaunchArgument(
        "world", default_value=world_path, description="Path to Gazebo world file"
    )

    return LaunchDescription([
        world_arg,
        
        # 1️⃣ Gazebo
        ExecuteProcess(
            cmd=["gz", "sim", "-r", LaunchConfiguration("world"), "--force-version", "8"],
            output="screen",
            additional_env={
                'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib/'
            }
        ),

        # 2️⃣ 🔥 IMPORTANT: Bridge pentru clock între Gazebo și ROS2
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
            ],
            output="screen",
            parameters=[{"use_sim_time": True}]
        ),

        # 3️⃣ Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher", 
            parameters=[robot_description, {"use_sim_time": True}],
            output="screen",
        ),

        # 4️⃣ Spawn robot
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    arguments=[
                        "-name", "humanoid_arm",
                        "-topic", "/robot_description",
                        "-z", "0.5",
                    ],
                    output="screen",
                    parameters=[{"use_sim_time": True}]
                )
            ]
        ),

        # 5️⃣ Controller spawners 
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster"],
                    output="screen",
                    parameters=[{"use_sim_time": True}]
                ),
                Node(
                    package="controller_manager", 
                    executable="spawner",
                    arguments=["joint_trajectory_controller"],
                    output="screen",
                    parameters=[{"use_sim_time": True}]
                )
            ]
        ),
    ])