"""
Gazebo module - launches Gazebo Harmonic simulation.

Provides reusable functions for starting Gazebo server, spawning robots,
and bridging ROS2 topics.
"""

import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def get_gazebo_server(world_file='empty_world.sdf',
                      gui=True,
                      verbose: int = 4,
                      run: bool = True) -> IncludeLaunchDescription:
    """
    Launch Gazebo Harmonic (gz-sim) server.

    Args:
        world_file: Name of .sdf world file OR LaunchConfiguration
        gui: Start Gazebo GUI OR LaunchConfiguration
        verbose: Verbosity level (0-4)
        run: Start simulation immediately (True) or paused (False)

    Returns:
        IncludeLaunchDescription: Gazebo launch
    """
    # Handle both string and LaunchConfiguration for world_file
    if isinstance(world_file, str):
        world_path = PathJoinSubstitution([
            FindPackageShare('humanoid_arm_bringup'),
            'worlds',
            world_file
        ])
    else:
        # It's a LaunchConfiguration
        world_path = PathJoinSubstitution([
            FindPackageShare('humanoid_arm_bringup'),
            'worlds',
            world_file
        ])

    # Build gz_args as substitution
    gz_args_list = []
    if run:
        gz_args_list.append('-r')  # Run immediately
    gz_args_list.append(f'-v {verbose}')  # Verbosity

    # For gui, we'll pass it through launch_arguments since it might be LaunchConfiguration
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [' '.join(gz_args_list), ' ', world_path],
            'on_exit_shutdown': 'true'
        }.items()
    )


def spawn_robot(robot_name: str = 'humanoid_arm_5dof',
                x_position: float = 0.0,
                y_position: float = 0.0,
                z_position: float = 0.5,
                yaw_rotation: float = 0.0,
                use_sim_time: bool = True) -> Node:
    """
    Spawn robot entity in Gazebo from /robot_description topic.

    Args:
        robot_name: Name of robot entity in Gazebo
        x_position: X position (meters)
        y_position: Y position (meters)
        z_position: Height to spawn robot (meters above ground)
        yaw_rotation: Yaw rotation in radians (rotation around Z axis)
        use_sim_time: Use simulation time

    Returns:
        Node: Gazebo spawn entity node
    """
    return Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', '/robot_description',
            '-x', str(x_position),
            '-y', str(y_position),
            '-z', str(z_position),
            '-Y', str(yaw_rotation)  # Capital Y for yaw in radians
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )


def get_clock_bridge() -> Node:
    """
    Bridge /clock topic from Gazebo to ROS2.

    Required for use_sim_time to work.

    Returns:
        Node: ROS-Gazebo bridge for /clock
    """
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )


def get_standard_gazebo_setup(world_file: str = 'empty_world.sdf',
                               robot_name: str = 'humanoid_arm_5dof',
                               gui: bool = True,
                               robot_x: float = 0.0,
                               robot_y: float = 0.0,
                               robot_z: float = 0.5,
                               robot_yaw: float = 0.0) -> list:
    """
    Get standard Gazebo setup: server + robot spawn + clock bridge.

    Args:
        world_file: Name of .sdf world file
        robot_name: Name of robot in Gazebo
        gui: Start Gazebo GUI
        robot_x: Robot X position (meters)
        robot_y: Robot Y position (meters)
        robot_z: Robot Z position (meters)
        robot_yaw: Robot yaw rotation (radians)

    Returns:
        list: [gazebo_server, spawn_robot, clock_bridge]
    """
    return [
        get_gazebo_server(world_file, gui=gui),
        spawn_robot(robot_name, robot_x, robot_y, robot_z, robot_yaw),
        get_clock_bridge(),
    ]
