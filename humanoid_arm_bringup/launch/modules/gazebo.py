"""
Gazebo module - launches Gazebo Harmonic simulation.

Provides reusable functions for starting Gazebo server, spawning robots,
and bridging ROS2 topics.
"""

import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def get_world_path(world_file: str = 'empty_world.sdf') -> str:
    """
    Get absolute path to world file.

    Args:
        world_file: Name of .sdf world file

    Returns:
        str: Absolute path to world file
    """
    return os.path.join(
        get_package_share_directory('humanoid_arm_bringup'),
        'worlds',
        world_file
    )


def get_gazebo_server(world_file: str = 'empty_world.sdf',
                      gui: bool = True,
                      verbose: int = 4,
                      run: bool = True) -> IncludeLaunchDescription:
    """
    Launch Gazebo Harmonic (gz-sim) server.

    Args:
        world_file: Name of .sdf world file (in humanoid_arm_bringup/worlds/)
        gui: Start Gazebo GUI
        verbose: Verbosity level (0-4)
        run: Start simulation immediately (True) or paused (False)

    Returns:
        IncludeLaunchDescription: Gazebo launch
    """
    world_path = get_world_path(world_file)

    # Build gz_args
    gz_args = []
    if run:
        gz_args.append('-r')  # Run immediately
    gz_args.append(f'-v {verbose}')  # Verbosity
    if not gui:
        gz_args.append('-s')  # Server only (headless)
    gz_args.append(world_path)

    gz_args_str = ' '.join(gz_args)

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': gz_args_str,
            'on_exit_shutdown': 'true'
        }.items()
    )


def spawn_robot(robot_name: str = 'humanoid_arm_5dof',
                z_position: float = 0.5,
                use_sim_time: bool = True) -> Node:
    """
    Spawn robot entity in Gazebo from /robot_description topic.

    Args:
        robot_name: Name of robot entity in Gazebo
        z_position: Height to spawn robot (meters above ground)
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
            '-z', str(z_position)
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
                               gui: bool = True) -> list:
    """
    Get standard Gazebo setup: server + robot spawn + clock bridge.

    Args:
        world_file: Name of .sdf world file
        robot_name: Name of robot in Gazebo
        gui: Start Gazebo GUI

    Returns:
        list: [gazebo_server, spawn_robot, clock_bridge]
    """
    return [
        get_gazebo_server(world_file, gui=gui),
        spawn_robot(robot_name),
        get_clock_bridge(),
    ]
