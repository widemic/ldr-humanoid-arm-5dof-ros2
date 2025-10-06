"""
RViz module - launches RViz2 visualization.

Provides reusable functions for starting RViz with different configurations.
"""

import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def get_config_path(config_file: str,
                    package: str = 'humanoid_arm_bringup') -> str:
    """
    Get absolute path to RViz config file.

    Args:
        config_file: Name of .rviz file
        package: Package containing config (default: humanoid_arm_bringup)

    Returns:
        str: Absolute path to config file
    """
    return os.path.join(
        get_package_share_directory(package),
        'config',
        config_file
    )


def get_rviz(config_file: str = 'view_robot.rviz',
             package: str = 'humanoid_arm_bringup',
             use_sim_time: bool = False,
             additional_parameters: list = None) -> Node:
    """
    Launch RViz2 with specified configuration.

    Args:
        config_file: Name of .rviz config file
        package: Package containing config file
        use_sim_time: Use simulation time
        additional_parameters: Additional parameters dict (optional)

    Returns:
        Node: RViz2 node
    """
    config_path = get_config_path(config_file, package)

    parameters = [{'use_sim_time': use_sim_time}]
    if additional_parameters:
        parameters.extend(additional_parameters)

    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', config_path],
        parameters=parameters
    )


def get_rviz_moveit(robot_description: dict = None,
                    robot_description_semantic: dict = None,
                    kinematics_config: dict = None,
                    use_sim_time: bool = True) -> Node:
    """
    Launch RViz2 with MoveIt configuration.

    Args:
        robot_description: Robot URDF dict
        robot_description_semantic: SRDF dict
        kinematics_config: Kinematics solver config dict
        use_sim_time: Use simulation time

    Returns:
        Node: RViz2 node with MoveIt config
    """
    config_path = get_config_path('moveit.rviz', 'humanoid_arm_moveit_config')

    parameters = [{'use_sim_time': use_sim_time}]

    if robot_description:
        parameters.append(robot_description)
    if robot_description_semantic:
        parameters.append(robot_description_semantic)
    if kinematics_config:
        parameters.append(kinematics_config)

    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', config_path],
        parameters=parameters
    )


def get_basic_rviz(use_sim_time: bool = False) -> Node:
    """
    Launch RViz2 with basic robot view config.

    Args:
        use_sim_time: Use simulation time

    Returns:
        Node: RViz2 node with basic config
    """
    return get_rviz('view_robot.rviz', use_sim_time=use_sim_time)
