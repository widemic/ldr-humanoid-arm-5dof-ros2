"""
Robot description module - loads and publishes robot model.

Provides reusable functions for loading URDF and spawning robot_state_publisher.
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def load_urdf(use_fake_hardware: bool = True,
              robot_controller: str = 'joint_trajectory_controller') -> dict:
    """
    Load and process robot URDF via xacro.

    Args:
        use_fake_hardware: Use mock hardware (True) or real/Gazebo hardware (False)
        robot_controller: Controller type to configure in URDF

    Returns:
        dict: {'robot_description': '<urdf_xml>'}
    """
    urdf_path = os.path.join(
        get_package_share_directory('humanoid_arm_description'),
        'urdf',
        'humanoid_arm_5dof_ros2_control.urdf.xacro'
    )

    robot_description_content = xacro.process_file(
        urdf_path,
        mappings={
            'use_fake_hardware': str(use_fake_hardware).lower(),
            'robot_controller': robot_controller
        }
    )

    return {'robot_description': robot_description_content.toxml()}


def get_robot_state_publisher(robot_description: dict = None,
                               use_sim_time: bool = False,
                               use_fake_hardware: bool = True,
                               robot_controller: str = 'joint_trajectory_controller') -> Node:
    """
    Create robot_state_publisher node.

    Args:
        robot_description: Pre-loaded robot description dict (optional)
        use_sim_time: Use simulation time from /clock topic
        use_fake_hardware: Load URDF with fake hardware if robot_description not provided
        robot_controller: Controller type if loading URDF

    Returns:
        Node: robot_state_publisher node
    """
    if robot_description is None:
        robot_description = load_urdf(use_fake_hardware, robot_controller)

    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )


def get_control_node(robot_description: dict = None,
                     controllers_config: str = None,
                     use_fake_hardware: bool = True,
                     robot_controller: str = 'joint_trajectory_controller') -> Node:
    """
    Create ros2_control_node (for mock hardware only, not Gazebo).

    Args:
        robot_description: Pre-loaded robot description dict (optional)
        controllers_config: Path to controllers.yaml (optional, uses default)
        use_fake_hardware: Load URDF with fake hardware if robot_description not provided
        robot_controller: Controller type if loading URDF

    Returns:
        Node: ros2_control_node
    """
    if robot_description is None:
        robot_description = load_urdf(use_fake_hardware, robot_controller)

    if controllers_config is None:
        controllers_config = os.path.join(
            get_package_share_directory('humanoid_arm_control'),
            'config',
            'controllers.yaml'
        )

    return Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_config],
        output='both',
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
    )
