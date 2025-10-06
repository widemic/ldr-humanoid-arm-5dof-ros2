"""
Controllers module - spawns and manages ros2_control controllers.

Provides reusable functions for spawning controller_manager controllers
with proper sequencing.
"""

from typing import List, Union
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def spawn_controller(controller_name: str,
                     use_sim_time: bool = False,
                     controller_manager: str = '/controller_manager',
                     inactive: bool = False) -> Node:
    """
    Spawn a single controller.

    Args:
        controller_name: Name of controller to spawn (must be in controllers.yaml)
        use_sim_time: Use simulation time from /clock topic
        controller_manager: Name of controller_manager node
        inactive: Start controller in inactive state

    Returns:
        Node: Controller spawner node
    """
    args = [controller_name, '--controller-manager', controller_manager]
    if inactive:
        args.append('--inactive')

    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=args,
        parameters=[{'use_sim_time': use_sim_time}],
    )


def get_joint_state_broadcaster(use_sim_time: bool = False) -> Node:
    """
    Get joint_state_broadcaster spawner.

    This should always be spawned first.

    Args:
        use_sim_time: Use simulation time

    Returns:
        Node: joint_state_broadcaster spawner
    """
    return spawn_controller('joint_state_broadcaster', use_sim_time)


def get_trajectory_controller(use_sim_time: bool = False,
                               inactive: bool = False) -> Node:
    """
    Get joint_trajectory_controller spawner.

    Args:
        use_sim_time: Use simulation time
        inactive: Start in inactive state

    Returns:
        Node: joint_trajectory_controller spawner
    """
    return spawn_controller('joint_trajectory_controller', use_sim_time, inactive=inactive)


def get_position_controller(use_sim_time: bool = False,
                            inactive: bool = False) -> Node:
    """
    Get arm_position_controller spawner.

    Args:
        use_sim_time: Use simulation time
        inactive: Start in inactive state

    Returns:
        Node: arm_position_controller spawner
    """
    return spawn_controller('arm_position_controller', use_sim_time, inactive=inactive)


def get_velocity_controller(use_sim_time: bool = False,
                            inactive: bool = False) -> Node:
    """
    Get arm_velocity_controller spawner.

    Args:
        use_sim_time: Use simulation time
        inactive: Start in inactive state

    Returns:
        Node: arm_velocity_controller spawner
    """
    return spawn_controller('arm_velocity_controller', use_sim_time, inactive=inactive)


def sequence_after(trigger_node: Node,
                   nodes_to_start: Union[Node, List[Node]]) -> RegisterEventHandler:
    """
    Start nodes after trigger_node exits.

    Args:
        trigger_node: Node to wait for
        nodes_to_start: Node or list of nodes to start after trigger exits

    Returns:
        RegisterEventHandler: Event handler for sequencing
    """
    if not isinstance(nodes_to_start, list):
        nodes_to_start = [nodes_to_start]

    return RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=trigger_node,
            on_exit=nodes_to_start,
        )
    )


def get_standard_controllers(primary_controller: str = 'joint_trajectory_controller',
                              use_sim_time: bool = False,
                              sequence: bool = True) -> List[Union[Node, RegisterEventHandler]]:
    """
    Get standard controller setup with proper sequencing.

    Spawns:
    1. joint_state_broadcaster (first)
    2. Primary controller (after joint_state_broadcaster)

    Args:
        primary_controller: 'joint_trajectory_controller', 'arm_position_controller',
                           or 'arm_velocity_controller'
        use_sim_time: Use simulation time
        sequence: Use event handlers for proper sequencing (recommended)

    Returns:
        List: Controller spawner nodes and event handlers
    """
    joint_state_bc = get_joint_state_broadcaster(use_sim_time)

    # Get primary controller
    controller_map = {
        'joint_trajectory_controller': get_trajectory_controller,
        'arm_position_controller': get_position_controller,
        'arm_velocity_controller': get_velocity_controller,
    }

    if primary_controller not in controller_map:
        raise ValueError(f"Unknown controller: {primary_controller}. "
                        f"Must be one of {list(controller_map.keys())}")

    primary = controller_map[primary_controller](use_sim_time)

    if sequence:
        return [
            joint_state_bc,
            sequence_after(joint_state_bc, primary)
        ]
    else:
        return [joint_state_bc, primary]


def get_all_controllers(use_sim_time: bool = False) -> List[Union[Node, RegisterEventHandler]]:
    """
    Get all controllers (for development/testing).

    Spawns joint_state_broadcaster + all motion controllers (inactive).

    Args:
        use_sim_time: Use simulation time

    Returns:
        List: All controller spawners with sequencing
    """
    joint_state_bc = get_joint_state_broadcaster(use_sim_time)

    # Load all controllers but keep them inactive (can switch later)
    trajectory = get_trajectory_controller(use_sim_time, inactive=False)  # Active by default
    position = get_position_controller(use_sim_time, inactive=True)
    velocity = get_velocity_controller(use_sim_time, inactive=True)

    return [
        joint_state_bc,
        sequence_after(joint_state_bc, [trajectory, position, velocity])
    ]
