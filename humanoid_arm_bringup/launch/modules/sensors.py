"""
Sensors module - spawns sensor broadcasters.

Provides reusable functions for spawning force/torque sensors and IMU broadcasters.
"""

from typing import List, Union
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from .controllers import spawn_controller, sequence_after


def get_tcp_ft_broadcaster(use_sim_time: bool = False) -> Node:
    """
    Get TCP (tool center point) force/torque sensor broadcaster.

    Args:
        use_sim_time: Use simulation time

    Returns:
        Node: tcp_force_torque_broadcaster spawner
    """
    return spawn_controller('tcp_force_torque_broadcaster', use_sim_time)


def get_forearm_ft_broadcaster(use_sim_time: bool = False) -> Node:
    """
    Get forearm force/torque sensor broadcaster.

    Args:
        use_sim_time: Use simulation time

    Returns:
        Node: forearm_force_torque_broadcaster spawner
    """
    return spawn_controller('forearm_force_torque_broadcaster', use_sim_time)


def get_wrist_ft_broadcaster(use_sim_time: bool = False) -> Node:
    """
    Get wrist force/torque sensor broadcaster.

    Args:
        use_sim_time: Use simulation time

    Returns:
        Node: wrist_force_torque_broadcaster spawner
    """
    return spawn_controller('wrist_force_torque_broadcaster', use_sim_time)


def get_imu_broadcaster(use_sim_time: bool = False) -> Node:
    """
    Get IMU sensor broadcaster.

    Args:
        use_sim_time: Use simulation time

    Returns:
        Node: arm_imu_broadcaster spawner
    """
    return spawn_controller('arm_imu_broadcaster', use_sim_time)


def get_all_ft_broadcasters(use_sim_time: bool = False) -> List[Node]:
    """
    Get all force/torque sensor broadcasters.

    Args:
        use_sim_time: Use simulation time

    Returns:
        List[Node]: All F/T sensor broadcaster spawners
    """
    return [
        get_tcp_ft_broadcaster(use_sim_time),
        get_forearm_ft_broadcaster(use_sim_time),
        get_wrist_ft_broadcaster(use_sim_time),
    ]


def get_all_sensors(use_sim_time: bool = False,
                    after_node: Node = None) -> Union[List[Node], RegisterEventHandler]:
    """
    Get all sensor broadcasters (F/T sensors + IMU).

    Args:
        use_sim_time: Use simulation time
        after_node: Start sensors after this node exits (optional)

    Returns:
        List[Node] or RegisterEventHandler: Sensor spawners, sequenced if after_node provided
    """
    sensors = [
        get_tcp_ft_broadcaster(use_sim_time),
        get_forearm_ft_broadcaster(use_sim_time),
        get_wrist_ft_broadcaster(use_sim_time),
        get_imu_broadcaster(use_sim_time),
    ]

    if after_node is not None:
        return sequence_after(after_node, sensors)
    else:
        return sensors
