"""
RQt module - launches RQt tools for monitoring and debugging.

Provides reusable functions for starting various RQt plugins.
"""

from launch_ros.actions import Node


def get_rqt(use_sim_time: bool = False) -> Node:
    """
    Launch RQt main window (user can load plugins manually).

    Args:
        use_sim_time: Use simulation time

    Returns:
        Node: RQt main window
    """
    return Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rqt_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )


def get_rqt_joint_trajectory_controller(use_sim_time: bool = False,
                                         controller_name: str = 'joint_trajectory_controller') -> Node:
    """
    Launch RQt Joint Trajectory Controller plugin.

    Provides GUI for sending trajectory commands to the joint trajectory controller.
    Shows current joint states, allows setting target positions, and displays tracking errors.

    Args:
        use_sim_time: Use simulation time
        controller_name: Name of the joint trajectory controller

    Returns:
        Node: RQt Joint Trajectory Controller plugin
    """
    return Node(
        package='rqt_joint_trajectory_controller',
        executable='rqt_joint_trajectory_controller',
        name='rqt_joint_trajectory_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )


def get_rqt_plot(use_sim_time: bool = False,
                 topics: list = None) -> Node:
    """
    Launch RQt Plot for real-time data visualization.

    Args:
        use_sim_time: Use simulation time
        topics: List of topics to plot (optional, can add in GUI)

    Returns:
        Node: RQt Plot plugin
    """
    args = []
    if topics:
        args.extend(topics)

    return Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='rqt_plot',
        output='screen',
        arguments=args,
        parameters=[{'use_sim_time': use_sim_time}]
    )


def get_rqt_console(use_sim_time: bool = False) -> Node:
    """
    Launch RQt Console for viewing ROS logs.

    Args:
        use_sim_time: Use simulation time

    Returns:
        Node: RQt Console plugin
    """
    return Node(
        package='rqt_console',
        executable='rqt_console',
        name='rqt_console',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )


def get_rqt_graph(use_sim_time: bool = False) -> Node:
    """
    Launch RQt Graph for visualizing node/topic connections.

    Args:
        use_sim_time: Use simulation time

    Returns:
        Node: RQt Graph plugin
    """
    return Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )


def get_rqt_reconfigure(use_sim_time: bool = False) -> Node:
    """
    Launch RQt Dynamic Reconfigure for adjusting parameters at runtime.

    Args:
        use_sim_time: Use simulation time

    Returns:
        Node: RQt Dynamic Reconfigure plugin
    """
    return Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )


def get_rqt_robot_monitor(use_sim_time: bool = False) -> Node:
    """
    Launch RQt Robot Monitor for viewing diagnostic messages.

    Args:
        use_sim_time: Use simulation time

    Returns:
        Node: RQt Robot Monitor plugin
    """
    return Node(
        package='rqt_robot_monitor',
        executable='rqt_robot_monitor',
        name='rqt_robot_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )


def get_rqt_controller_manager(use_sim_time: bool = False) -> Node:
    """
    Launch RQt Controller Manager for loading/unloading/switching controllers.

    Args:
        use_sim_time: Use simulation time

    Returns:
        Node: RQt Controller Manager plugin
    """
    return Node(
        package='rqt_controller_manager',
        executable='rqt_controller_manager',
        name='rqt_controller_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )


def get_tuning_tools(use_sim_time: bool = True) -> list:
    """
    Get recommended RQt tools for PID tuning and trajectory testing.

    Returns:
        list: [joint_trajectory_controller, plot]
    """
    return [
        get_rqt_joint_trajectory_controller(use_sim_time),
        get_rqt_plot(
            use_sim_time,
            topics=[
                '/joint_states/position[0]',
                '/joint_trajectory_controller/controller_state/desired/positions[0]',
                '/joint_trajectory_controller/controller_state/error/positions[0]'
            ]
        ),
    ]


def get_monitoring_tools(use_sim_time: bool = True) -> list:
    """
    Get recommended RQt tools for system monitoring.

    Returns:
        list: [console, graph, robot_monitor]
    """
    return [
        get_rqt_console(use_sim_time),
        get_rqt_graph(use_sim_time),
        get_rqt_robot_monitor(use_sim_time),
    ]
