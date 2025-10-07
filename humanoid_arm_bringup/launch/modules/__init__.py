"""
Reusable launch modules for humanoid arm bringup.

Organized by type for clarity and maintainability:

providers/
    Core system providers that manage robot functionality:
    - robot_description: URDF loading, robot_state_publisher
    - controllers: ros2_control controller spawning and sequencing
    - sensors: Sensor broadcaster management (F/T, IMU)
    - gazebo: Gazebo simulation server and robot spawning

gui/
    Graphical user interface tools:
    - rviz: RViz2 3D visualization configuration
    - rqt: RQt debugging and monitoring tools

utilities/
    Helper functions and common utilities
    (Reserved for future expansion)

Usage:
    from modules.providers import robot_description, controllers
    from modules.gui import rviz, rqt

    # Or import the whole category
    from modules import providers, gui

    # Use the modules
    robot_desc = providers.robot_description.load_urdf()
    rsp = providers.robot_description.get_robot_state_publisher(robot_desc)
    ctrl_nodes = providers.controllers.get_standard_controllers()
    rviz_node = gui.rviz.get_basic_rviz()
"""

# Import subpackages for convenience
from . import providers
from . import gui
from . import utilities

# Also expose individual modules at top level for backward compatibility
from .providers import robot_description, controllers, sensors, gazebo
from .gui import rviz, rqt

__all__ = [
    # Subpackages
    'providers',
    'gui',
    'utilities',
    # Individual modules (backward compatible)
    'robot_description',
    'controllers',
    'sensors',
    'gazebo',
    'rviz',
    'rqt',
]
