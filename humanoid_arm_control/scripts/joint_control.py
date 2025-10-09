#!/usr/bin/env python3
"""
Joint Control GUI - Simple MVC Pattern

Loads UI from .ui file and connects to ROS2 for joint control and effort monitoring.
Clean separation: UI definition in .ui file, logic in this controller.
"""

import sys
import os

# Fix Qt/snap library conflicts from VSCode snap
for key in ['GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE', 'GIO_MODULE_DIR']:
    os.environ.pop(key, None)
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer, pyqtSlot


class JointControlNode(Node):
    """ROS2 Node for joint control and state monitoring"""

    def __init__(self):
        super().__init__('joint_control_gui')

        # Joint controller names
        self.controller_names = [
            'base_rotation_joint_position_controller',
            'shoulder_pitch_joint_position_controller',
            'elbow_pitch_joint_position_controller',
            'wrist_pitch_joint_position_controller',
            'wrist_roll_joint_position_controller',
        ]

        # Publishers for individual joint controllers (they use JointTrajectory)
        self.joint_pubs = {}
        for name in self.controller_names:
            self.joint_pubs[name] = self.create_publisher(
                JointTrajectory,
                f'/{name}/joint_trajectory',
                10
            )

        # Service client for switching controllers
        self.switch_controller_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller'
        )

        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Storage for latest joint states
        self.joint_states = {}

        self.get_logger().info('Joint Control Node ready')

    def joint_state_callback(self, msg):
        """Store latest joint state data"""
        for i, name in enumerate(msg.name):
            self.joint_states[name] = {
                'position': msg.position[i] if i < len(msg.position) else 0.0,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                'effort': msg.effort[i] if i < len(msg.effort) else 0.0,
            }

    def switch_controller(self, controller_name, activate):
        """Activate or deactivate a controller"""
        if not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Controller manager service not available')
            return False

        request = SwitchController.Request()
        if activate:
            request.activate_controllers = [controller_name]
            request.deactivate_controllers = []
        else:
            request.activate_controllers = []
            request.deactivate_controllers = [controller_name]

        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()

        future = self.switch_controller_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if future.done():
            response = future.result()
            return response.ok
        return False

    def send_joint_command(self, controller_name, joint_name, position):
        """Send trajectory command to individual joint controller"""
        traj = JointTrajectory()
        traj.joint_names = [joint_name]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=0, nanosec=500000000)

        traj.points = [point]
        self.joint_pubs[controller_name].publish(traj)


class JointControlGUI(QtWidgets.QMainWindow):
    """Main GUI Controller"""

    def __init__(self, node):
        super().__init__()
        self.node = node

        # Load UI file from share directory
        from ament_index_python.packages import get_package_share_directory
        ui_path = Path(get_package_share_directory('humanoid_arm_control')) / 'ui' / 'joint_control.ui'
        uic.loadUi(str(ui_path), self)

        # Joint configuration
        self.joints = [
            {'name': 'base_rotation_joint', 'controller': 'base_rotation_joint_position_controller', 'index': 0, 'scale': 0.01},
            {'name': 'shoulder_pitch_joint', 'controller': 'shoulder_pitch_joint_position_controller', 'index': 1, 'scale': 0.01},
            {'name': 'elbow_pitch_joint', 'controller': 'elbow_pitch_joint_position_controller', 'index': 2, 'scale': 0.01},
            {'name': 'wrist_pitch_joint', 'controller': 'wrist_pitch_joint_position_controller', 'index': 3, 'scale': 0.01},
            {'name': 'wrist_roll_joint', 'controller': 'wrist_roll_joint_position_controller', 'index': 4, 'scale': 0.01},
        ]

        # Connect signals
        self.connect_signals()

        # Setup update timer (20 Hz)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_displays)
        self.update_timer.start(50)  # 50ms = 20Hz

        # Status
        self.statusBar.showMessage('Ready')

    def connect_signals(self):
        """Connect UI signals to slots"""
        # Emergency stop
        self.estopButton.clicked.connect(self.emergency_stop)

        # For each joint: connect enable checkbox and slider
        for joint in self.joints:
            idx = joint['index']

            # Enable checkbox
            checkbox = getattr(self, f'enableCheck{idx}')
            checkbox.stateChanged.connect(
                lambda state, j=joint: self.on_enable_changed(j, state)
            )

            # Slider
            slider = getattr(self, f'slider{idx}')
            slider.valueChanged.connect(
                lambda val, j=joint: self.on_slider_changed(j, val)
            )

    @pyqtSlot(object, int)
    def on_enable_changed(self, joint, state):
        """Handle enable/disable checkbox"""
        idx = joint['index']
        slider = getattr(self, f'slider{idx}')

        if state:
            # Activate controller
            success = self.node.switch_controller(joint['controller'], activate=True)
            if success:
                slider.setEnabled(True)
                # Send current slider position
                position = slider.value() * joint['scale']
                self.node.send_joint_command(joint['controller'], joint['name'], position)
                self.statusBar.showMessage(f"✓ {joint['name']} enabled")
            else:
                # Revert checkbox if activation failed
                checkbox = getattr(self, f'enableCheck{idx}')
                checkbox.setChecked(False)
                self.statusBar.showMessage(f"✗ Failed to enable {joint['name']}")
        else:
            # Deactivate controller
            self.node.switch_controller(joint['controller'], activate=False)
            slider.setEnabled(False)
            self.statusBar.showMessage(f"○ {joint['name']} disabled")

    @pyqtSlot(object, int)
    def on_slider_changed(self, joint, value):
        """Handle slider movement"""
        idx = joint['index']
        checkbox = getattr(self, f'enableCheck{idx}')

        # Update target label
        position = value * joint['scale']
        target_label = getattr(self, f'targetLabel{idx}')
        target_label.setText(f"{position:.2f} rad")

        # Send command if enabled
        if checkbox.isChecked():
            self.node.send_joint_command(joint['controller'], joint['name'], position)

    def update_displays(self):
        """Update all display elements from ROS data"""
        for joint in self.joints:
            name = joint['name']
            idx = joint['index']

            if name in self.node.joint_states:
                state = self.node.joint_states[name]

                # Position label
                pos_label = getattr(self, f'posLabel{idx}')
                pos_label.setText(f"{state['position']:.2f} rad")

                # Velocity label
                vel_label = getattr(self, f'velLabel{idx}')
                vel_label.setText(f"{state['velocity']:.2f} rad/s")

                # Controller effort output (PID output, what matters for tuning)
                effort = state['effort']
                effort_label = getattr(self, f'effortLabel{idx}')
                effort_label.setText(f"{effort:.1f} Nm")

                # Effort bar (progress bar from -100 to +100)
                effort_bar = getattr(self, f'effortBar{idx}')
                # Scale effort to percentage (max 50 Nm for visualization)
                effort_percent = int((effort / 50.0) * 100)
                effort_percent = max(-100, min(100, effort_percent))
                effort_bar.setValue(effort_percent)


    @pyqtSlot()
    def emergency_stop(self):
        """Disable all joints immediately"""
        for joint in self.joints:
            idx = joint['index']
            checkbox = getattr(self, f'enableCheck{idx}')
            checkbox.setChecked(False)

        self.statusBar.showMessage('⚠ EMERGENCY STOP - All joints disabled')


def main():
    # Initialize ROS2
    rclpy.init()

    # Create ROS node
    ros_node = JointControlNode()

    # Create Qt application
    app = QtWidgets.QApplication(sys.argv)

    # Create GUI
    gui = JointControlGUI(ros_node)
    gui.show()

    # Setup ROS spin timer
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    ros_timer.start(10)  # 10ms = 100Hz ROS spin

    # Handle Ctrl+C gracefully
    import signal
    signal.signal(signal.SIGINT, lambda sig, frame: app.quit())

    # Run Qt event loop
    try:
        app.exec_()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
