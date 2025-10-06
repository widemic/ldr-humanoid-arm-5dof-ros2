#!/usr/bin/env python3
"""
PID Trajectory Tuner GUI - Loads UI from Qt Designer file
Separates UI design from logic
"""

import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from builtin_interfaces.msg import Duration
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import numpy as np
import time

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer, pyqtSignal, QObject
from PyQt5 import uic
import pyqtgraph as pg


class ROSWorker(QObject):
    """Worker to handle ROS2 operations"""
    position_updated = pyqtSignal(list, list, list, list)

    def __init__(self):
        super().__init__()
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('trajectory_tuner_gui')
        self.is_shutdown = False

        self.joint_names = [
            'base_rotation_joint',
            'shoulder_pitch_joint',
            'elbow_pitch_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint'
        ]

        self.current_positions = None
        self.controller_state = None
        self.selected_joint_idx = 0

        # Data storage
        self.time_data = []
        self.desired_data = []
        self.actual_data = []
        self.error_data = []
        self.start_time = None

        # Subscribers
        self.joint_state_sub = self.node.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        self.controller_state_sub = self.node.create_subscription(
            JointTrajectoryControllerState,
            '/joint_trajectory_controller/controller_state',
            self.controller_state_callback, 10)

        # Action client
        self.action_client = ActionClient(
            self.node, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')

        # Service clients
        self.set_param_client = self.node.create_client(
            SetParameters, '/joint_trajectory_controller/set_parameters')
        self.get_param_client = self.node.create_client(
            GetParameters, '/joint_trajectory_controller/get_parameters')

        # Spin timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_once)
        self.timer.start(10)

    def joint_state_callback(self, msg):
        self.current_positions = list(msg.position[:5])

    def controller_state_callback(self, msg):
        self.controller_state = msg

        if self.start_time is not None:
            current_time = time.time() - self.start_time
            self.time_data.append(current_time)

            idx = self.selected_joint_idx
            self.desired_data.append(msg.desired.positions[idx])
            self.actual_data.append(msg.actual.positions[idx])
            self.error_data.append(msg.error.positions[idx])

            self.position_updated.emit(
                self.time_data.copy(),
                self.desired_data.copy(),
                self.actual_data.copy(),
                self.error_data.copy()
            )

    def spin_once(self):
        if not self.is_shutdown and rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0)
            except Exception:
                pass

    def get_pid_gains(self, joint_name):
        """Get current PID gains"""
        param_names = [
            f'gains.{joint_name}.p',
            f'gains.{joint_name}.i',
            f'gains.{joint_name}.d',
        ]

        request = GetParameters.Request()
        request.names = param_names
        future = self.get_param_client.call_async(request)

        timeout = 2.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.01)

        if future.result():
            values = future.result().values
            return {
                'p': values[0].double_value,
                'i': values[1].double_value,
                'd': values[2].double_value,
            }
        return None

    def set_pid_gains(self, joint_name, p=None, i=None, d=None):
        """Set PID gains"""
        params = []

        if p is not None:
            param = Parameter()
            param.name = f'gains.{joint_name}.p'
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(p)
            params.append(param)

        if i is not None:
            param = Parameter()
            param.name = f'gains.{joint_name}.i'
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(i)
            params.append(param)

        if d is not None:
            param = Parameter()
            param.name = f'gains.{joint_name}.d'
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(d)
            params.append(param)

        request = SetParameters.Request()
        request.parameters = params
        future = self.set_param_client.call_async(request)

        timeout = 2.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.01)

        return future.done()

    def send_trajectory(self, joint_idx, trajectory_type, amplitude, duration):
        """Send test trajectory"""
        # Reset data
        self.time_data = []
        self.desired_data = []
        self.actual_data = []
        self.error_data = []
        self.start_time = time.time()
        self.selected_joint_idx = joint_idx

        # Wait for current position
        timeout = 2.0
        start = time.time()
        while self.current_positions is None and (time.time() - start) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.01)

        if self.current_positions is None:
            start_pos = [0.0] * 5
        else:
            start_pos = self.current_positions.copy()

        # Generate trajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        if trajectory_type == 'step':
            point1 = JointTrajectoryPoint()
            point1.positions = start_pos
            point1.time_from_start = Duration(sec=0, nanosec=0)

            target_pos = start_pos.copy()
            target_pos[joint_idx] += amplitude

            point2 = JointTrajectoryPoint()
            point2.positions = target_pos
            point2.time_from_start = Duration(sec=int(duration),
                                             nanosec=int((duration % 1) * 1e9))

            goal.trajectory.points = [point1, point2]

        elif trajectory_type == 'sine':
            points = 50
            for i in range(points + 1):
                t = i * duration / points
                point = JointTrajectoryPoint()
                point.positions = start_pos.copy()
                point.positions[joint_idx] += amplitude * np.sin(2 * np.pi * t / duration)
                point.time_from_start = Duration(sec=int(t),
                                                nanosec=int((t % 1) * 1e9))
                goal.trajectory.points.append(point)

        # Send goal
        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal)

    def cleanup(self):
        self.is_shutdown = True
        self.timer.stop()
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


class TunerGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # Load UI file
        ui_file = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'ui', 'trajectory_tuner.ui'
        )
        uic.loadUi(ui_file, self)

        # Create ROS worker
        self.ros_worker = ROSWorker()
        self.ros_worker.position_updated.connect(self.update_plot)

        # Setup plots in the placeholder widgets
        self.setup_plots()

        # Connect signals
        self.jointCombo.currentIndexChanged.connect(self.joint_changed)
        self.applyBtn.clicked.connect(self.apply_pid)
        self.getBtn.clicked.connect(self.get_current_pid)
        self.runBtn.clicked.connect(self.run_trajectory)

    def setup_plots(self):
        """Replace placeholder widgets with actual plots"""
        # Position plot
        self.pos_plot_widget = pg.PlotWidget(title="Position Tracking")
        self.pos_plot_widget.setLabel('left', 'Position', units='rad')
        self.pos_plot_widget.setLabel('bottom', 'Time', units='s')
        self.pos_plot_widget.addLegend()
        self.desired_curve = self.pos_plot_widget.plot(pen='r', name='Desired')
        self.actual_curve = self.pos_plot_widget.plot(pen='b', name='Actual')

        # Error plot
        self.error_plot_widget = pg.PlotWidget(title="Tracking Error")
        self.error_plot_widget.setLabel('left', 'Error', units='rad')
        self.error_plot_widget.setLabel('bottom', 'Time', units='s')
        self.error_curve = self.error_plot_widget.plot(pen='r')

        # Replace placeholders
        layout = self.plotPanel.layout()
        layout.replaceWidget(self.posPlot, self.pos_plot_widget)
        layout.replaceWidget(self.errorPlot, self.error_plot_widget)
        self.posPlot.deleteLater()
        self.errorPlot.deleteLater()

    def joint_changed(self):
        text = self.jointCombo.currentText()
        self.current_joint_idx = int(text.split(':')[0])
        self.get_current_pid()

    def apply_pid(self):
        joint_idx = int(self.jointCombo.currentText().split(':')[0])
        joint_name = self.ros_worker.joint_names[joint_idx]

        success = self.ros_worker.set_pid_gains(
            joint_name,
            p=self.pSpin.value(),
            i=self.iSpin.value(),
            d=self.dSpin.value()
        )

        if success:
            print(f"✓ Applied PID gains to {joint_name}")
        else:
            print(f"✗ Failed to apply gains")

    def get_current_pid(self):
        joint_idx = int(self.jointCombo.currentText().split(':')[0])
        joint_name = self.ros_worker.joint_names[joint_idx]

        gains = self.ros_worker.get_pid_gains(joint_name)
        if gains:
            self.pSpin.setValue(gains['p'])
            self.iSpin.setValue(gains['i'])
            self.dSpin.setValue(gains['d'])
            print(f"✓ Got current gains for {joint_name}")

    def run_trajectory(self):
        joint_idx = int(self.jointCombo.currentText().split(':')[0])

        self.ros_worker.send_trajectory(
            joint_idx,
            self.trajType.currentText(),
            self.amplitudeSpin.value(),
            self.durationSpin.value()
        )

        print(f"▶ Running {self.trajType.currentText()} trajectory on joint {joint_idx}")

    def update_plot(self, time_data, desired, actual, error):
        self.desired_curve.setData(time_data, desired)
        self.actual_curve.setData(time_data, actual)
        self.error_curve.setData(time_data, error)

    def closeEvent(self, event):
        self.ros_worker.cleanup()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = TunerGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
