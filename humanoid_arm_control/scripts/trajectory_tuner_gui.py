#!/usr/bin/env python3
"""
GUI for PID tuning with test trajectories and real-time plotting
Combines test trajectory generation with live visualization
"""

import sys
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

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QComboBox, QLabel,
                             QSpinBox, QDoubleSpinBox, QGroupBox)
from PyQt5.QtCore import QTimer, pyqtSignal, QObject
import pyqtgraph as pg


class ROSWorker(QObject):
    """Worker to handle ROS2 in separate thread"""
    position_updated = pyqtSignal(list, list, list, list)  # time, desired, actual, error

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

        # Data storage for plotting
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
        self.timer.start(10)  # 100Hz

    def joint_state_callback(self, msg):
        self.current_positions = list(msg.position[:5])

    def controller_state_callback(self, msg):
        self.controller_state = msg

        # Update plotting data if trajectory is running
        if self.start_time is not None:
            current_time = time.time() - self.start_time
            self.time_data.append(current_time)

            idx = self.selected_joint_idx
            self.desired_data.append(msg.desired.positions[idx])
            self.actual_data.append(msg.actual.positions[idx])
            self.error_data.append(msg.error.positions[idx])

            # Emit signal for GUI update
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
                pass  # Ignore errors during shutdown

    def get_pid_gains(self, joint_name):
        """Get current PID gains"""
        param_names = [
            f'gains.{joint_name}.p',
            f'gains.{joint_name}.i',
            f'gains.{joint_name}.d',
            f'gains.{joint_name}.i_clamp',
            f'gains.{joint_name}.ff_velocity_scale',
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
                'i_clamp': values[3].double_value,
                'ff_vel': values[4].double_value,
            }
        return None

    def set_pid_gains(self, joint_name, p=None, i=None, d=None, i_clamp=None, ff_vel=None):
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

        if i_clamp is not None:
            param = Parameter()
            param.name = f'gains.{joint_name}.i_clamp'
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(i_clamp)
            params.append(param)

        if ff_vel is not None:
            param = Parameter()
            param.name = f'gains.{joint_name}.ff_velocity_scale'
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(ff_vel)
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
            # Two points: current → target
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
            # Multiple points for sine wave
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
        future = self.action_client.send_goal_async(goal)

        timeout = 2.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.01)

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
        self.setWindowTitle("PID Trajectory Tuner")
        self.setGeometry(100, 100, 1200, 800)

        # Create ROS worker
        self.ros_worker = ROSWorker()
        self.ros_worker.position_updated.connect(self.update_plot)

        # Main widget
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)

        # Left panel - controls
        left_panel = self.create_control_panel()
        layout.addWidget(left_panel, 1)

        # Right panel - plots
        right_panel = self.create_plot_panel()
        layout.addWidget(right_panel, 3)

    def create_control_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # Joint selection
        joint_group = QGroupBox("Joint Selection")
        joint_layout = QVBoxLayout()
        self.joint_combo = QComboBox()
        self.joint_combo.addItems([
            "4: wrist_roll (lightest)",
            "3: wrist_pitch",
            "2: elbow_pitch",
            "1: shoulder_pitch",
            "0: base_rotation (heaviest)"
        ])
        self.joint_combo.currentIndexChanged.connect(self.joint_changed)
        joint_layout.addWidget(QLabel("Select Joint:"))
        joint_layout.addWidget(self.joint_combo)
        joint_group.setLayout(joint_layout)
        layout.addWidget(joint_group)

        # PID gains
        pid_group = QGroupBox("PID Gains")
        pid_layout = QVBoxLayout()

        self.p_spin = QDoubleSpinBox()
        self.p_spin.setRange(0, 1000)
        self.p_spin.setValue(100)
        self.p_spin.setSingleStep(10)

        self.i_spin = QDoubleSpinBox()
        self.i_spin.setRange(0, 100)
        self.i_spin.setValue(0)
        self.i_spin.setSingleStep(1)

        self.d_spin = QDoubleSpinBox()
        self.d_spin.setRange(0, 100)
        self.d_spin.setValue(10)
        self.d_spin.setSingleStep(1)

        pid_layout.addWidget(QLabel("P Gain:"))
        pid_layout.addWidget(self.p_spin)
        pid_layout.addWidget(QLabel("I Gain:"))
        pid_layout.addWidget(self.i_spin)
        pid_layout.addWidget(QLabel("D Gain:"))
        pid_layout.addWidget(self.d_spin)

        apply_btn = QPushButton("Apply PID Gains")
        apply_btn.clicked.connect(self.apply_pid)
        pid_layout.addWidget(apply_btn)

        get_btn = QPushButton("Get Current Gains")
        get_btn.clicked.connect(self.get_current_pid)
        pid_layout.addWidget(get_btn)

        pid_group.setLayout(pid_layout)
        layout.addWidget(pid_group)

        # Trajectory settings
        traj_group = QGroupBox("Trajectory")
        traj_layout = QVBoxLayout()

        self.traj_type = QComboBox()
        self.traj_type.addItems(["step", "sine"])

        self.amplitude_spin = QDoubleSpinBox()
        self.amplitude_spin.setRange(0.1, 3.0)
        self.amplitude_spin.setValue(0.5)
        self.amplitude_spin.setSingleStep(0.1)

        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setRange(1.0, 10.0)
        self.duration_spin.setValue(5.0)
        self.duration_spin.setSingleStep(0.5)

        traj_layout.addWidget(QLabel("Type:"))
        traj_layout.addWidget(self.traj_type)
        traj_layout.addWidget(QLabel("Amplitude (rad):"))
        traj_layout.addWidget(self.amplitude_spin)
        traj_layout.addWidget(QLabel("Duration (s):"))
        traj_layout.addWidget(self.duration_spin)

        run_btn = QPushButton("Run Trajectory")
        run_btn.clicked.connect(self.run_trajectory)
        run_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        traj_layout.addWidget(run_btn)

        traj_group.setLayout(traj_layout)
        layout.addWidget(traj_group)

        layout.addStretch()

        return panel

    def create_plot_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # Position plot
        self.pos_plot = pg.PlotWidget(title="Position Tracking")
        self.pos_plot.setLabel('left', 'Position', units='rad')
        self.pos_plot.setLabel('bottom', 'Time', units='s')
        self.pos_plot.addLegend()

        self.desired_curve = self.pos_plot.plot(pen='r', name='Desired')
        self.actual_curve = self.pos_plot.plot(pen='b', name='Actual')

        layout.addWidget(self.pos_plot)

        # Error plot
        self.error_plot = pg.PlotWidget(title="Tracking Error")
        self.error_plot.setLabel('left', 'Error', units='rad')
        self.error_plot.setLabel('bottom', 'Time', units='s')

        self.error_curve = self.error_plot.plot(pen='r')

        layout.addWidget(self.error_plot)

        return panel

    def joint_changed(self):
        # Extract joint index from combo text
        text = self.joint_combo.currentText()
        self.current_joint_idx = int(text.split(':')[0])
        self.get_current_pid()

    def apply_pid(self):
        joint_idx = int(self.joint_combo.currentText().split(':')[0])
        joint_name = self.ros_worker.joint_names[joint_idx]

        success = self.ros_worker.set_pid_gains(
            joint_name,
            p=self.p_spin.value(),
            i=self.i_spin.value(),
            d=self.d_spin.value()
        )

        if success:
            print(f"✓ Applied PID gains to {joint_name}")
        else:
            print(f"✗ Failed to apply gains")

    def get_current_pid(self):
        joint_idx = int(self.joint_combo.currentText().split(':')[0])
        joint_name = self.ros_worker.joint_names[joint_idx]

        gains = self.ros_worker.get_pid_gains(joint_name)
        if gains:
            self.p_spin.setValue(gains['p'])
            self.i_spin.setValue(gains['i'])
            self.d_spin.setValue(gains['d'])
            print(f"✓ Got current gains for {joint_name}")

    def run_trajectory(self):
        joint_idx = int(self.joint_combo.currentText().split(':')[0])

        self.ros_worker.send_trajectory(
            joint_idx,
            self.traj_type.currentText(),
            self.amplitude_spin.value(),
            self.duration_spin.value()
        )

        print(f"▶ Running {self.traj_type.currentText()} trajectory on joint {joint_idx}")

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
