#!/usr/bin/env python3
"""
Simple GUI for PID tuning without real-time plotting
Use PlotJuggler separately for visualization
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import numpy as np
import time

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QComboBox, QLabel,
                             QDoubleSpinBox, QGroupBox, QGridLayout)
from PyQt5.QtCore import QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont


class ROSWorker(QObject):
    """Worker to handle ROS2"""
    status_updated = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('simple_tuner_gui')
        self.is_shutdown = False

        self.joint_names = [
            'base_rotation_joint',
            'shoulder_pitch_joint',
            'elbow_pitch_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint'
        ]

        self.current_positions = None

        # Subscribers
        self.joint_state_sub = self.node.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

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
        self.status_updated.emit(f"▶ Running {trajectory_type} trajectory...")

        # Wait for current position
        timeout = 2.0
        start = time.time()
        while self.current_positions is None and (time.time() - start) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.01)

        if self.current_positions is None:
            start_pos = [0.0] * 5
            self.status_updated.emit("⚠ Could not get current position, using zero")
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

        elif trajectory_type == 'square':
            # Square wave - 3 cycles within duration
            cycles = 3
            period = duration / cycles
            points_per_half = 10

            for cycle in range(cycles):
                # High phase
                for i in range(points_per_half):
                    t = cycle * period + i * (period / 2) / points_per_half
                    point = JointTrajectoryPoint()
                    point.positions = start_pos.copy()
                    point.positions[joint_idx] += amplitude
                    point.time_from_start = Duration(sec=int(t),
                                                    nanosec=int((t % 1) * 1e9))
                    goal.trajectory.points.append(point)

                # Low phase
                for i in range(points_per_half):
                    t = cycle * period + (period / 2) + i * (period / 2) / points_per_half
                    point = JointTrajectoryPoint()
                    point.positions = start_pos.copy()
                    point.positions[joint_idx] -= amplitude
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

        if future.done():
            self.status_updated.emit(f"✓ Trajectory sent! Use PlotJuggler to monitor")
        else:
            self.status_updated.emit("✗ Failed to send trajectory")

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


class SimpleTunerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PID Tuner - Use PlotJuggler for Visualization")
        self.setGeometry(100, 100, 600, 700)

        # Create ROS worker
        self.ros_worker = ROSWorker()
        self.ros_worker.status_updated.connect(self.update_status)

        # Main widget
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)

        # Title
        title = QLabel("🎛️  Simple PID Tuner")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        # Instructions
        instructions = QLabel(
            "Use PlotJuggler to visualize tracking:\n"
            "ros2 run plotjuggler plotjuggler\n"
            "Subscribe to: /joint_trajectory_controller/controller_state"
        )
        instructions.setStyleSheet("background-color: #f0f0f0; padding: 10px; border-radius: 5px;")
        layout.addWidget(instructions)

        # Joint selection
        joint_group = self.create_joint_group()
        layout.addWidget(joint_group)

        # PID gains
        pid_group = self.create_pid_group()
        layout.addWidget(pid_group)

        # Trajectory settings
        traj_group = self.create_trajectory_group()
        layout.addWidget(traj_group)

        # Status
        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet(
            "background-color: #e8f5e9; padding: 10px; "
            "border-radius: 5px; font-weight: bold;"
        )
        layout.addWidget(self.status_label)

        layout.addStretch()

    def create_joint_group(self):
        group = QGroupBox("1. Select Joint (Start from Lightest!)")
        layout = QVBoxLayout()

        self.joint_combo = QComboBox()
        self.joint_combo.addItems([
            "4: wrist_roll_joint (RMD-X4, 17 Nm) ← START HERE",
            "3: wrist_pitch_joint (RMD-X6, 60 Nm)",
            "2: elbow_pitch_joint (RMD-X6, 60 Nm)",
            "1: shoulder_pitch_joint (RMD-X8-PRO, 120 Nm)",
            "0: base_rotation_joint (RMD-X8-PRO, 120 Nm) ← TUNE LAST"
        ])
        self.joint_combo.currentIndexChanged.connect(self.joint_changed)
        layout.addWidget(self.joint_combo)

        group.setLayout(layout)
        return group

    def create_pid_group(self):
        group = QGroupBox("2. Adjust PID Gains")
        grid = QGridLayout()

        # P gain
        grid.addWidget(QLabel("P Gain:"), 0, 0)
        self.p_spin = QDoubleSpinBox()
        self.p_spin.setRange(0, 1000)
        self.p_spin.setValue(50)
        self.p_spin.setSingleStep(10)
        grid.addWidget(self.p_spin, 0, 1)

        # I gain
        grid.addWidget(QLabel("I Gain:"), 1, 0)
        self.i_spin = QDoubleSpinBox()
        self.i_spin.setRange(0, 100)
        self.i_spin.setValue(0)
        self.i_spin.setSingleStep(1)
        grid.addWidget(self.i_spin, 1, 1)

        # D gain
        grid.addWidget(QLabel("D Gain:"), 2, 0)
        self.d_spin = QDoubleSpinBox()
        self.d_spin.setRange(0, 100)
        self.d_spin.setValue(5)
        self.d_spin.setSingleStep(1)
        grid.addWidget(self.d_spin, 2, 1)

        # Buttons
        button_layout = QHBoxLayout()

        get_btn = QPushButton("📥 Get Current")
        get_btn.clicked.connect(self.get_current_pid)
        button_layout.addWidget(get_btn)

        apply_btn = QPushButton("✅ Apply Gains")
        apply_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        apply_btn.clicked.connect(self.apply_pid)
        button_layout.addWidget(apply_btn)

        grid.addLayout(button_layout, 3, 0, 1, 2)

        group.setLayout(grid)
        return group

    def create_trajectory_group(self):
        group = QGroupBox("3. Run Test Trajectory")
        grid = QGridLayout()

        # Type
        grid.addWidget(QLabel("Type:"), 0, 0)
        self.traj_type = QComboBox()
        self.traj_type.addItems(["step", "sine", "square"])
        grid.addWidget(self.traj_type, 0, 1)

        # Amplitude
        grid.addWidget(QLabel("Amplitude (rad):"), 1, 0)
        self.amplitude_spin = QDoubleSpinBox()
        self.amplitude_spin.setRange(0.1, 3.0)
        self.amplitude_spin.setValue(0.5)
        self.amplitude_spin.setSingleStep(0.1)
        grid.addWidget(self.amplitude_spin, 1, 1)

        # Duration
        grid.addWidget(QLabel("Duration (s):"), 2, 0)
        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setRange(1.0, 10.0)
        self.duration_spin.setValue(5.0)
        self.duration_spin.setSingleStep(0.5)
        grid.addWidget(self.duration_spin, 2, 1)

        # Run button
        run_btn = QPushButton("▶ Run Trajectory")
        run_btn.setStyleSheet(
            "background-color: #2196F3; color: white; "
            "font-weight: bold; padding: 10px; font-size: 14px;"
        )
        run_btn.clicked.connect(self.run_trajectory)
        grid.addWidget(run_btn, 3, 0, 1, 2)

        group.setLayout(grid)
        return group

    def joint_changed(self):
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
            self.update_status(f"✓ Applied PID to {joint_name}: P={self.p_spin.value()}, D={self.d_spin.value()}")
        else:
            self.update_status(f"✗ Failed to apply gains to {joint_name}")

    def get_current_pid(self):
        joint_idx = int(self.joint_combo.currentText().split(':')[0])
        joint_name = self.ros_worker.joint_names[joint_idx]

        gains = self.ros_worker.get_pid_gains(joint_name)
        if gains:
            self.p_spin.setValue(gains['p'])
            self.i_spin.setValue(gains['i'])
            self.d_spin.setValue(gains['d'])
            self.update_status(f"✓ Got gains for {joint_name}: P={gains['p']}, I={gains['i']}, D={gains['d']}")
        else:
            self.update_status(f"✗ Failed to get gains for {joint_name}")

    def run_trajectory(self):
        joint_idx = int(self.joint_combo.currentText().split(':')[0])

        self.ros_worker.send_trajectory(
            joint_idx,
            self.traj_type.currentText(),
            self.amplitude_spin.value(),
            self.duration_spin.value()
        )

    def update_status(self, message):
        self.status_label.setText(message)

    def closeEvent(self, event):
        self.ros_worker.cleanup()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = SimpleTunerGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
