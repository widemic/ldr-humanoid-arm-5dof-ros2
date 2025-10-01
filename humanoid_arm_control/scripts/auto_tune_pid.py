#!/usr/bin/env python3
"""
Automatic PID tuning using relay feedback method (Ziegler-Nichols variant)
This finds the ultimate gain (Ku) and period (Pu) by inducing oscillations
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from builtin_interfaces.msg import Duration
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType
import numpy as np
import argparse
import time


class AutoTuner(Node):
    def __init__(self, joint_idx):
        super().__init__('auto_tuner')

        self.joint_names = [
            'base_rotation_joint',
            'shoulder_pitch_joint',
            'elbow_pitch_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint'
        ]

        self.joint_idx = joint_idx
        self.joint_name = self.joint_names[joint_idx]

        # Oscillation detection
        self.error_history = []
        self.time_history = []
        self.peaks = []
        self.start_time = None

        # Subscribe to controller state
        self.state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/joint_trajectory_controller/controller_state',
            self.state_callback,
            10
        )

        # Service client
        self.set_param_client = self.create_client(
            SetParameters,
            '/joint_trajectory_controller/set_parameters'
        )

        # Action client
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info(f'🤖 Auto-tuning {self.joint_name}...')

    def state_callback(self, msg):
        """Collect tracking error data"""
        if self.start_time is None:
            return

        current_time = time.time() - self.start_time
        error = msg.error.positions[self.joint_idx]

        self.time_history.append(current_time)
        self.error_history.append(error)

        # Detect peaks (oscillations)
        if len(self.error_history) >= 3:
            if (self.error_history[-2] > self.error_history[-3] and
                self.error_history[-2] > self.error_history[-1] and
                abs(self.error_history[-2]) > 0.01):  # Minimum peak threshold
                self.peaks.append((self.time_history[-2], self.error_history[-2]))

    def set_pid(self, p, i, d):
        """Set PID gains"""
        params = []

        param = Parameter()
        param.name = f'gains.{self.joint_name}.p'
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(p)
        params.append(param)

        param = Parameter()
        param.name = f'gains.{self.joint_name}.i'
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(i)
        params.append(param)

        param = Parameter()
        param.name = f'gains.{self.joint_name}.d'
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(d)
        params.append(param)

        request = SetParameters.Request()
        request.parameters = params

        future = self.set_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        return future.done()

    def send_step_trajectory(self, amplitude=0.5, duration=5.0):
        """Send a step trajectory"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        # Start position (current)
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0] * 5  # Simplified
        point1.time_from_start = Duration(sec=0, nanosec=0)

        # Target position
        point2 = JointTrajectoryPoint()
        point2.positions = [0.0] * 5
        point2.positions[self.joint_idx] = amplitude
        point2.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))

        goal.trajectory.points = [point1, point2]

        # Reset data collection
        self.error_history = []
        self.time_history = []
        self.peaks = []
        self.start_time = time.time()

        # Send
        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

    def find_ultimate_gain(self, p_start=10, p_step=20, max_iterations=10):
        """
        Iteratively increase P gain until sustained oscillations occur
        Returns (Ku, Pu) - ultimate gain and period
        """
        self.get_logger().info('🔍 Phase 1: Finding ultimate gain (Ku)...')

        p_current = p_start

        for iteration in range(max_iterations):
            # Set P gain, zero I and D
            self.get_logger().info(f'   Testing P = {p_current}')
            self.set_pid(p_current, 0, 0)
            time.sleep(0.5)

            # Send step trajectory
            self.send_step_trajectory(amplitude=0.3, duration=5.0)

            # Wait for trajectory to complete
            time.sleep(6.0)

            # Check for oscillations
            if len(self.peaks) >= 3:
                # Calculate period from peaks
                periods = []
                for i in range(1, len(self.peaks)):
                    period = self.peaks[i][0] - self.peaks[i-1][0]
                    periods.append(period)

                avg_period = np.mean(periods)

                # Check if oscillations are sustained (consistent period)
                period_std = np.std(periods)
                if period_std < avg_period * 0.3:  # Period variation < 30%
                    self.get_logger().info(f'✓ Found sustained oscillations!')
                    self.get_logger().info(f'   Ku = {p_current}')
                    self.get_logger().info(f'   Pu = {avg_period:.3f} seconds')
                    return p_current, avg_period

            # Increase P and try again
            p_current += p_step

        self.get_logger().warn('⚠ Could not find sustained oscillations. Try manually.')
        return None, None

    def calculate_ziegler_nichols(self, ku, pu):
        """
        Calculate PID gains using Ziegler-Nichols method

        Classic Ziegler-Nichols:
        - P controller:   Kp = 0.5 * Ku
        - PI controller:  Kp = 0.45 * Ku,  Ki = 0.54 * Ku / Pu
        - PID controller: Kp = 0.6 * Ku,   Ki = 1.2 * Ku / Pu,  Kd = 0.075 * Ku * Pu

        Returns: (P, I, D)
        """
        # Use PID formula
        p = 0.6 * ku
        i = 1.2 * ku / pu
        d = 0.075 * ku * pu

        return p, i, d

    def auto_tune(self):
        """Main auto-tuning procedure"""
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'🎛️  AUTO-TUNING: {self.joint_name}')
        self.get_logger().info('=' * 60)

        # Step 1: Find ultimate gain
        ku, pu = self.find_ultimate_gain(p_start=20, p_step=30)

        if ku is None:
            self.get_logger().error('❌ Auto-tuning failed')
            return None

        # Step 2: Calculate PID gains
        self.get_logger().info('')
        self.get_logger().info('🧮 Phase 2: Calculating PID gains...')
        p, i, d = self.calculate_ziegler_nichols(ku, pu)

        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('✅ AUTO-TUNING COMPLETE')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Recommended PID gains for {self.joint_name}:')
        self.get_logger().info(f'  P = {p:.1f}')
        self.get_logger().info(f'  I = {i:.1f}')
        self.get_logger().info(f'  D = {d:.1f}')
        self.get_logger().info('')
        self.get_logger().info('Applying gains...')

        # Step 3: Apply the gains
        self.set_pid(p, i, d)

        self.get_logger().info('✓ Gains applied! Test with:')
        self.get_logger().info(f'  python3 humanoid_arm_control/scripts/test_trajectory.py --joint {self.joint_idx} --reset')

        return p, i, d


def main():
    parser = argparse.ArgumentParser(description='Automatic PID tuning using relay feedback')
    parser.add_argument('--joint', type=int, required=True, choices=[0, 1, 2, 3, 4],
                       help='Joint index (0=base, 1=shoulder, 2=elbow, 3=wrist_pitch, 4=wrist_roll)')

    args = parser.parse_args()

    rclpy.init()

    tuner = AutoTuner(args.joint)

    try:
        tuner.auto_tune()
    except KeyboardInterrupt:
        pass
    finally:
        tuner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
