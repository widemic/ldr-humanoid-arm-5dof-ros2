#!/usr/bin/env python3
"""
Continuous test trajectory generator - runs indefinitely until Ctrl+C
Perfect for PID tuning while adjusting parameters in real-time
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import argparse
import signal
import sys


class ContinuousTrajectoryTester(Node):
    def __init__(self, joint_idx, traj_type, amplitude, period, frequency):
        super().__init__('continuous_trajectory_tester')

        self.joint_idx = joint_idx
        self.traj_type = traj_type
        self.amplitude = amplitude
        self.period = period
        self.frequency = frequency
        self.running = True

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.joint_names = [
            'base_rotation_joint',
            'shoulder_pitch_joint',
            'elbow_pitch_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint'
        ]

        # Handle Ctrl+C gracefully
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        self.get_logger().info('\n🛑 Stopping continuous test...')
        self.running = False
        sys.exit(0)

    def wait_for_server(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server ready!')

    def send_goal_async(self, goal_msg, callback):
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(callback)

    def generate_square_wave_cycle(self):
        """Generate one cycle of square wave"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        points = []
        half_period = self.period / 2.0

        # High
        point_high = JointTrajectoryPoint()
        point_high.positions = [0.0] * 5
        point_high.positions[self.joint_idx] = self.amplitude
        point_high.velocities = [0.0] * 5
        t = half_period
        point_high.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        points.append(point_high)

        # Low
        point_low = JointTrajectoryPoint()
        point_low.positions = [0.0] * 5
        point_low.positions[self.joint_idx] = -self.amplitude
        point_low.velocities = [0.0] * 5
        t = self.period
        point_low.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        points.append(point_low)

        goal.trajectory.points = points
        return goal

    def generate_sine_wave_cycle(self):
        """Generate one cycle of sine wave"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        dt = 0.05
        cycle_time = 1.0 / self.frequency
        num_points = int(cycle_time / dt)

        points = []
        for i in range(num_points):
            t = i * dt
            point = JointTrajectoryPoint()
            point.positions = [0.0] * 5
            point.positions[self.joint_idx] = self.amplitude * np.sin(2 * np.pi * self.frequency * t)
            point.velocities = [0.0] * 5
            point.velocities[self.joint_idx] = self.amplitude * 2 * np.pi * self.frequency * np.cos(2 * np.pi * self.frequency * t)
            point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            points.append(point)

        goal.trajectory.points = points
        return goal

    def run_continuous(self):
        """Run continuous trajectory"""
        cycle_count = 0

        joint_name = self.joint_names[self.joint_idx]
        self.get_logger().info(f'\n🔄 Starting continuous {self.traj_type} on {joint_name}')
        self.get_logger().info(f'   Amplitude: {self.amplitude} rad')
        if self.traj_type == 'square':
            self.get_logger().info(f'   Period: {self.period} s')
        else:
            self.get_logger().info(f'   Frequency: {self.frequency} Hz')
        self.get_logger().info('\n💡 Adjust PID parameters using the GUI or tune_pid.py while this runs!')
        self.get_logger().info('   Press Ctrl+C to stop\n')

        while self.running:
            cycle_count += 1

            if self.traj_type == 'square':
                goal = self.generate_square_wave_cycle()
            else:  # sine
                goal = self.generate_sine_wave_cycle()

            self.get_logger().info(f'Cycle {cycle_count}...')

            # Send goal and wait for completion
            send_goal_future = self._action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future)

            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected!')
                continue

            # Wait for trajectory to complete
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)


def main():
    parser = argparse.ArgumentParser(description='Continuous test trajectory for PID tuning')
    parser.add_argument('--joint', type=int, default=0, choices=[0, 1, 2, 3, 4],
                       help='Joint index (0=base, 1=shoulder, 2=elbow, 3=wrist_pitch, 4=wrist_roll)')
    parser.add_argument('--type', type=str, default='square',
                       choices=['square', 'sine'],
                       help='Trajectory type')
    parser.add_argument('--amplitude', type=float, default=0.5,
                       help='Amplitude in radians (default: 0.5)')
    parser.add_argument('--period', type=float, default=2.0,
                       help='Period in seconds for square wave (default: 2.0)')
    parser.add_argument('--frequency', type=float, default=0.5,
                       help='Frequency in Hz for sine wave (default: 0.5)')

    args = parser.parse_args()

    rclpy.init()
    node = ContinuousTrajectoryTester(args.joint, args.type, args.amplitude, args.period, args.frequency)

    node.wait_for_server()
    node.run_continuous()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()