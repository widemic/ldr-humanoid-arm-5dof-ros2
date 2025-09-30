#!/usr/bin/env python3
"""
Generate test trajectories for PID tuning
Supports: step, square wave, sine wave, chirp
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import argparse


class TrajectoryTester(Node):
    def __init__(self):
        super().__init__('trajectory_tester')
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

    def wait_for_server(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server ready!')

    def send_goal(self, goal_msg):
        self.get_logger().info('Sending trajectory...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f'Result: {result.error_code}')

    def generate_step(self, joint_idx, amplitude, duration=2.0):
        """Generate step response test"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        # Point 1: Start position (all zeros)
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0] * 5
        point1.velocities = [0.0] * 5
        point1.time_from_start = Duration(sec=0, nanosec=0)

        # Point 2: Step to amplitude
        point2 = JointTrajectoryPoint()
        point2.positions = [0.0] * 5
        point2.positions[joint_idx] = amplitude
        point2.velocities = [0.0] * 5
        point2.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))

        goal.trajectory.points = [point1, point2]
        return goal

    def generate_square_wave(self, joint_idx, amplitude, period=2.0, num_cycles=3):
        """Generate square wave for frequency response"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        points = []
        half_period = period / 2.0

        for cycle in range(num_cycles):
            # High
            point_high = JointTrajectoryPoint()
            point_high.positions = [0.0] * 5
            point_high.positions[joint_idx] = amplitude
            point_high.velocities = [0.0] * 5
            t = cycle * period + half_period
            point_high.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            points.append(point_high)

            # Low
            point_low = JointTrajectoryPoint()
            point_low.positions = [0.0] * 5
            point_low.positions[joint_idx] = -amplitude
            point_low.velocities = [0.0] * 5
            t = (cycle + 1) * period
            point_low.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            points.append(point_low)

        # Return to zero
        point_zero = JointTrajectoryPoint()
        point_zero.positions = [0.0] * 5
        point_zero.velocities = [0.0] * 5
        t = num_cycles * period + 1.0
        point_zero.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        points.append(point_zero)

        goal.trajectory.points = points
        return goal

    def generate_sine_wave(self, joint_idx, amplitude, frequency=0.5, duration=10.0):
        """Generate sine wave for smooth tracking test"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        dt = 0.1  # 100ms time step
        num_points = int(duration / dt)

        points = []
        for i in range(num_points):
            t = i * dt
            point = JointTrajectoryPoint()
            point.positions = [0.0] * 5
            point.positions[joint_idx] = amplitude * np.sin(2 * np.pi * frequency * t)
            point.velocities = [0.0] * 5
            point.velocities[joint_idx] = amplitude * 2 * np.pi * frequency * np.cos(2 * np.pi * frequency * t)
            point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            points.append(point)

        goal.trajectory.points = points
        return goal

    def generate_chirp(self, joint_idx, amplitude, freq_start=0.1, freq_end=2.0, duration=10.0):
        """Generate frequency sweep (chirp) for system identification"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        dt = 0.05  # 50ms time step
        num_points = int(duration / dt)

        points = []
        for i in range(num_points):
            t = i * dt
            # Linear frequency sweep
            freq = freq_start + (freq_end - freq_start) * (t / duration)
            phase = 2 * np.pi * (freq_start * t + 0.5 * (freq_end - freq_start) * (t**2) / duration)

            point = JointTrajectoryPoint()
            point.positions = [0.0] * 5
            point.positions[joint_idx] = amplitude * np.sin(phase)
            point.velocities = [0.0] * 5
            point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            points.append(point)

        goal.trajectory.points = points
        return goal


def main():
    parser = argparse.ArgumentParser(description='Generate test trajectories for PID tuning')
    parser.add_argument('--joint', type=int, default=0, choices=[0, 1, 2, 3, 4],
                       help='Joint index (0=base, 1=shoulder, 2=elbow, 3=wrist_pitch, 4=wrist_roll)')
    parser.add_argument('--type', type=str, default='step',
                       choices=['step', 'square', 'sine', 'chirp'],
                       help='Trajectory type')
    parser.add_argument('--amplitude', type=float, default=0.5,
                       help='Amplitude in radians (default: 0.5)')
    parser.add_argument('--period', type=float, default=2.0,
                       help='Period in seconds for square wave (default: 2.0)')
    parser.add_argument('--frequency', type=float, default=0.5,
                       help='Frequency in Hz for sine wave (default: 0.5)')
    parser.add_argument('--duration', type=float, default=10.0,
                       help='Total duration in seconds (default: 10.0)')
    parser.add_argument('--cycles', type=int, default=3,
                       help='Number of cycles for square wave (default: 3)')

    args = parser.parse_args()

    joint_names_map = {
        0: 'base_rotation_joint',
        1: 'shoulder_pitch_joint',
        2: 'elbow_pitch_joint',
        3: 'wrist_pitch_joint',
        4: 'wrist_roll_joint'
    }

    rclpy.init()
    node = TrajectoryTester()

    node.wait_for_server()

    print(f"\n🎯 Generating {args.type} trajectory:")
    print(f"   Joint: {joint_names_map[args.joint]} (index {args.joint})")
    print(f"   Amplitude: {args.amplitude} rad")

    if args.type == 'step':
        print(f"   Duration: {args.duration} s")
        goal = node.generate_step(args.joint, args.amplitude, args.duration)
    elif args.type == 'square':
        print(f"   Period: {args.period} s")
        print(f"   Cycles: {args.cycles}")
        goal = node.generate_square_wave(args.joint, args.amplitude, args.period, args.cycles)
    elif args.type == 'sine':
        print(f"   Frequency: {args.frequency} Hz")
        print(f"   Duration: {args.duration} s")
        goal = node.generate_sine_wave(args.joint, args.amplitude, args.frequency, args.duration)
    elif args.type == 'chirp':
        print(f"   Duration: {args.duration} s")
        print(f"   Frequency sweep: 0.1 - 2.0 Hz")
        goal = node.generate_chirp(args.joint, args.amplitude, duration=args.duration)

    print("\n📊 Start PlotJuggler to monitor the response!")
    print(f"   ros2 run plotjuggler plotjuggler\n")

    node.send_goal(goal)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()