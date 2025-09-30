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
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import numpy as np
import argparse
import time


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

        # Subscribe to joint states to get current position
        self.current_positions = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Service client for getting parameters
        from rcl_interfaces.srv import GetParameters
        self.get_param_client = self.create_client(
            GetParameters,
            '/joint_trajectory_controller/get_parameters'
        )

    def joint_state_callback(self, msg):
        """Store current joint positions"""
        self.current_positions = list(msg.position[:5])

    def wait_for_server(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server ready!')

    def get_current_positions(self):
        """Wait for and return current joint positions"""
        self.get_logger().info('Getting current joint positions...')
        timeout = 5.0
        start_time = time.time()
        while self.current_positions is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_positions is None:
            self.get_logger().warn('Could not get current positions, using zeros')
            return [0.0] * 5

        self.get_logger().info(f'Current positions: {[f"{p:.3f}" for p in self.current_positions]}')
        return self.current_positions

    def get_pid_gains(self, joint_name):
        """Get current PID gains for a joint"""
        from rcl_interfaces.srv import GetParameters

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
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is not None:
            values = future.result().values
            return {
                'p': values[0].double_value,
                'i': values[1].double_value,
                'd': values[2].double_value,
                'i_clamp': values[3].double_value,
                'ff_velocity_scale': values[4].double_value,
            }
        return None

    def send_goal(self, goal_msg):
        self.get_logger().info('Sending trajectory...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False

        self.get_logger().info('Goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code == 0:
            self.get_logger().info('✓ Trajectory completed successfully')
            return True
        else:
            self.get_logger().error(f'✗ Trajectory failed with error code: {result.error_code}')
            return False

    def move_to_zero(self, duration=5.0):
        """Move all joints to zero position"""
        self.get_logger().info('Moving to zero position...')

        # Get current position to calculate distance
        current_pos = self.get_current_positions()

        # Calculate max distance to zero
        max_dist = max(abs(p) for p in current_pos)

        # Adjust duration based on distance (at least 2 seconds per radian)
        adjusted_duration = max(duration, max_dist * 2.0)

        if adjusted_duration > duration:
            self.get_logger().info(f'Adjusted duration to {adjusted_duration:.1f}s for safe movement')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [0.0] * 5
        point.velocities = [0.0] * 5
        point.time_from_start = Duration(sec=int(adjusted_duration), nanosec=int((adjusted_duration % 1) * 1e9))

        goal.trajectory.points = [point]

        success = self.send_goal(goal)
        if success:
            # Update current positions cache
            self.current_positions = [0.0] * 5
        return success

    def generate_step(self, joint_idx, amplitude, duration=2.0):
        """Generate step response test"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        # Get current positions
        current_pos = self.get_current_positions()

        # Point 1: Step to current + amplitude
        point1 = JointTrajectoryPoint()
        point1.positions = current_pos.copy()
        point1.positions[joint_idx] = current_pos[joint_idx] + amplitude
        point1.velocities = [0.0] * 5
        point1.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))

        goal.trajectory.points = [point1]
        return goal

    def generate_square_wave(self, joint_idx, amplitude, period=2.0, num_cycles=3):
        """Generate square wave for frequency response"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        # Get current positions
        current_pos = self.get_current_positions()
        center = current_pos[joint_idx]

        points = []
        half_period = period / 2.0

        for cycle in range(num_cycles):
            # High
            point_high = JointTrajectoryPoint()
            point_high.positions = current_pos.copy()
            point_high.positions[joint_idx] = center + amplitude
            point_high.velocities = [0.0] * 5
            t = cycle * period + half_period
            point_high.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            points.append(point_high)

            # Low
            point_low = JointTrajectoryPoint()
            point_low.positions = current_pos.copy()
            point_low.positions[joint_idx] = center - amplitude
            point_low.velocities = [0.0] * 5
            t = (cycle + 1) * period
            point_low.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            points.append(point_low)

        # Return to center
        point_center = JointTrajectoryPoint()
        point_center.positions = current_pos.copy()
        point_center.velocities = [0.0] * 5
        t = num_cycles * period + 1.0
        point_center.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        points.append(point_center)

        goal.trajectory.points = points
        return goal

    def generate_sine_wave(self, joint_idx, amplitude, frequency=0.5, duration=10.0):
        """Generate sine wave for smooth tracking test"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        # Get current positions
        current_pos = self.get_current_positions()
        center = current_pos[joint_idx]

        dt = 0.1  # 100ms time step
        num_points = int(duration / dt)

        points = []
        for i in range(num_points):
            t = i * dt
            point = JointTrajectoryPoint()
            point.positions = current_pos.copy()
            point.positions[joint_idx] = center + amplitude * np.sin(2 * np.pi * frequency * t)
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

        # Get current positions
        current_pos = self.get_current_positions()
        center = current_pos[joint_idx]

        dt = 0.05  # 50ms time step
        num_points = int(duration / dt)

        points = []
        for i in range(num_points):
            t = i * dt
            # Linear frequency sweep
            freq = freq_start + (freq_end - freq_start) * (t / duration)
            phase = 2 * np.pi * (freq_start * t + 0.5 * (freq_end - freq_start) * (t**2) / duration)

            point = JointTrajectoryPoint()
            point.positions = current_pos.copy()
            point.positions[joint_idx] = center + amplitude * np.sin(phase)
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
    parser.add_argument('--reset', action='store_true',
                       help='Move to zero position before test (recommended for consistent testing)')

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

    # Reset to zero if requested
    if args.reset:
        print("\n🔄 Resetting to zero position first...")
        if not node.move_to_zero(duration=3.0):
            print("✗ Failed to move to zero position")
            node.destroy_node()
            rclpy.shutdown()
            return
        print("✓ Ready at zero position\n")
        # Wait a moment for robot to settle
        import time
        time.sleep(0.5)

    # Get and display current PID gains
    joint_name = joint_names_map[args.joint]
    gains = node.get_pid_gains(joint_name)

    print(f"\n🎯 Generating {args.type} trajectory:")
    print(f"   Joint: {joint_name} (index {args.joint})")
    print(f"   Amplitude: {args.amplitude} rad")

    if gains:
        print(f"\n⚙️  Current PID Gains for {joint_name}:")
        print(f"   P = {gains['p']:.1f}")
        print(f"   I = {gains['i']:.1f}")
        print(f"   D = {gains['d']:.1f}")
        print(f"   I Clamp = {gains['i_clamp']:.1f}")
        print(f"   FF Velocity = {gains['ff_velocity_scale']:.2f}")
    else:
        print(f"\n⚠️  Could not retrieve PID gains")

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