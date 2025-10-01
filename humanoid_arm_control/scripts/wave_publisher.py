#!/usr/bin/env python3
"""
Continuous wave publisher for PID tuning
Publishes step, sine, square, or triangle waves indefinitely
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


class WavePublisher(Node):
    def __init__(self, joint_idx, wave_type, amplitude, period):
        super().__init__('wave_publisher')

        self.joint_names = [
            'base_rotation_joint',
            'shoulder_pitch_joint',
            'elbow_pitch_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint'
        ]

        self.joint_idx = joint_idx
        self.wave_type = wave_type
        self.amplitude = amplitude
        self.period = period

        self.current_positions = None

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Action client
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info(f'📡 Wave Publisher started')
        self.get_logger().info(f'   Joint: {self.joint_names[joint_idx]}')
        self.get_logger().info(f'   Wave: {wave_type}')
        self.get_logger().info(f'   Amplitude: {amplitude} rad')
        self.get_logger().info(f'   Period: {period} s')

    def joint_state_callback(self, msg):
        self.current_positions = list(msg.position[:5])

    def generate_wave(self, start_pos, cycles=10):
        """Generate wave trajectory - wave on target joint, others stay still"""
        goal = FollowJointTrajectory.Goal()
        # Must include all joints for controller to accept
        goal.trajectory.joint_names = self.joint_names

        duration = self.period * cycles
        points_per_cycle = 50  # Smooth tracking for sine/triangle

        if self.wave_type == 'square':
            # Square wave with fast transitions and holds
            # Use 5 points per transition for smooth tracking
            transition_time = 0.2  # 200ms transition time

            for cycle in range(cycles):
                cycle_start = cycle * self.period

                # Transition to high (5 points over transition_time)
                for i in range(5):
                    t = cycle_start + i * transition_time / 4
                    point = JointTrajectoryPoint()
                    point.positions = start_pos.copy()
                    point.positions[self.joint_idx] = start_pos[self.joint_idx] + self.amplitude
                    point.velocities = [0.0] * 5
                    point.accelerations = [0.0] * 5
                    point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                    goal.trajectory.points.append(point)

                # Hold high until half period
                t_hold_high = cycle_start + self.period / 2 - transition_time
                point_hold = JointTrajectoryPoint()
                point_hold.positions = start_pos.copy()
                point_hold.positions[self.joint_idx] = start_pos[self.joint_idx] + self.amplitude
                point_hold.velocities = [0.0] * 5
                point_hold.accelerations = [0.0] * 5
                point_hold.time_from_start = Duration(sec=int(t_hold_high), nanosec=int((t_hold_high % 1) * 1e9))
                goal.trajectory.points.append(point_hold)

                # Transition to low (5 points over transition_time)
                for i in range(5):
                    t = cycle_start + self.period / 2 + i * transition_time / 4
                    point = JointTrajectoryPoint()
                    point.positions = start_pos.copy()
                    point.positions[self.joint_idx] = start_pos[self.joint_idx] - self.amplitude
                    point.velocities = [0.0] * 5
                    point.accelerations = [0.0] * 5
                    point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                    goal.trajectory.points.append(point)

                # Hold low until end of cycle (if not last cycle)
                if cycle < cycles - 1:
                    t_hold_low = cycle_start + self.period - transition_time
                    point_hold = JointTrajectoryPoint()
                    point_hold.positions = start_pos.copy()
                    point_hold.positions[self.joint_idx] = start_pos[self.joint_idx] - self.amplitude
                    point_hold.velocities = [0.0] * 5
                    point_hold.accelerations = [0.0] * 5
                    point_hold.time_from_start = Duration(sec=int(t_hold_low), nanosec=int((t_hold_low % 1) * 1e9))
                    goal.trajectory.points.append(point_hold)

        elif self.wave_type == 'step':
            # Step wave: single step with smooth transition
            transition_time = 0.5  # 500ms transition

            # Start at zero
            point_start = JointTrajectoryPoint()
            point_start.positions = start_pos.copy()
            point_start.velocities = [0.0] * 5
            point_start.accelerations = [0.0] * 5
            point_start.time_from_start = Duration(sec=0, nanosec=0)
            goal.trajectory.points.append(point_start)

            # Hold at zero until near midpoint
            t_before = duration / 2 - transition_time
            point_before = JointTrajectoryPoint()
            point_before.positions = start_pos.copy()
            point_before.velocities = [0.0] * 5
            point_before.accelerations = [0.0] * 5
            point_before.time_from_start = Duration(sec=int(t_before), nanosec=int((t_before % 1) * 1e9))
            goal.trajectory.points.append(point_before)

            # Smooth transition to amplitude (5 points)
            for i in range(1, 6):
                t = duration / 2 - transition_time + i * transition_time / 5
                point = JointTrajectoryPoint()
                point.positions = start_pos.copy()
                # Linear ramp from 0 to amplitude
                point.positions[self.joint_idx] = start_pos[self.joint_idx] + self.amplitude * (i / 5)
                point.velocities = [0.0] * 5
                point.accelerations = [0.0] * 5
                point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                goal.trajectory.points.append(point)

            # Hold at amplitude until end
            t_end = duration
            point_end = JointTrajectoryPoint()
            point_end.positions = start_pos.copy()
            point_end.positions[self.joint_idx] = start_pos[self.joint_idx] + self.amplitude
            point_end.velocities = [0.0] * 5
            point_end.accelerations = [0.0] * 5
            point_end.time_from_start = Duration(sec=int(t_end), nanosec=int((t_end % 1) * 1e9))
            goal.trajectory.points.append(point_end)

        else:
            # Sine and triangle: many points for smooth curve
            total_points = points_per_cycle * cycles

            for i in range(total_points + 1):
                t = i * duration / total_points
                point = JointTrajectoryPoint()
                point.positions = start_pos.copy()

                # Calculate phase (0 to 2π per period)
                phase = (t / self.period) * 2 * np.pi

                if self.wave_type == 'sine':
                    value = self.amplitude * np.sin(phase)

                elif self.wave_type == 'triangle':
                    # Triangle wave: -1 to +1 linear ramps
                    # Normalize phase to 0-1 range
                    cycle_position = (phase % (2 * np.pi)) / (2 * np.pi)
                    if cycle_position < 0.25:
                        # Rising: 0 to 1
                        value = self.amplitude * (4 * cycle_position)
                    elif cycle_position < 0.75:
                        # Falling: 1 to -1
                        value = self.amplitude * (2 - 4 * cycle_position)
                    else:
                        # Rising: -1 to 0
                        value = self.amplitude * (-4 + 4 * cycle_position)
                else:
                    value = 0

                point.positions[self.joint_idx] = start_pos[self.joint_idx] + value
                point.velocities = [0.0] * 5
                point.accelerations = [0.0] * 5
                point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                goal.trajectory.points.append(point)

        return goal

    def publish_continuous(self):
        """Publish waves continuously"""
        self.get_logger().info('⏳ Waiting for current position...')

        # Wait for current position
        while self.current_positions is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        start_pos = self.current_positions.copy()
        self.get_logger().info(f'✓ Starting from position: {[f"{p:.3f}" for p in start_pos]}')

        self.action_client.wait_for_server()
        self.get_logger().info('✓ Action server connected')
        self.get_logger().info('')
        self.get_logger().info('🌊 Publishing continuous waves... (Ctrl+C to stop)')
        self.get_logger().info('')

        cycle_count = 0

        try:
            while rclpy.ok():
                cycle_count += 1
                self.get_logger().info(f'📤 Sending batch {cycle_count} (10 periods = {self.period * 10:.1f}s)...')

                # Generate and send trajectory
                goal = self.generate_wave(start_pos, cycles=10)
                future = self.action_client.send_goal_async(goal)

                # Wait for goal to be accepted
                while not future.done():
                    rclpy.spin_once(self, timeout_sec=0.01)

                goal_handle = future.result()
                if not goal_handle.accepted:
                    self.get_logger().warn('⚠ Goal rejected')
                    time.sleep(1)
                    continue

                # Wait for trajectory to complete
                result_future = goal_handle.get_result_async()

                # Spin while waiting, but check periodically
                start_time = time.time()
                timeout = self.period * 10 + 5.0  # 10 cycles + buffer

                while not result_future.done():
                    rclpy.spin_once(self, timeout_sec=0.01)
                    if time.time() - start_time > timeout:
                        self.get_logger().warn('⚠ Trajectory timed out, restarting...')
                        break

                if result_future.done():
                    result = result_future.result().result
                    error_code = result.error_code

                    if error_code == 0:
                        self.get_logger().info('✓ Cycle complete')
                    else:
                        self.get_logger().warn(f'⚠ Cycle finished with error: {error_code}')

                # Brief pause between cycles
                time.sleep(0.1)

        except KeyboardInterrupt:
            self.get_logger().info('')
            self.get_logger().info('🛑 Stopped by user')


def main():
    parser = argparse.ArgumentParser(description='Continuous wave publisher for PID tuning')
    parser.add_argument('--joint', type=int, required=True, choices=[0, 1, 2, 3, 4],
                       help='Joint index (0=base, 1=shoulder, 2=elbow, 3=wrist_pitch, 4=wrist_roll)')
    parser.add_argument('--wave', type=str, default='sine',
                       choices=['sine', 'square', 'triangle', 'step'],
                       help='Wave type (default: sine)')
    parser.add_argument('--amplitude', type=float, default=0.3,
                       help='Wave amplitude in radians (default: 0.3)')
    parser.add_argument('--period', type=float, default=2.0,
                       help='Wave period in seconds (default: 2.0)')

    args = parser.parse_args()

    rclpy.init()

    publisher = WavePublisher(args.joint, args.wave, args.amplitude, args.period)

    try:
        publisher.publish_continuous()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            publisher.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
