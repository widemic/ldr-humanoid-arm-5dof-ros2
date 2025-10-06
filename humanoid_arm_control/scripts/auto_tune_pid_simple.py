#!/usr/bin/env python3
"""
Simple PID Auto-Tuning: Start high and reduce until stable

Usage:
    ros2 run humanoid_arm_control auto_tune_pid_simple.py --joint base_rotation_joint
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
import numpy as np
import time
import argparse
import threading


class SimplePIDTuner(Node):
    def __init__(self, joint_name):
        super().__init__('simple_pid_tuner')

        self.joint_name = joint_name
        self.controller_name = f'{joint_name}_position_controller'

        joint_map = {
            'base_rotation_joint': 0,
            'shoulder_pitch_joint': 1,
            'elbow_pitch_joint': 2,
            'wrist_pitch_joint': 3,
            'wrist_roll_joint': 4
        }
        self.joint_index = joint_map.get(joint_name, 0)

        self.positions = []
        self.timestamps = []
        self.current_position = 0.0
        self.lock = threading.Lock()

        self.cmd_pub = self.create_publisher(
            JointTrajectory,
            f'/{self.controller_name}/joint_trajectory',
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.set_param_client = self.create_client(
            SetParameters,
            f'/{self.controller_name}/set_parameters'
        )

        self.get_logger().info(f'Simple PID tuner ready for {joint_name}')

    def joint_callback(self, msg):
        if self.joint_index < len(msg.position):
            with self.lock:
                self.current_position = msg.position[self.joint_index]

    def send_command(self, position, duration=0.5):
        msg = JointTrajectory()
        msg.joint_names = [self.joint_name]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points = [point]
        self.cmd_pub.publish(msg)

    def set_gains(self, p, i, d, i_clamp=25.0, ff_velocity_scale=0.12):
        params = []
        for name, val in [('p', p), ('i', i), ('d', d), ('i_clamp', i_clamp), ('ff_velocity_scale', ff_velocity_scale)]:
            param = Parameter()
            param.name = f'gains.{self.joint_name}.{name}'
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(val))
            params.append(param)

        req = SetParameters.Request()
        req.parameters = params
        future = self.set_param_client.call_async(req)

        timeout = 2.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.01)

        if future.done() and future.result():
            self.get_logger().info(f'Set gains: P={p:.1f}, I={i:.1f}, D={d:.1f}')
            return True
        return False

    def test_step(self, target=0.5, duration=6.0):
        """Test step response and analyze performance"""
        self.positions.clear()
        self.timestamps.clear()

        self.send_command(target, duration=1.5)

        start_time = time.time()
        while (time.time() - start_time) < duration:
            elapsed = time.time() - start_time
            with self.lock:
                self.positions.append(self.current_position)
                self.timestamps.append(elapsed)
            time.sleep(0.02)

        pos = np.array(self.positions)
        times = np.array(self.timestamps)

        if len(pos) < 50:
            return None

        # Calculate metrics
        overshoot = ((np.max(pos) - target) / abs(target)) * 100 if target != 0 else 0
        final_error = abs(pos[-20:].mean() - target)

        # Detect significant oscillations (count zero crossings with threshold to ignore noise)
        error = pos - target
        threshold = 0.05  # Only count crossings with amplitude > 5cm (0.05 rad)

        # Find peaks and valleys in error signal
        significant_oscillations = 0
        for i in range(1, len(error) - 1):
            # Peak (local maximum)
            if error[i] > error[i-1] and error[i] > error[i+1] and abs(error[i]) > threshold:
                significant_oscillations += 0.5
            # Valley (local minimum)
            if error[i] < error[i-1] and error[i] < error[i+1] and abs(error[i]) > threshold:
                significant_oscillations += 0.5

        oscillation_count = significant_oscillations

        # Rise time (10% to 90%)
        rise_start_idx = np.where(np.abs(pos - 0) > 0.1 * abs(target))[0]
        rise_end_idx = np.where(np.abs(pos - target) < 0.1 * abs(target))[0]
        rise_time = times[rise_end_idx[0]] - times[rise_start_idx[0]] if len(rise_start_idx) > 0 and len(rise_end_idx) > 0 else 999

        return {
            'overshoot': overshoot,
            'final_error': final_error,
            'oscillations': oscillation_count,
            'rise_time': rise_time,
            'max_pos': np.max(pos),
            'min_pos': np.min(pos)
        }

    def run(self):
        """Iterative tuning: start high, reduce until stable"""
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'SIMPLE AUTO-TUNE: {self.joint_name}')
        self.get_logger().info(f'Strategy: Start with high gains, reduce until stable')
        self.get_logger().info(f'{"="*60}\n')

        time.sleep(2.0)

        # Phase 1: Find good P gain (start LOW, go HIGH until unstable)
        self.get_logger().info('=== PHASE 1: Finding Proportional Gain ===')
        self.get_logger().info('Strategy: Start low, increase until we find the highest stable P\n')

        # Start higher for heavy arm with effort control
        p_values = [200, 300, 400, 500, 600, 700, 800, 900, 1000]
        best_p = 200

        for p in p_values:
            self.get_logger().info(f'\nTesting P={p}, I=0, D=0')
            self.set_gains(p, 0, 0)
            time.sleep(1.0)

            result = self.test_step(0.5)
            if result is None:
                continue

            self.get_logger().info(f'  Overshoot: {result["overshoot"]:.1f}%')
            self.get_logger().info(f'  Final error: {result["final_error"]:.4f}rad')
            self.get_logger().info(f'  Oscillations: {result["oscillations"]:.0f}')
            self.get_logger().info(f'  Rise time: {result["rise_time"]:.2f}s')

            # Stop when we hit too many oscillations (found the limit)
            if result['oscillations'] >= 8:
                self.get_logger().info(f'  ✗ P={p} is too high ({result["oscillations"]:.0f} oscillations)')
                self.get_logger().info(f'  Using previous stable value: P={best_p}')
                break
            else:
                # This P is good, keep it and try higher
                best_p = p
                self.get_logger().info(f'  ✓ P={p} is stable, trying higher...')
                if p == p_values[-1]:
                    self.get_logger().info(f'  Reached maximum test value!')

        self.get_logger().info(f'\n>>> Selected P = {best_p} <<<')

        # Phase 2: Add D for damping (try higher D values first for heavy arm)
        self.get_logger().info(f'\n=== PHASE 2: Adding Derivative Gain ===')

        d_values = [best_p/3, best_p/4, best_p/5, best_p/6, best_p/8, best_p/10, best_p/15]
        best_d = best_p / 8

        for d in d_values:
            self.get_logger().info(f'\nTesting P={best_p}, I=0, D={d:.1f}')
            self.set_gains(best_p, 0, d)
            time.sleep(1.0)

            result = self.test_step(0.5)
            if result is None:
                continue

            self.get_logger().info(f'  Overshoot: {result["overshoot"]:.1f}%')
            self.get_logger().info(f'  Oscillations: {result["oscillations"]:.0f}')
            self.get_logger().info(f'  Final error: {result["final_error"]:.4f}rad')

            # Look for low oscillations AND low final error (trembling shows as steady-state error)
            if result['oscillations'] < 2 and abs(result['overshoot']) < 30 and result['final_error'] < 0.05:
                best_d = d
                self.get_logger().info(f'✓ Good D gain found: {d:.1f} (stable, no trembling)')
                break
            elif result['final_error'] > 0.04:
                # Still trembling/oscillating, need more D
                self.get_logger().info(f'  Still trembling (error {result["final_error"]:.4f}), trying higher D')
                best_d = d
                continue
            else:
                best_d = d

        # Phase 3: Add small I for steady-state error
        self.get_logger().info(f'\n=== PHASE 3: Adding Integral Gain ===')

        i_values = [best_p/50, best_p/75, best_p/100, 0]
        best_i = 0

        for i in i_values:
            self.get_logger().info(f'\nTesting P={best_p}, I={i:.2f}, D={best_d:.1f}')
            self.set_gains(best_p, i, best_d)
            time.sleep(1.0)

            result = self.test_step(0.5)
            if result is None:
                continue

            self.get_logger().info(f'  Final error: {result["final_error"]:.4f}rad')
            self.get_logger().info(f'  Oscillations: {result["oscillations"]:.0f}')

            if result['final_error'] < 0.08 and result['oscillations'] < 4:
                best_i = i
                self.get_logger().info(f'✓ Good I gain found: {i:.2f}')
                break

        # Final test
        self.get_logger().info(f'\n=== FINAL TEST ===')
        self.set_gains(best_p, best_i, best_d)
        time.sleep(1.0)

        result = self.test_step(0.5)

        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'✓ AUTO-TUNE COMPLETE!')
        self.get_logger().info(f'\nFinal PID Gains for {self.joint_name}:')
        self.get_logger().info(f'  p: {best_p:.1f}')
        self.get_logger().info(f'  i: {best_i:.2f}')
        self.get_logger().info(f'  d: {best_d:.1f}')
        self.get_logger().info(f'  i_clamp: 25.0')
        self.get_logger().info(f'  ff_velocity_scale: 0.12')

        if result:
            self.get_logger().info(f'\nPerformance:')
            self.get_logger().info(f'  Overshoot: {result["overshoot"]:.1f}%')
            self.get_logger().info(f'  Final error: {result["final_error"]:.4f}rad')
            self.get_logger().info(f'  Rise time: {result["rise_time"]:.2f}s')
            self.get_logger().info(f'  Oscillations: {result["oscillations"]:.0f}')

        self.get_logger().info(f'\nUpdate controllers.yaml with these values.')
        self.get_logger().info(f'{"="*60}\n')

        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--joint', default='base_rotation_joint')
    args = parser.parse_args()

    rclpy.init()
    node = SimplePIDTuner(args.joint)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
