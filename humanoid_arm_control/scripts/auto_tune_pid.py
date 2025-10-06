#!/usr/bin/env python3
"""
Automatic PID Tuning using Relay Feedback Method

Usage:
    ros2 run humanoid_arm_control auto_tune_pid.py --joint base_rotation_joint
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


class PIDAutoTuner(Node):
    def __init__(self, joint_name):
        super().__init__('pid_auto_tuner')

        self.joint_name = joint_name
        self.controller_name = f'{joint_name}_position_controller'

        # Joint index
        joint_map = {
            'base_rotation_joint': 0,
            'shoulder_pitch_joint': 1,
            'elbow_pitch_joint': 2,
            'wrist_pitch_joint': 3,
            'wrist_roll_joint': 4
        }
        self.joint_index = joint_map.get(joint_name, 0)

        # Parameters
        self.relay_amplitude = 0.3
        self.setpoint = 0.0

        # Data collection
        self.positions = []
        self.timestamps = []
        self.current_position = 0.0
        self.lock = threading.Lock()

        # Publishers/Subscribers
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

        # Service client
        self.set_param_client = self.create_client(
            SetParameters,
            f'/{self.controller_name}/set_parameters'
        )

        self.get_logger().info(f'Auto-tuner ready for {joint_name}')

    def joint_callback(self, msg):
        """Update current position from joint states"""
        if self.joint_index < len(msg.position):
            with self.lock:
                self.current_position = msg.position[self.joint_index]

    def send_command(self, position, duration=0.5):
        """Send position command to controller"""
        msg = JointTrajectory()
        msg.joint_names = [self.joint_name]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points = [point]
        self.cmd_pub.publish(msg)

    def set_gains(self, p, i, d, i_clamp=20.0, ff_velocity_scale=0.1):
        """Set PID gains via service call"""
        params = []

        for name, val in [('p', p), ('i', i), ('d', d), ('i_clamp', i_clamp), ('ff_velocity_scale', ff_velocity_scale)]:
            param = Parameter()
            param.name = f'gains.{self.joint_name}.{name}'
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(val))
            params.append(param)

        req = SetParameters.Request()
        req.parameters = params
        future = self.set_param_client.call_async(req)

        # Non-blocking wait
        timeout = 2.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.01)

        if future.done() and future.result():
            self.get_logger().info(f'Gains set: P={p:.2f}, I={i:.2f}, D={d:.2f}')
            return True

        self.get_logger().error('Failed to set gains')
        return False

    def collect_data(self, duration):
        """Collect position data for specified duration"""
        self.positions.clear()
        self.timestamps.clear()

        start_time = time.time()

        while (time.time() - start_time) < duration:
            with self.lock:
                self.positions.append(self.current_position)
                self.timestamps.append(time.time() - start_time)
            time.sleep(0.02)  # 50 Hz sampling

        self.get_logger().info(f'Collected {len(self.positions)} samples over {duration}s')

    def run_relay_test(self, duration=30.0):
        """Run relay feedback test"""
        self.get_logger().info('=== RELAY TEST ===')
        self.get_logger().info('Setting P=200, I=0, D=0 for relay feedback')

        if not self.set_gains(200.0, 0.0, 0.0):
            return False

        time.sleep(2.0)

        # Move to setpoint
        self.get_logger().info(f'Moving to setpoint {self.setpoint}rad...')
        self.send_command(self.setpoint, duration=2.0)
        time.sleep(3.0)

        # Start relay oscillations
        self.get_logger().info('Starting relay oscillations...')

        start_time = time.time()
        self.positions.clear()
        self.timestamps.clear()

        while (time.time() - start_time) < duration:
            elapsed = time.time() - start_time

            # Get current position
            with self.lock:
                pos = self.current_position
                self.positions.append(pos)
                self.timestamps.append(elapsed)

            # Relay control
            error = self.setpoint - pos
            relay_offset = self.relay_amplitude if error > 0 else -self.relay_amplitude
            target = self.setpoint + relay_offset
            self.send_command(target, duration=0.1)

            # Status update every 2 seconds
            if int(elapsed) % 2 == 0 and len(self.positions) > 0:
                if elapsed - int(elapsed) < 0.1:  # Only print once per interval
                    self.get_logger().info(f'[{elapsed:.1f}s] Pos={pos:.4f}, Target={target:.4f}, Samples={len(self.positions)}')

            # Check for oscillations after 10 seconds
            if elapsed > 10.0 and int(elapsed) % 3 == 0:
                if elapsed - int(elapsed) < 0.1:
                    result = self.detect_oscillation()
                    if result is not None:
                        period, amplitude = result
                        if elapsed > 20.0:  # Have enough data
                            self.get_logger().info(f'✓ Oscillation detected after {elapsed:.1f}s')
                            return period, amplitude

            time.sleep(0.02)  # 50 Hz

        # Final check
        result = self.detect_oscillation()
        if result is not None:
            return result

        self.get_logger().error('Failed to detect oscillations')
        return None

    def detect_oscillation(self):
        """Detect oscillation period and amplitude"""
        if len(self.positions) < 200:
            return None

        pos = np.array(self.positions)
        times = np.array(self.timestamps)

        # Center around mean
        pos_centered = pos - np.mean(pos)

        # Simple peak detection
        peaks = []
        troughs = []

        for i in range(5, len(pos) - 5):
            # Peak
            if all(pos_centered[i] > pos_centered[i-j] for j in range(1, 6)) and \
               all(pos_centered[i] > pos_centered[i+j] for j in range(1, 6)):
                if abs(pos_centered[i]) > 0.015:
                    peaks.append((times[i], pos_centered[i]))

            # Trough
            if all(pos_centered[i] < pos_centered[i-j] for j in range(1, 6)) and \
               all(pos_centered[i] < pos_centered[i+j] for j in range(1, 6)):
                if abs(pos_centered[i]) > 0.015:
                    troughs.append((times[i], pos_centered[i]))

        if len(peaks) < 3 or len(troughs) < 3:
            self.get_logger().info(f'Not enough peaks ({len(peaks)}) or troughs ({len(troughs)})')
            return None

        # Calculate period from peaks
        periods = [peaks[i][0] - peaks[i-1][0] for i in range(1, len(peaks))]
        avg_period = np.mean(periods)
        std_period = np.std(periods)

        # Calculate amplitude
        peak_vals = [p[1] for p in peaks]
        trough_vals = [t[1] for t in troughs]
        amplitude = (max(peak_vals) - min(trough_vals)) / 2.0

        # Check consistency
        if std_period < 0.4 * avg_period and amplitude > 0.02:
            self.get_logger().info(f'✓ Valid oscillation: Period={avg_period:.3f}s, Amplitude={amplitude:.4f}rad')
            return avg_period, amplitude

        self.get_logger().info(f'Oscillation not stable: Period {avg_period:.3f}±{std_period:.3f}s, Amp={amplitude:.4f}')
        return None

    def calculate_gains(self, period, amplitude):
        """Calculate PID gains using modified Ziegler-Nichols for effort control"""
        Tu = period
        a = amplitude
        d = self.relay_amplitude

        # Ultimate gain from relay test
        Ku = (4 * d) / (np.pi * a)

        self.get_logger().info(f'Ultimate parameters: Ku={Ku:.2f}, Tu={Tu:.3f}s')

        # Aggressive tuning for heavy arm with effort control
        # Standard Z-N: Kp=0.6*Ku, Ki=1.2*Ku/Tu, Kd=0.075*Ku*Tu
        # For heavy arm with effort control, we need MUCH higher gains
        Kp = 1.2 * Ku  # Very aggressive proportional for heavy load
        Ki = 0.3 * Ku / Tu  # Low integral to prevent windup
        Kd = 0.25 * Ku * Tu  # High damping for stability

        self.get_logger().info(f'Using aggressive tuning for heavy arm + effort control')
        self.get_logger().info(f'  Kp = 1.2 * Ku = {Kp:.2f}')
        self.get_logger().info(f'  Ki = 0.3 * Ku / Tu = {Ki:.2f}')
        self.get_logger().info(f'  Kd = 0.25 * Ku * Tu = {Kd:.2f}')

        return {
            'p': Kp,
            'i': Ki,
            'd': Kd,
            'i_clamp': max(20.0, Ki * 5),  # Higher clamp for lower Ki
            'ff_velocity_scale': 0.12  # Increased feedforward
        }

    def test_step_response(self, target=0.5, duration=8.0):
        """Test step response"""
        self.get_logger().info(f'\n=== TESTING STEP RESPONSE to {target}rad ===')

        self.positions.clear()
        self.timestamps.clear()

        # Send step command
        self.send_command(target, duration=2.0)

        # Collect data
        start_time = time.time()
        while (time.time() - start_time) < duration:
            elapsed = time.time() - start_time
            with self.lock:
                self.positions.append(self.current_position)
                self.timestamps.append(elapsed)
            time.sleep(0.02)

        # Analyze
        pos = np.array(self.positions)
        if len(pos) > 50:
            overshoot = ((np.max(pos) - target) / abs(target)) * 100 if target != 0 else 0
            final_error = abs(pos[-20:].mean() - target)
            settling_tolerance = 0.05  # 5% of target
            settled_indices = np.where(np.abs(pos - target) < settling_tolerance)[0]
            settling_time = self.timestamps[settled_indices[0]] if len(settled_indices) > 0 else 999

            self.get_logger().info(f'Results:')
            self.get_logger().info(f'  Overshoot: {overshoot:.1f}%')
            self.get_logger().info(f'  Final error: {final_error:.4f}rad ({final_error/abs(target)*100:.1f}%)')
            self.get_logger().info(f'  Settling time (±5%): {settling_time:.2f}s')

            # Success criteria for effort control (more relaxed than position control)
            success = abs(overshoot) < 50 and final_error < 0.15
            return success

        return False

    def run(self):
        """Main auto-tuning sequence"""
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'AUTO-TUNE STARTING: {self.joint_name}')
        self.get_logger().info(f'{"="*60}\n')

        time.sleep(2.0)

        # Step 1: Relay test
        result = self.run_relay_test(duration=35.0)
        if result is None:
            self.get_logger().error('❌ Relay test failed')
            return False

        period, amplitude = result

        # Step 2: Calculate gains
        gains = self.calculate_gains(period, amplitude)
        self.get_logger().info(f'\nCalculated PID gains:')
        self.get_logger().info(f'  P = {gains["p"]:.2f}')
        self.get_logger().info(f'  I = {gains["i"]:.2f}')
        self.get_logger().info(f'  D = {gains["d"]:.2f}')

        # Step 3: Apply gains
        if not self.set_gains(**gains):
            return False

        time.sleep(2.0)

        # Step 4: Test
        success = self.test_step_response(0.5)

        # Final report
        self.get_logger().info(f'\n{"="*60}')
        if success:
            self.get_logger().info(f'✓ AUTO-TUNE COMPLETE!')
        else:
            self.get_logger().info(f'⚠ AUTO-TUNE COMPLETE (may need manual adjustment)')

        self.get_logger().info(f'\nFinal PID Gains for {self.joint_name}:')
        self.get_logger().info(f'  p: {gains["p"]:.2f}')
        self.get_logger().info(f'  i: {gains["i"]:.2f}')
        self.get_logger().info(f'  d: {gains["d"]:.2f}')
        self.get_logger().info(f'  i_clamp: {gains["i_clamp"]:.2f}')
        self.get_logger().info(f'  ff_velocity_scale: {gains["ff_velocity_scale"]:.2f}')
        self.get_logger().info(f'\nUpdate controllers.yaml with these values.')
        self.get_logger().info(f'{"="*60}\n')

        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--joint', default='base_rotation_joint')
    args = parser.parse_args()

    rclpy.init()
    node = PIDAutoTuner(args.joint)

    # Spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
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
