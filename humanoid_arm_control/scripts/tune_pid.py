#!/usr/bin/env python3
"""
Simple PID tuning script for joint_trajectory_controller
Usage: ros2 run humanoid_arm_control tune_pid.py --joint base_rotation_joint --p 200 --d 10 --i 5
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
import argparse


class PIDTuner(Node):
    def __init__(self):
        super().__init__('pid_tuner')
        self.client = self.create_client(
            SetParameters,
            '/joint_trajectory_controller/set_parameters'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for parameter service...')

    def set_gains(self, joint_name, p=None, d=None, i=None, i_clamp=None, ff_vel=None):
        """Set PID gains for a specific joint"""
        params = []

        if p is not None:
            param = Parameter()
            param.name = f'gains.{joint_name}.p'
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(p))
            params.append(param)

        if d is not None:
            param = Parameter()
            param.name = f'gains.{joint_name}.d'
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(d))
            params.append(param)

        if i is not None:
            param = Parameter()
            param.name = f'gains.{joint_name}.i'
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(i))
            params.append(param)

        if i_clamp is not None:
            param = Parameter()
            param.name = f'gains.{joint_name}.i_clamp'
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(i_clamp))
            params.append(param)

        if ff_vel is not None:
            param = Parameter()
            param.name = f'gains.{joint_name}.ff_velocity_scale'
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(ff_vel))
            params.append(param)

        request = SetParameters.Request()
        request.parameters = params

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        for r in result.results:
            if r.successful:
                self.get_logger().info(f'✓ Parameter set successfully')
            else:
                self.get_logger().error(f'✗ Failed: {r.reason}')


def main():
    parser = argparse.ArgumentParser(description='Tune PID gains for joint_trajectory_controller')
    parser.add_argument('--joint', type=str, required=True,
                       choices=['base_rotation_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint',
                               'wrist_pitch_joint', 'wrist_roll_joint'],
                       help='Joint name to tune')
    parser.add_argument('--p', type=float, help='Proportional gain')
    parser.add_argument('--d', type=float, help='Derivative gain')
    parser.add_argument('--i', type=float, help='Integral gain')
    parser.add_argument('--i_clamp', type=float, help='Integral clamp')
    parser.add_argument('--ff_vel', type=float, help='Feedforward velocity scale')

    args = parser.parse_args()

    rclpy.init()
    tuner = PIDTuner()

    print(f"\n🎛️  Tuning {args.joint}")
    if args.p: print(f"   P = {args.p}")
    if args.d: print(f"   D = {args.d}")
    if args.i: print(f"   I = {args.i}")
    if args.i_clamp: print(f"   I_clamp = {args.i_clamp}")
    if args.ff_vel: print(f"   FF_vel = {args.ff_vel}")
    print()

    tuner.set_gains(args.joint, args.p, args.d, args.i, args.i_clamp, args.ff_vel)

    tuner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()