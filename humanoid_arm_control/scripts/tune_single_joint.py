#!/usr/bin/env python3
"""
Tune a single joint while holding all others still with high damping
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType
import argparse


class SingleJointTuner(Node):
    def __init__(self):
        super().__init__('single_joint_tuner')

        self.joint_names = [
            'base_rotation_joint',
            'shoulder_pitch_joint',
            'elbow_pitch_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint'
        ]

        self.set_param_client = self.create_client(
            SetParameters,
            '/joint_trajectory_controller/set_parameters'
        )

    def set_pid(self, joint_name, p, i, d):
        """Set PID gains for a joint"""
        params = []

        param = Parameter()
        param.name = f'gains.{joint_name}.p'
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(p)
        params.append(param)

        param = Parameter()
        param.name = f'gains.{joint_name}.i'
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(i)
        params.append(param)

        param = Parameter()
        param.name = f'gains.{joint_name}.d'
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(d)
        params.append(param)

        request = SetParameters.Request()
        request.parameters = params

        future = self.set_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        return future.done()

    def lock_all_except(self, active_joint_idx, active_p=50, active_d=10):
        """
        Lock all joints with high damping except the active one
        Active joint gets tunable gains, others get stiff locking gains
        """
        self.get_logger().info('🔒 Locking all joints except target...')

        for idx, joint_name in enumerate(self.joint_names):
            if idx == active_joint_idx:
                # Active joint - set to tunable gains
                self.set_pid(joint_name, p=active_p, i=0, d=active_d)
                self.get_logger().info(f'  ✓ {joint_name}: P={active_p}, D={active_d} (ACTIVE)')
            else:
                # Lock with very high D, moderate P
                self.set_pid(joint_name, p=30, i=0, d=50)
                self.get_logger().info(f'  🔒 {joint_name}: P=30, D=50 (LOCKED)')

        self.get_logger().info('')
        self.get_logger().info(f'✅ Ready to tune {self.joint_names[active_joint_idx]}')
        self.get_logger().info('   Use simple_tuner_gui.py or test_trajectory.py to tune')


def main():
    parser = argparse.ArgumentParser(description='Prepare for single joint tuning')
    parser.add_argument('--joint', type=int, required=True, choices=[0, 1, 2, 3, 4],
                       help='Joint to tune (0=base, 1=shoulder, 2=elbow, 3=wrist_pitch, 4=wrist_roll)')
    parser.add_argument('--p', type=float, default=50,
                       help='Initial P gain for active joint (default: 50)')
    parser.add_argument('--d', type=float, default=10,
                       help='Initial D gain for active joint (default: 10)')

    args = parser.parse_args()

    rclpy.init()
    tuner = SingleJointTuner()

    try:
        tuner.lock_all_except(args.joint, active_p=args.p, active_d=args.d)
    except KeyboardInterrupt:
        pass
    finally:
        tuner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
