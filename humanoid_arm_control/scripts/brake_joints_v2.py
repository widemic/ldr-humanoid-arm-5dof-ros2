#!/usr/bin/env python3
"""
Virtual Brake Control via Effort Commands (ROS2 Control Method)

This version works by subscribing to joint states and publishing effort commands
to hold joints at their current position with high gains (simulates brake).

Works with both Gazebo and real hardware (uses ros2_control interfaces).

Usage:
    # Brake all joints EXCEPT base_rotation (for tuning base_rotation)
    ros2 run humanoid_arm_control brake_joints_v2.py --free base_rotation_joint

    # Release all brakes
    ros2 run humanoid_arm_control brake_joints_v2.py --release
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import argparse
import sys


class BrakeController(Node):
    """Virtual brake controller using effort commands"""

    JOINTS = [
        'base_rotation_joint',
        'shoulder_pitch_joint',
        'elbow_pitch_joint',
        'wrist_pitch_joint',
        'wrist_roll_joint'
    ]

    # Brake PD gains (high to simulate locked brake)
    BRAKE_GAINS = {
        'base_rotation_joint': {'p': 2000.0, 'd': 200.0},
        'shoulder_pitch_joint': {'p': 2500.0, 'd': 250.0},  # Highest (gravity)
        'elbow_pitch_joint': {'p': 1500.0, 'd': 150.0},
        'wrist_pitch_joint': {'p': 1000.0, 'd': 100.0},
        'wrist_roll_joint': {'p': 500.0, 'd': 50.0},
    }

    # Motor torque limits (from actuator_specs.yaml)
    TORQUE_LIMITS = {
        'base_rotation_joint': 120.0,
        'shoulder_pitch_joint': 120.0,
        'elbow_pitch_joint': 60.0,
        'wrist_pitch_joint': 60.0,
        'wrist_roll_joint': 17.0,
    }

    def __init__(self, free_joints=None):
        super().__init__('brake_controller')

        self.free_joints = free_joints if free_joints else []
        self.braked_joints = [j for j in self.JOINTS if j not in self.free_joints]

        self.get_logger().info('Virtual Brake Controller (ROS2 Control Method)')
        self.get_logger().info(f'  Free joints: {self.free_joints}')
        self.get_logger().info(f'  Braked joints: {self.braked_joints}')

        # Current state
        self.current_positions = None
        self.current_velocities = None
        self.brake_target_positions = None

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for effort commands
        # This publishes to the forward command controller
        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_effort_controller/commands',
            10
        )

        # Control loop at 100 Hz (matches controller manager)
        self.timer = self.create_timer(0.01, self.control_loop)

        self.get_logger().info('Brake controller ready!')
        self.get_logger().info('  Publishing to: /forward_effort_controller/commands')
        self.get_logger().info('  Make sure forward_effort_controller is loaded and active!')

    def joint_state_callback(self, msg):
        """Store current joint states"""
        positions = []
        velocities = []

        for joint_name in self.JOINTS:
            try:
                idx = msg.name.index(joint_name)
                positions.append(msg.position[idx])
                velocities.append(msg.velocity[idx])
            except (ValueError, IndexError):
                positions.append(0.0)
                velocities.append(0.0)

        self.current_positions = positions
        self.current_velocities = velocities

        # Lock target positions on first callback
        if self.brake_target_positions is None and len(self.braked_joints) > 0:
            self.brake_target_positions = positions.copy()
            self.get_logger().info(f'Brake locked at: {[f"{p:.3f}" for p in positions]}')

    def control_loop(self):
        """Calculate and publish brake efforts"""
        if self.current_positions is None or self.current_velocities is None:
            return

        if self.brake_target_positions is None:
            return

        efforts = []

        for i, joint_name in enumerate(self.JOINTS):
            if joint_name in self.braked_joints:
                # Apply brake: PD controller with high gains
                gains = self.BRAKE_GAINS[joint_name]
                p_gain = gains['p']
                d_gain = gains['d']

                pos_error = self.brake_target_positions[i] - self.current_positions[i]
                velocity = self.current_velocities[i]

                # Brake effort = P * error - D * velocity
                effort = p_gain * pos_error - d_gain * velocity

                # Clamp to motor limits
                max_torque = self.TORQUE_LIMITS[joint_name]
                effort = max(-max_torque, min(max_torque, effort))

                efforts.append(effort)
            else:
                # Free joint - zero effort (controller will handle it)
                efforts.append(0.0)

        # Publish efforts
        msg = Float64MultiArray()
        msg.data = efforts
        self.effort_pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(
        description='Virtual brake controller using ROS2 control',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('--free', nargs='+', metavar='JOINT',
                       help='Joint(s) to leave free for tuning (brake all others)')
    parser.add_argument('--release', action='store_true',
                       help='Release all brakes (no braking)')

    args = parser.parse_args()

    if not (args.free or args.release):
        parser.print_help()
        sys.exit(1)

    rclpy.init()

    if args.release:
        print("Releasing brakes (node not started)")
        sys.exit(0)

    # Validate joint names
    if args.free:
        invalid = [j for j in args.free if j not in BrakeController.JOINTS]
        if invalid:
            print(f"Error: Invalid joint names: {invalid}")
            print(f"Valid joints: {BrakeController.JOINTS}")
            sys.exit(1)

    node = BrakeController(free_joints=args.free)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
