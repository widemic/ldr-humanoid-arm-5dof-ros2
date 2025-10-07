#!/usr/bin/env python3

"""
Simple Reach Target Script - Move robot arm using joint trajectory action client

This script sends joint trajectories directly to the joint_trajectory_controller
without requiring MoveIt2. Uses simple geometric IK approximations.

Usage:
    # Reach predefined sphere targets
    ros2 run humanoid_arm_control reach_target_simple.py --target sphere1
    ros2 run humanoid_arm_control reach_target_simple.py --target sphere2
    ros2 run humanoid_arm_control reach_target_simple.py --target sphere3

    # Reach all spheres in sequence
    ros2 run humanoid_arm_control reach_target_simple.py --target all

    # Home position
    ros2 run humanoid_arm_control reach_target_simple.py --target home

Predefined Targets:
    sphere1: (1.0, 0.5, 1.2)  - Upper right
    sphere2: (0.8, -0.3, 1.5) - Highest, left
    sphere3: (1.2, 0.0, 1.0)  - Center, lowest
    home: All joints to 0.0

Requirements:
    - Robot launched (any mode: robot.launch.py, gazebo.launch.py, or full_system.launch.py)
    - joint_trajectory_controller active
"""

import sys
import argparse
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# Predefined joint configurations (manually tuned for sphere positions)
# Format: [base_rotation, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll]
JOINT_TARGETS = {
    'home': [0.0, 0.0, 0.0, 0.0, 0.0],

    # Updated configurations for new sphere positions (closer to robot)
    'sphere1': [0.4, 0.8, -0.5, 0.2, 0.0],     # (0.6, 0.3, 0.8) - front right, mid
    'sphere2': [-0.5, 1.0, -0.7, 0.3, 0.0],    # (0.5, -0.3, 1.0) - front left, high
    'sphere3': [0.0, 0.6, -0.3, 0.1, 0.0],     # (0.7, 0.0, 0.6) - front center, low

    # Alternative configurations
    'reach_forward': [0.0, 1.2, -1.5, 0.5, 0.0],
    'reach_up': [0.0, 2.0, -1.0, 0.0, 0.0],

    # Small test movement
    'test_small': [0.0, 0.3, 0.0, 0.0, 0.0],
}

# Sphere positions for reference (FINAL - negative X, robot faces backwards)
SPHERE_POSITIONS = {
    'sphere1': (-0.6, 0.3, 0.7),   # Red - right
    'sphere2': (-0.5, -0.3, 0.9),  # Blue - left
    'sphere3': (-0.9, 0.0, 0.5),   # Green - center, further
}


class ReachTargetSimpleNode(Node):
    """Node to reach targets using action client"""

    def __init__(self):
        super().__init__('reach_target_simple_node')

        # Joint names
        self.joint_names = [
            'base_rotation_joint',
            'shoulder_pitch_joint',
            'elbow_pitch_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
        ]

        # Create action client
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info('ReachTargetSimpleNode initialized')
        self.get_logger().info('Waiting for action server...')

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            self.get_logger().error('Make sure joint_trajectory_controller is running')
            raise RuntimeError('Action server timeout')

        self.get_logger().info('Action server ready')

    def send_joint_trajectory(self, joint_positions, duration_sec=5.0):
        """
        Send joint trajectory goal

        Args:
            joint_positions: List of 5 joint positions [rad]
            duration_sec: Time to reach target

        Returns:
            bool: True if motion succeeded
        """
        if len(joint_positions) != 5:
            self.get_logger().error(f'Expected 5 joint positions, got {len(joint_positions)}')
            return False

        self.get_logger().info(f'Sending trajectory: {[f"{p:.3f}" for p in joint_positions]}')

        # Create goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(sec=int(duration_sec),
                                         nanosec=int((duration_sec % 1) * 1e9))

        goal_msg.trajectory.points = [point]

        # Send goal
        self.get_logger().info(f'Executing motion ({duration_sec}s)...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server')
            return False

        self.get_logger().info('Goal accepted, waiting for completion...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        error_code = result.error_code

        if error_code == 0:
            self.get_logger().info('Motion completed successfully!')
            return True
        else:
            self.get_logger().error(f'Motion failed with error code: {error_code}')
            return False

    def reach_named_target(self, target_name, duration_sec=5.0):
        """
        Reach a predefined target by name

        Args:
            target_name: Name of target from JOINT_TARGETS
            duration_sec: Time to reach target

        Returns:
            bool: True if motion succeeded
        """
        if target_name not in JOINT_TARGETS:
            self.get_logger().error(f'Unknown target: {target_name}')
            self.get_logger().info(f'Available targets: {list(JOINT_TARGETS.keys())}')
            return False

        joint_positions = JOINT_TARGETS[target_name]

        # Show target info
        if target_name in SPHERE_POSITIONS:
            x, y, z = SPHERE_POSITIONS[target_name]
            self.get_logger().info(f'Target: {target_name} at ({x:.3f}, {y:.3f}, {z:.3f})')
        else:
            self.get_logger().info(f'Target: {target_name}')

        return self.send_joint_trajectory(joint_positions, duration_sec)

    def reach_all_spheres(self, delay_between=3.0, duration_per_target=5.0):
        """
        Reach all sphere targets in sequence

        Args:
            delay_between: Delay in seconds between targets
            duration_per_target: Motion duration for each target
        """
        self.get_logger().info('=== Starting sequence to reach all spheres ===\n')

        # Go to home first
        self.get_logger().info('Moving to home position first...')
        if not self.reach_named_target('home', duration_sec=4.0):
            self.get_logger().error('Failed to reach home position')
            return False

        import time
        time.sleep(delay_between)

        # Visit each sphere
        sphere_targets = ['sphere1', 'sphere2', 'sphere3']

        for i, target in enumerate(sphere_targets, 1):
            self.get_logger().info(f'\n=== Sphere {i}/{len(sphere_targets)}: {target} ===')
            success = self.reach_named_target(target, duration_sec=duration_per_target)

            if not success:
                self.get_logger().error(f'Failed to reach {target}')
                return False

            if i < len(sphere_targets):
                self.get_logger().info(f'Waiting {delay_between}s before next target...\n')
                time.sleep(delay_between)

        # Return home
        self.get_logger().info('\n=== Returning to home position ===')
        self.reach_named_target('home', duration_sec=4.0)

        self.get_logger().info('\n=== All spheres reached successfully! ===')
        return True


def main(args=None):
    parser = argparse.ArgumentParser(description='Move robot arm to target positions')
    parser.add_argument('--target', type=str,
                        choices=['home', 'sphere1', 'sphere2', 'sphere3', 'all',
                                'reach_forward', 'reach_up', 'test_small'],
                        required=True,
                        help='Target name')
    parser.add_argument('--duration', type=float, default=15.0,
                        help='Motion duration in seconds (default: 15.0)')
    parser.add_argument('--delay', type=float, default=3.0,
                        help='Delay between targets when using --target all (default: 3.0s)')

    # Parse args
    parsed_args = parser.parse_args()

    # Initialize ROS2
    rclpy.init(args=args)

    # Create node
    node = ReachTargetSimpleNode()

    # Execute motion
    try:
        if parsed_args.target == 'all':
            node.reach_all_spheres(
                delay_between=parsed_args.delay,
                duration_per_target=parsed_args.duration
            )
        else:
            node.reach_named_target(
                parsed_args.target,
                duration_sec=parsed_args.duration
            )

    except KeyboardInterrupt:
        node.get_logger().info('Motion interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Error during motion: {e}')
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
