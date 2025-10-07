#!/usr/bin/env python3

"""
Reach Target Script - Move robot arm to target XYZ positions using MoveIt2

This script uses MoveIt2's move_group interface to:
1. Calculate inverse kinematics for target position
2. Plan collision-free trajectory
3. Execute motion to reach target

Usage:
    # Reach a specific target
    ros2 run humanoid_arm_control reach_target.py --x 1.0 --y 0.5 --z 1.2

    # Reach predefined sphere targets
    ros2 run humanoid_arm_control reach_target.py --target sphere1
    ros2 run humanoid_arm_control reach_target.py --target sphere2
    ros2 run humanoid_arm_control reach_target.py --target sphere3

    # Reach all spheres in sequence
    ros2 run humanoid_arm_control reach_target.py --target all

Predefined Targets:
    sphere1: (1.0, 0.5, 1.2)
    sphere2: (0.8, -0.3, 1.5)
    sphere3: (1.2, 0.0, 1.0)

Requirements:
    - MoveIt2 running (use full_system.launch.py)
    - Controllers loaded (joint_trajectory_controller)
"""

import sys
import argparse
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from pymoveit2 import MoveIt2
from pymoveit2.robots import humanoid_arm_5dof

# Predefined target positions (from Gazebo sphere locations)
TARGETS = {
    'sphere1': (1.0, 0.5, 1.2),
    'sphere2': (0.8, -0.3, 1.5),
    'sphere3': (1.2, 0.0, 1.0),
}


class ReachTargetNode(Node):
    """Node to reach target positions using MoveIt2"""

    def __init__(self):
        super().__init__('reach_target_node')

        # Initialize MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                'base_rotation_joint',
                'shoulder_pitch_joint',
                'elbow_pitch_joint',
                'wrist_pitch_joint',
                'wrist_roll_joint',
            ],
            base_link_name='base_link',
            end_effector_name='wrist_roll_link',  # TCP link
            group_name='arm',
        )

        self.get_logger().info('ReachTargetNode initialized')
        self.get_logger().info('MoveIt2 interface ready')

    def reach_position(self, x, y, z, orientation=None):
        """
        Move end effector to target XYZ position

        Args:
            x, y, z: Target position in meters (base_link frame)
            orientation: Optional quaternion (x, y, z, w). If None, uses default orientation

        Returns:
            bool: True if motion succeeded, False otherwise
        """
        self.get_logger().info(f'Planning to reach target: ({x:.3f}, {y:.3f}, {z:.3f})')

        # Create pose goal
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        # Default orientation (pointing forward/down for manipulation)
        if orientation is None:
            # Quaternion for end effector pointing downward
            pose.orientation.x = 0.7071
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.7071
        else:
            pose.orientation.x = orientation[0]
            pose.orientation.y = orientation[1]
            pose.orientation.z = orientation[2]
            pose.orientation.w = orientation[3]

        # Plan motion
        self.get_logger().info('Computing IK and planning trajectory...')
        self.moveit2.move_to_pose(
            position=[x, y, z],
            quat_xyzw=orientation if orientation else [0.7071, 0.0, 0.0, 0.7071],
            cartesian=False,  # Use joint space planning (faster, more reliable)
        )

        # Wait for motion to complete
        self.get_logger().info('Executing motion...')
        self.moveit2.wait_until_executed()

        self.get_logger().info('Motion completed')
        return True

    def reach_named_target(self, target_name):
        """
        Reach a predefined target by name

        Args:
            target_name: Name of target ('sphere1', 'sphere2', 'sphere3')

        Returns:
            bool: True if motion succeeded
        """
        if target_name not in TARGETS:
            self.get_logger().error(f'Unknown target: {target_name}')
            self.get_logger().info(f'Available targets: {list(TARGETS.keys())}')
            return False

        x, y, z = TARGETS[target_name]
        self.get_logger().info(f'Reaching target: {target_name}')
        return self.reach_position(x, y, z)

    def reach_all_targets(self, delay_between=3.0):
        """
        Reach all predefined targets in sequence

        Args:
            delay_between: Delay in seconds between targets
        """
        self.get_logger().info('Starting sequence to reach all targets')

        for i, (name, (x, y, z)) in enumerate(TARGETS.items(), 1):
            self.get_logger().info(f'\n=== Target {i}/{len(TARGETS)}: {name} ===')
            success = self.reach_position(x, y, z)

            if not success:
                self.get_logger().error(f'Failed to reach {name}')
                return False

            if i < len(TARGETS):
                self.get_logger().info(f'Waiting {delay_between}s before next target...')
                import time
                time.sleep(delay_between)

        self.get_logger().info('\n=== All targets reached successfully! ===')
        return True


def main(args=None):
    parser = argparse.ArgumentParser(description='Move robot arm to target positions')
    parser.add_argument('--x', type=float, help='Target X position (meters)')
    parser.add_argument('--y', type=float, help='Target Y position (meters)')
    parser.add_argument('--z', type=float, help='Target Z position (meters)')
    parser.add_argument('--target', type=str, choices=['sphere1', 'sphere2', 'sphere3', 'all'],
                        help='Predefined target name')
    parser.add_argument('--delay', type=float, default=3.0,
                        help='Delay between targets when using --target all (default: 3.0s)')

    # Parse args (filter out ROS args)
    parsed_args = parser.parse_args(args=sys.argv[1:] if args is None else args)

    # Initialize ROS2
    rclpy.init(args=args)

    # Create node
    node = ReachTargetNode()

    # Execute motion based on arguments
    try:
        if parsed_args.target:
            # Named target(s)
            if parsed_args.target == 'all':
                node.reach_all_targets(delay_between=parsed_args.delay)
            else:
                node.reach_named_target(parsed_args.target)
        elif parsed_args.x is not None and parsed_args.y is not None and parsed_args.z is not None:
            # Custom XYZ position
            node.reach_position(parsed_args.x, parsed_args.y, parsed_args.z)
        else:
            # No arguments - show help
            node.get_logger().error('Must specify either --target or --x --y --z')
            parser.print_help()
            return

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
