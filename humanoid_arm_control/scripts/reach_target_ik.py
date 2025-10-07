#!/usr/bin/env python3

"""
IK-based Reach Target Script - Uses MoveIt2 to compute inverse kinematics

This script uses MoveIt's IK solver to automatically calculate joint angles
for target XYZ positions and executes the motion.

Usage:
    # Reach specific XYZ coordinates
    ros2 run humanoid_arm_control reach_target_ik.py --x -0.6 --y 0.3 --z 0.7

    # Reach predefined sphere targets
    ros2 run humanoid_arm_control reach_target_ik.py --target sphere1
    ros2 run humanoid_arm_control reach_target_ik.py --target sphere2
    ros2 run humanoid_arm_control reach_target_ik.py --target sphere3

    # Reach all spheres in sequence
    ros2 run humanoid_arm_control reach_target_ik.py --target all

Requirements:
    - MoveIt2 running (full_system.launch.py)
    - ros-jazzy-moveit-py installed
"""

import sys
import argparse
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

# Predefined target sphere positions
SPHERE_POSITIONS = {
    'sphere1': (-0.6, 0.3, 0.7),   # Red - right
    'sphere2': (-0.5, -0.3, 0.9),  # Blue - left
    'sphere3': (-0.9, 0.0, 0.5),   # Green - center, further
}


class ReachTargetIKNode(Node):
    """Node to reach targets using MoveIt IK solver"""

    def __init__(self):
        super().__init__('reach_target_ik_node')

        self.get_logger().info('Initializing MoveIt...')

        # Initialize MoveIt
        self.moveit = MoveItPy(node_name="moveit_py_planning")
        self.arm_group = self.moveit.get_planning_component("humanoid_5dof_arm")
        self.robot_model = self.moveit.get_robot_model()

        self.get_logger().info('MoveIt initialized successfully')
        self.get_logger().info(f'Planning group: humanoid_5dof_arm')

    def reach_position(self, x, y, z):
        """
        Move end effector to target XYZ position using IK

        Args:
            x, y, z: Target position in meters (base_link frame)

        Returns:
            bool: True if motion succeeded
        """
        self.get_logger().info(f'Planning to reach: ({x:.3f}, {y:.3f}, {z:.3f})')

        # Create pose goal
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z

        # Default orientation (end effector pointing down)
        pose_goal.pose.orientation.x = 0.7071
        pose_goal.pose.orientation.y = 0.0
        pose_goal.pose.orientation.z = 0.0
        pose_goal.pose.orientation.w = 0.7071

        # Set goal
        self.arm_group.set_goal_state(pose_stamped_msg=pose_goal, pose_link="wrist_roll_link")

        # Plan
        self.get_logger().info('Computing motion plan...')
        plan_result = self.arm_group.plan()

        if not plan_result:
            self.get_logger().error('Planning failed!')
            return False

        self.get_logger().info('Plan successful, executing...')

        # Execute
        robot_trajectory = plan_result.trajectory
        execute_result = self.moveit.execute(robot_trajectory, blocking=True)

        if execute_result:
            self.get_logger().info('Motion completed successfully!')
            return True
        else:
            self.get_logger().error('Execution failed!')
            return False

    def reach_named_target(self, target_name):
        """
        Reach a predefined target by name

        Args:
            target_name: Name of target from SPHERE_POSITIONS

        Returns:
            bool: True if motion succeeded
        """
        if target_name not in SPHERE_POSITIONS:
            self.get_logger().error(f'Unknown target: {target_name}')
            self.get_logger().info(f'Available targets: {list(SPHERE_POSITIONS.keys())}')
            return False

        x, y, z = SPHERE_POSITIONS[target_name]
        self.get_logger().info(f'Target: {target_name}')
        return self.reach_position(x, y, z)

    def reach_all_targets(self, delay_between=3.0):
        """
        Reach all predefined targets in sequence

        Args:
            delay_between: Delay in seconds between targets
        """
        self.get_logger().info('=== Starting sequence to reach all spheres ===\n')

        sphere_targets = ['sphere1', 'sphere2', 'sphere3']

        for i, target in enumerate(sphere_targets, 1):
            self.get_logger().info(f'\n=== Sphere {i}/{len(sphere_targets)}: {target} ===')
            success = self.reach_named_target(target)

            if not success:
                self.get_logger().error(f'Failed to reach {target}')
                return False

            if i < len(sphere_targets):
                self.get_logger().info(f'Waiting {delay_between}s before next target...\n')
                time.sleep(delay_between)

        self.get_logger().info('\n=== All spheres reached successfully! ===')
        return True


def main(args=None):
    parser = argparse.ArgumentParser(description='Move robot arm using IK to reach targets')
    parser.add_argument('--x', type=float, help='Target X position (meters)')
    parser.add_argument('--y', type=float, help='Target Y position (meters)')
    parser.add_argument('--z', type=float, help='Target Z position (meters)')
    parser.add_argument('--target', type=str,
                        choices=['sphere1', 'sphere2', 'sphere3', 'all'],
                        help='Predefined target name')
    parser.add_argument('--delay', type=float, default=3.0,
                        help='Delay between targets when using --target all (default: 3.0s)')

    # Parse args
    parsed_args = parser.parse_args()

    # Initialize ROS2
    rclpy.init(args=args)

    # Create node
    node = ReachTargetIKNode()

    # Execute motion
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
