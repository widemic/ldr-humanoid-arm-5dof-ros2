#!/usr/bin/env python3

"""
MoveIt Action Client - Reach targets using move_group action interface

Uses MoveIt's move_group action server to compute IK and execute motion.
Much simpler than MoveItPy - just needs move_group to be running.

Usage:
    # Reach specific XYZ coordinates
    ros2 run humanoid_arm_control reach_target_moveit.py --x -0.6 --y 0.3 --z 0.7

    # Reach predefined sphere targets
    ros2 run humanoid_arm_control reach_target_moveit.py --target sphere1
    ros2 run humanoid_arm_control reach_target_moveit.py --target sphere2
    ros2 run humanoid_arm_control reach_target_moveit.py --target sphere3

    # Reach all spheres in sequence
    ros2 run humanoid_arm_control reach_target_moveit.py --target all

Requirements:
    - MoveIt2 running (full_system.launch.py)
"""

import sys
import argparse
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive

# Predefined target sphere positions
SPHERE_POSITIONS = {
    'sphere1': (-0.6, 0.3, 0.7),   # Red - right
    'sphere2': (-0.5, -0.3, 0.9),  # Blue - left
    'sphere3': (-0.9, 0.0, 0.5),   # Green - center, further
}


class ReachTargetMoveItNode(Node):
    """Node to reach targets using MoveIt move_group action"""

    def __init__(self):
        super().__init__('reach_target_moveit_node')

        # Create action client for move_group
        self._action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.get_logger().info('Waiting for move_group action server...')

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('move_group action server not available!')
            self.get_logger().error('Make sure full_system.launch.py is running')
            raise RuntimeError('Action server timeout')

        self.get_logger().info('move_group action server ready')

    def reach_position(self, x, y, z):
        """
        Move end effector to target XYZ position

        Args:
            x, y, z: Target position in meters (base_link frame)

        Returns:
            bool: True if motion succeeded
        """
        self.get_logger().info(f'Planning to reach: ({x:.3f}, {y:.3f}, {z:.3f})')

        # Create goal message
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "humanoid_5dof_arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1

        # Set workspace bounds
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.workspace_parameters.min_corner.x = -2.0
        goal_msg.request.workspace_parameters.min_corner.y = -2.0
        goal_msg.request.workspace_parameters.min_corner.z = -2.0
        goal_msg.request.workspace_parameters.max_corner.x = 2.0
        goal_msg.request.workspace_parameters.max_corner.y = 2.0
        goal_msg.request.workspace_parameters.max_corner.z = 2.0

        # Create pose constraint for end effector
        pose_constraint = PositionConstraint()
        pose_constraint.header.frame_id = "base_link"
        pose_constraint.link_name = "wrist_roll_link"

        # Target position as a small bounding box
        pose_constraint.constraint_region.primitives.append(SolidPrimitive())
        pose_constraint.constraint_region.primitives[0].type = SolidPrimitive.SPHERE
        pose_constraint.constraint_region.primitives[0].dimensions = [0.01]  # 1cm tolerance

        pose_constraint.constraint_region.primitive_poses.append(Pose())
        pose_constraint.constraint_region.primitive_poses[0].position.x = x
        pose_constraint.constraint_region.primitive_poses[0].position.y = y
        pose_constraint.constraint_region.primitive_poses[0].position.z = z
        pose_constraint.constraint_region.primitive_poses[0].orientation.w = 1.0

        pose_constraint.weight = 1.0

        # Add to goal constraints
        goal_msg.request.goal_constraints.append(Constraints())
        goal_msg.request.goal_constraints[0].position_constraints.append(pose_constraint)

        # Send goal
        self.get_logger().info('Sending goal to move_group...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by move_group')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        error_code = result.error_code.val

        if error_code == 1:  # SUCCESS
            self.get_logger().info('Motion completed successfully!')
            return True
        else:
            self.get_logger().error(f'Motion failed with error code: {error_code}')
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
    parser = argparse.ArgumentParser(description='Move robot arm using MoveIt to reach targets')
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
    node = ReachTargetMoveItNode()

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
