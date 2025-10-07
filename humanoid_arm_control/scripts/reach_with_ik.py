#!/usr/bin/env python3

"""
Reach Target with IK Service - Compute IK then execute trajectory

Uses MoveIt's /compute_ik service to get joint angles for target position,
then sends those angles to the joint_trajectory_controller.

This is the simplest and most reliable approach:
1. Call /compute_ik service with target XYZ → get joint angles
2. Send joint angles to trajectory controller → arm moves

Usage:
    # Reach specific XYZ coordinates
    ros2 run humanoid_arm_control reach_with_ik.py --x -0.6 --y 0.3 --z 0.7

    # Reach predefined sphere targets
    ros2 run humanoid_arm_control reach_with_ik.py --target sphere1
    ros2 run humanoid_arm_control reach_with_ik.py --target sphere2
    ros2 run humanoid_arm_control reach_with_ik.py --target sphere3

    # Reach all spheres in sequence
    ros2 run humanoid_arm_control reach_with_ik.py --target all

Requirements:
    - MoveIt2 running (full_system.launch.py) for IK service
    - joint_trajectory_controller active
"""

import sys
import argparse
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Predefined target sphere positions
SPHERE_POSITIONS = {
    'sphere1': (-0.6, 0.3, 0.7),   # Red - right
    'sphere2': (-0.5, -0.3, 0.9),  # Blue - left
    'sphere3': (-0.9, 0.0, 0.5),   # Green - center, further
}


class ReachWithIKNode(Node):
    """Node to reach targets using IK service + trajectory controller"""

    def __init__(self):
        super().__init__('reach_with_ik_node')

        # Joint names
        self.joint_names = [
            'base_rotation_joint',
            'shoulder_pitch_joint',
            'elbow_pitch_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
        ]

        # Create IK service client
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')

        self.get_logger().info('Waiting for IK service...')
        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('IK service not available!')
            self.get_logger().error('Make sure MoveIt is running (full_system.launch.py)')
            raise RuntimeError('IK service timeout')

        # Create trajectory action client
        self._traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info('Waiting for trajectory controller...')
        if not self._traj_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Trajectory controller not available!')
            raise RuntimeError('Trajectory controller timeout')

        self.get_logger().info('Ready to reach targets!')

    def compute_ik(self, x, y, z):
        """
        Compute IK for target position

        Args:
            x, y, z: Target position in meters (base_link frame)

        Returns:
            list: Joint positions, or None if IK failed
        """
        self.get_logger().info(f'Computing IK for: ({x:.3f}, {y:.3f}, {z:.3f})')

        # Create IK request
        request = GetPositionIK.Request()
        request.ik_request.group_name = "humanoid_5dof_arm"
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(sec=5)

        # Set target pose
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header.frame_id = "base_link"
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        request.ik_request.pose_stamped.pose.position.z = z

        # Default orientation (end effector pointing down)
        request.ik_request.pose_stamped.pose.orientation.x = 0.7071
        request.ik_request.pose_stamped.pose.orientation.y = 0.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.7071

        # Call service
        future = self._ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        if response.error_code.val == 1:  # SUCCESS
            # Extract joint positions
            joint_positions = []
            for joint_name in self.joint_names:
                idx = response.solution.joint_state.name.index(joint_name)
                joint_positions.append(response.solution.joint_state.position[idx])

            self.get_logger().info(f'IK solution: {[f"{p:.3f}" for p in joint_positions]}')
            return joint_positions
        else:
            self.get_logger().error(f'IK failed with error code: {response.error_code.val}')
            return None

    def execute_trajectory(self, joint_positions, duration_sec=15.0):
        """
        Execute trajectory to joint positions

        Args:
            joint_positions: List of 5 joint positions [rad]
            duration_sec: Time to reach target

        Returns:
            bool: True if execution succeeded
        """
        self.get_logger().info(f'Executing trajectory ({duration_sec}s)...')

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
        send_goal_future = self._traj_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected')
            return False

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

    def reach_position(self, x, y, z):
        """
        Reach target position using IK + trajectory execution

        Args:
            x, y, z: Target position in meters

        Returns:
            bool: True if successful
        """
        # Compute IK
        joint_positions = self.compute_ik(x, y, z)
        if joint_positions is None:
            return False

        # Execute trajectory
        return self.execute_trajectory(joint_positions)

    def reach_named_target(self, target_name):
        """Reach predefined target"""
        if target_name not in SPHERE_POSITIONS:
            self.get_logger().error(f'Unknown target: {target_name}')
            return False

        x, y, z = SPHERE_POSITIONS[target_name]
        self.get_logger().info(f'Target: {target_name}')
        return self.reach_position(x, y, z)

    def reach_all_targets(self, delay_between=3.0):
        """Reach all sphere targets in sequence"""
        self.get_logger().info('=== Starting sequence ===\n')

        for i, target in enumerate(['sphere1', 'sphere2', 'sphere3'], 1):
            self.get_logger().info(f'\n=== Sphere {i}/3: {target} ===')
            success = self.reach_named_target(target)

            if not success:
                self.get_logger().error(f'Failed to reach {target}')
                return False

            if i < 3:
                self.get_logger().info(f'Waiting {delay_between}s...\n')
                time.sleep(delay_between)

        self.get_logger().info('\n=== All spheres reached! ===')
        return True


def main(args=None):
    parser = argparse.ArgumentParser(description='Reach targets using IK')
    parser.add_argument('--x', type=float, help='Target X (meters)')
    parser.add_argument('--y', type=float, help='Target Y (meters)')
    parser.add_argument('--z', type=float, help='Target Z (meters)')
    parser.add_argument('--target', type=str,
                        choices=['sphere1', 'sphere2', 'sphere3', 'all'],
                        help='Target name')
    parser.add_argument('--delay', type=float, default=3.0,
                        help='Delay between targets (default: 3.0s)')

    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    node = ReachWithIKNode()

    try:
        if parsed_args.target:
            if parsed_args.target == 'all':
                node.reach_all_targets(delay_between=parsed_args.delay)
            else:
                node.reach_named_target(parsed_args.target)
        elif parsed_args.x is not None and parsed_args.y is not None and parsed_args.z is not None:
            node.reach_position(parsed_args.x, parsed_args.y, parsed_args.z)
        else:
            node.get_logger().error('Must specify --target or --x --y --z')
            parser.print_help()

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
