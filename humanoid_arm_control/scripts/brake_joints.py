#!/usr/bin/env python3
"""
Virtual Brake Control for Single-Joint PID Tuning

Simulates electromagnetic brakes by increasing joint damping for non-active joints.
This prevents oscillation/interference when tuning a single joint.

Usage:
    # Brake all joints EXCEPT base_rotation (for tuning base_rotation)
    ./brake_joints.py --free base_rotation_joint

    # Brake all joints EXCEPT shoulder_pitch
    ./brake_joints.py --free shoulder_pitch_joint

    # Release all brakes (restore normal damping)
    ./brake_joints.py --release

How it works:
    - Uses Gazebo's dynamic joint parameter setting
    - Temporarily increases damping coefficient to simulate brake
    - Can be engaged/released without stopping controllers
"""

import subprocess
import argparse
import time
import sys


class BrakeController:
    """Controls virtual brakes on Gazebo joints"""

    JOINTS = [
        'base_rotation_joint',
        'shoulder_pitch_joint',
        'elbow_pitch_joint',
        'wrist_pitch_joint',
        'wrist_roll_joint'
    ]

    # Normal damping values (from URDF)
    NORMAL_DAMPING = 0.1

    # Brake damping values (simulates electromagnetic brake)
    # High enough to prevent motion but not infinite
    BRAKE_DAMPING = {
        'base_rotation_joint': 50.0,      # Heavy base needs high damping
        'shoulder_pitch_joint': 80.0,     # Fights gravity - highest damping
        'elbow_pitch_joint': 40.0,        # Medium
        'wrist_pitch_joint': 30.0,        # Light
        'wrist_roll_joint': 20.0,         # Lightest
    }

    def __init__(self, model_name='humanoid_arm'):
        self.model_name = model_name
        self.engaged_brakes = set()

    def set_joint_damping(self, joint_name, damping_value):
        """Set damping coefficient for a joint via Gazebo command"""
        topic = f"/model/{self.model_name}/joint/{joint_name}/damping_coefficient"
        cmd = [
            'gz', 'topic', '-t', topic,
            '-m', 'gz.msgs.Double',
            '-p', f'data: {damping_value}'
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                return True
            else:
                print(f"Warning: Failed to set damping for {joint_name}: {result.stderr}")
                return False
        except subprocess.TimeoutExpired:
            print(f"Warning: Timeout setting damping for {joint_name}")
            return False
        except FileNotFoundError:
            print("Error: 'gz' command not found. Is Gazebo Harmonic running?")
            return False

    def engage_brake(self, joint_name):
        """Engage brake on a specific joint"""
        damping = self.BRAKE_DAMPING.get(joint_name, 50.0)
        print(f"  Engaging brake on {joint_name} (damping: {self.NORMAL_DAMPING} → {damping})")

        if self.set_joint_damping(joint_name, damping):
            self.engaged_brakes.add(joint_name)
            return True
        return False

    def release_brake(self, joint_name):
        """Release brake on a specific joint"""
        print(f"  Releasing brake on {joint_name} (damping: → {self.NORMAL_DAMPING})")

        if self.set_joint_damping(joint_name, self.NORMAL_DAMPING):
            self.engaged_brakes.discard(joint_name)
            return True
        return False

    def brake_all_except(self, free_joints):
        """Engage brakes on all joints except specified ones"""
        print(f"\n🔒 Engaging brakes on all joints except: {free_joints}")
        print(f"   Free joints can be tuned without interference\n")

        success_count = 0
        for joint in self.JOINTS:
            if joint not in free_joints:
                if self.engage_brake(joint):
                    success_count += 1
            else:
                print(f"  Skipping {joint} (free for tuning)")

        print(f"\n✓ Engaged {success_count} brakes")
        return success_count

    def release_all_brakes(self):
        """Release all brakes"""
        print(f"\n🔓 Releasing all brakes...")

        success_count = 0
        for joint in self.JOINTS:
            if self.release_brake(joint):
                success_count += 1

        print(f"\n✓ Released {success_count} brakes")
        return success_count

    def status(self):
        """Print current brake status"""
        print("\n📊 Brake Status:")
        for joint in self.JOINTS:
            status = "🔒 ENGAGED" if joint in self.engaged_brakes else "🔓 Released"
            print(f"  {joint}: {status}")
        print()


def main():
    parser = argparse.ArgumentParser(
        description='Control virtual brakes for single-joint PID tuning',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Tune base_rotation_joint (brake all others)
  %(prog)s --free base_rotation_joint

  # Tune shoulder_pitch_joint
  %(prog)s --free shoulder_pitch_joint

  # Tune multiple joints simultaneously (brake others)
  %(prog)s --free base_rotation_joint shoulder_pitch_joint

  # Release all brakes
  %(prog)s --release

  # Check current brake status
  %(prog)s --status

Note: Gazebo must be running for this script to work.
        """
    )

    parser.add_argument('--free', nargs='+', metavar='JOINT',
                       help='Joint(s) to leave free for tuning (brake all others)')
    parser.add_argument('--release', action='store_true',
                       help='Release all brakes')
    parser.add_argument('--status', action='store_true',
                       help='Show current brake status')
    parser.add_argument('--model', default='humanoid_arm',
                       help='Gazebo model name (default: humanoid_arm)')

    args = parser.parse_args()

    # Check if at least one action is specified
    if not (args.free or args.release or args.status):
        parser.print_help()
        sys.exit(1)

    controller = BrakeController(model_name=args.model)

    # Execute requested action
    if args.release:
        controller.release_all_brakes()
    elif args.free:
        # Validate joint names
        invalid_joints = [j for j in args.free if j not in controller.JOINTS]
        if invalid_joints:
            print(f"Error: Invalid joint names: {invalid_joints}")
            print(f"Valid joints: {controller.JOINTS}")
            sys.exit(1)

        controller.brake_all_except(args.free)

    if args.status:
        controller.status()

    print("\n💡 Tips:")
    print("  - Brakes simulate electromagnetic brakes by increasing joint damping")
    print("  - Use this when tuning a single joint to prevent interference")
    print("  - Release brakes when done: ./brake_joints.py --release")
    print()


if __name__ == '__main__':
    main()
