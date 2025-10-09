#!/bin/bash
# Single Joint Tuning Helper Script
# Usage: ./tune_single_joint.sh <joint_number>
# Example: ./tune_single_joint.sh 0

set -e

JOINT_NUM=$1

if [ -z "$JOINT_NUM" ]; then
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  SINGLE JOINT TUNING HELPER"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "Usage: $0 <joint_number>"
    echo ""
    echo "Available joints:"
    echo "  0 - base_rotation_joint"
    echo "  1 - shoulder_pitch_joint"
    echo "  2 - elbow_pitch_joint"
    echo "  3 - wrist_pitch_joint"
    echo "  4 - wrist_roll_joint"
    echo ""
    echo "Example: $0 0    # Tune base_rotation_joint"
    echo ""
    exit 1
fi

# Joint names mapping
declare -A JOINT_NAMES=(
    [0]="base_rotation_joint"
    [1]="shoulder_pitch_joint"
    [2]="elbow_pitch_joint"
    [3]="wrist_pitch_joint"
    [4]="wrist_roll_joint"
)

declare -A CONTROLLER_NAMES=(
    [0]="base_rotation_joint_position_controller"
    [1]="shoulder_pitch_joint_position_controller"
    [2]="elbow_pitch_joint_position_controller"
    [3]="wrist_pitch_joint_position_controller"
    [4]="wrist_roll_joint_position_controller"
)

if [ -z "${JOINT_NAMES[$JOINT_NUM]}" ]; then
    echo "❌ Invalid joint number: $JOINT_NUM"
    echo "   Valid range: 0-4"
    exit 1
fi

JOINT_NAME=${JOINT_NAMES[$JOINT_NUM]}
CONTROLLER_NAME=${CONTROLLER_NAMES[$JOINT_NUM]}

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  SWITCHING TO SINGLE JOINT MODE"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "  Joint Number:     $JOINT_NUM"
echo "  Joint Name:       $JOINT_NAME"
echo "  Controller:       $CONTROLLER_NAME"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check if robot is running
echo ""
echo "⏳ Checking if robot is running..."
if ! ros2 node list | grep -q "controller_manager"; then
    echo "❌ ERROR: controller_manager not found!"
    echo "   Please launch the robot first:"
    echo "   ros2 launch humanoid_arm_bringup full_system.launch.py"
    exit 1
fi
echo "✅ Robot is running"

# Stop all individual controllers first (in case one is running)
echo ""
echo "⏳ Stopping any active individual controllers..."
for i in 0 1 2 3 4; do
    CTRL=${CONTROLLER_NAMES[$i]}
    STATUS=$(ros2 control list_controllers | grep "$CTRL" | awk '{print $2}' || echo "unknown")
    if [ "$STATUS" = "active" ]; then
        echo "   Stopping $CTRL..."
        ros2 control switch_controllers --stop-controllers $CTRL 2>/dev/null || true
    fi
done

# Stop main trajectory controller
echo ""
echo "⏳ Stopping main trajectory controller..."
MAIN_STATUS=$(ros2 control list_controllers | grep "joint_trajectory_controller" | awk '{print $2}' || echo "unknown")
if [ "$MAIN_STATUS" = "active" ]; then
    ros2 control switch_controllers --stop-controllers joint_trajectory_controller
    echo "✅ Stopped joint_trajectory_controller"
else
    echo "ℹ️  joint_trajectory_controller already stopped"
fi

# Start single-joint controller
echo ""
echo "⏳ Starting $CONTROLLER_NAME..."
ros2 control switch_controllers --start-controllers $CONTROLLER_NAME

if [ $? -eq 0 ]; then
    echo "✅ Successfully switched to single-joint mode"
else
    echo "❌ ERROR: Failed to start controller"
    exit 1
fi

# Show controller status
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  ACTIVE CONTROLLERS:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
ros2 control list_controllers | grep -E "(active|inactive)"

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  READY TO TUNE!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "  ✅ Only joint $JOINT_NUM ($JOINT_NAME) is active"
echo "  ✅ Other 4 joints are unpowered (won't interfere)"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  NEXT STEPS:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "  1️⃣  Launch PID tuner GUI (in another terminal):"
echo "     ros2 run humanoid_arm_control pid_tuner_gui.py"
echo ""
echo "  2️⃣  Test trajectory (in another terminal):"
echo "     python3 humanoid_arm_control/scripts/test_trajectory.py \\"
echo "       --joint $JOINT_NUM --type step --amplitude 0.5 --reset"
echo ""
echo "  3️⃣  Tune P/I/D values with GUI until happy"
echo ""
echo "  4️⃣  Move to next joint:"
echo "     $0 $((JOINT_NUM + 1))"
echo ""
echo "  5️⃣  Return to multi-joint mode (when done with all joints):"
echo "     $0 done"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
