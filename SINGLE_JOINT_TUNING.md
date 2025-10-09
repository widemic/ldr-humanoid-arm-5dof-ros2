# Single Joint PID Tuning Guide

**Problem:** When tuning one joint with the full trajectory controller, other joints may oscillate or interfere with your measurements.

**Solution:** Use **individual joint controllers** to isolate each joint completely.

## Quick Start: Isolate a Single Joint

### Step 1: Launch System (Normal)

```bash
ros2 launch humanoid_arm_bringup full_system.launch.py
```

### Step 2: Stop Multi-Joint Controller

```bash
# Stop the main trajectory controller (controls all 5 joints)
ros2 control switch_controllers \
  --stop-controllers joint_trajectory_controller
```

### Step 3: Start Individual Joint Controller

```bash
# Example: Tune base_rotation_joint only
ros2 control switch_controllers \
  --start-controllers base_rotation_joint_position_controller
```

### Step 4: Tune with PID GUI (Only This Joint Active)

```bash
# In another terminal
ros2 run humanoid_arm_control pid_tuner_gui.py
```

### Step 5: Test Isolated Joint

```bash
# Test trajectory for joint 0 (base_rotation)
python3 humanoid_arm_control/scripts/test_trajectory.py \
  --joint 0 --type step --amplitude 0.5 --reset
```

**Now:** Only base_rotation_joint moves. All other joints are **unpowered** and won't interfere!

---

## Available Individual Controllers

Your system has 5 individual controllers (one per joint):

| Controller Name | Joint | Use Case |
|-----------------|-------|----------|
| `base_rotation_joint_position_controller` | base_rotation_joint | Tune joint 0 |
| `shoulder_pitch_joint_position_controller` | shoulder_pitch_joint | Tune joint 1 |
| `elbow_pitch_joint_position_controller` | elbow_pitch_joint | Tune joint 2 |
| `wrist_pitch_joint_position_controller` | wrist_pitch_joint | Tune joint 3 |
| `wrist_roll_joint_position_controller` | wrist_roll_joint | Tune joint 4 |

**Note:** Joint numbers (0-4) map to controllers in this order.

---

## Complete Workflow: Tune Each Joint

### 1. Base Rotation Joint (Joint 0)

```bash
# Switch to single-joint controller
ros2 control switch_controllers \
  --stop-controllers joint_trajectory_controller \
  --start-controllers base_rotation_joint_position_controller

# Launch PID tuner
ros2 run humanoid_arm_control pid_tuner_gui.py

# Test
python3 humanoid_arm_control/scripts/test_trajectory.py \
  --joint 0 --type step --amplitude 0.5 --reset

# When done, save values and move to next joint
```

### 2. Shoulder Pitch Joint (Joint 1)

```bash
# Switch controllers
ros2 control switch_controllers \
  --stop-controllers base_rotation_joint_position_controller \
  --start-controllers shoulder_pitch_joint_position_controller

# Tune with GUI
ros2 run humanoid_arm_control pid_tuner_gui.py

# Test
python3 humanoid_arm_control/scripts/test_trajectory.py \
  --joint 1 --type step --amplitude 0.5 --reset
```

### 3. Elbow Pitch Joint (Joint 2)

```bash
ros2 control switch_controllers \
  --stop-controllers shoulder_pitch_joint_position_controller \
  --start-controllers elbow_pitch_joint_position_controller

ros2 run humanoid_arm_control pid_tuner_gui.py

python3 humanoid_arm_control/scripts/test_trajectory.py \
  --joint 2 --type step --amplitude 0.5 --reset
```

### 4. Wrist Pitch Joint (Joint 3)

```bash
ros2 control switch_controllers \
  --stop-controllers elbow_pitch_joint_position_controller \
  --start-controllers wrist_pitch_joint_position_controller

ros2 run humanoid_arm_control pid_tuner_gui.py

python3 humanoid_arm_control/scripts/test_trajectory.py \
  --joint 3 --type step --amplitude 0.5 --reset
```

### 5. Wrist Roll Joint (Joint 4)

```bash
ros2 control switch_controllers \
  --stop-controllers wrist_pitch_joint_position_controller \
  --start-controllers wrist_roll_joint_position_controller

ros2 run humanoid_arm_control pid_tuner_gui.py

python3 humanoid_arm_control/scripts/test_trajectory.py \
  --joint 4 --type step --amplitude 0.5 --reset
```

---

## After Tuning All Joints: Return to Multi-Joint Mode

Once each joint is individually tuned:

```bash
# Stop the last individual controller
ros2 control switch_controllers \
  --stop-controllers wrist_roll_joint_position_controller \
  --start-controllers joint_trajectory_controller

# Now test coordinated motion with MoveIt
```

---

## Why This Works

### Controller Exclusivity

ros2_control ensures **only one controller** can claim a joint's command interface at a time:

- `joint_trajectory_controller` → Claims ALL 5 joints
- `base_rotation_joint_position_controller` → Claims ONLY base_rotation_joint

**When you switch:**
1. Stop multi-joint controller → Releases all 5 joints
2. Start single-joint controller → Claims only 1 joint
3. **Other 4 joints become unpowered** → No interference!

### Gravity Considerations

**Important:** When you stop a joint controller, that joint becomes unpowered (effort = 0). For joints affected by gravity:

#### Shoulder Pitch (Joint 1)
- **Problem:** Arm will droop when unpowered
- **Solution 1:** Manually support the arm while tuning other joints
- **Solution 2:** Set robot to "safe" pose before switching controllers:

```bash
# Example: Move arm to upright position before tuning wrist
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{
  joint_names: [base_rotation_joint, shoulder_pitch_joint, elbow_pitch_joint, wrist_pitch_joint, wrist_roll_joint],
  points: [{
    positions: [0.0, 1.57, 0.0, 0.0, 0.0],
    time_from_start: {sec: 2}
  }]
}"

# Then switch to individual controller
```

#### Other Joints (Base, Elbow, Wrist)
- Less affected by gravity when shoulder is upright
- Should stay in place when unpowered (friction/damping in simulation)

---

## Tuning Order Recommendation

### Option A: Proximal to Distal (Base → Wrist)

**Recommended for most cases:**

1. **base_rotation_joint** (Joint 0) - Least affected by other joints
2. **shoulder_pitch_joint** (Joint 1) - Tune with arm hanging down
3. **elbow_pitch_joint** (Joint 2) - Tune with shoulder locked
4. **wrist_pitch_joint** (Joint 3) - Tune with elbow locked
5. **wrist_roll_joint** (Joint 4) - Lightest, fastest joint

**Advantage:** Gravity effects decrease as you move distal.

### Option B: Least to Most Critical

1. **wrist_roll_joint** (Joint 4) - Easiest (no gravity, light)
2. **wrist_pitch_joint** (Joint 3) - Simple dynamics
3. **base_rotation_joint** (Joint 0) - No gravity, just inertia
4. **elbow_pitch_joint** (Joint 2) - Moderate gravity
5. **shoulder_pitch_joint** (Joint 1) - Most challenging (gravity + load)

**Advantage:** Build confidence with easy joints first.

---

## Checking Active Controllers

### List All Controllers

```bash
ros2 control list_controllers
```

**Output example:**
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] inactive
base_rotation_joint_position_controller[joint_trajectory_controller/JointTrajectoryController] active
```

### Check Which Joints Are Claimed

```bash
ros2 control list_hardware_interfaces
```

**Output shows:** Which controller claims which interface (position/velocity/effort).

---

## Troubleshooting

### Problem: "Controller already claimed"

```
Error: base_rotation_joint/effort already claimed by joint_trajectory_controller
```

**Solution:** Stop the multi-joint controller first:
```bash
ros2 control switch_controllers --stop-controllers joint_trajectory_controller
```

### Problem: PID tuner says "No controllers available"

**Solution:** Make sure you started a single-joint controller:
```bash
ros2 control list_controllers  # Check if controller is active
```

### Problem: Joint immediately drops when switched

**Cause:** Gravity pulling on unpowered joints (especially shoulder).

**Solutions:**
1. Manually support the arm
2. Use a "safe pose" before switching (upright position)
3. Tune in simulation first (Gazebo has no real gravity effect on you!)

### Problem: test_trajectory.py moves wrong joint

**Cause:** Joint number mismatch.

**Solution:** Check mapping:
- Joint 0 = base_rotation
- Joint 1 = shoulder_pitch
- Joint 2 = elbow_pitch
- Joint 3 = wrist_pitch
- Joint 4 = wrist_roll

---

## Advanced: Keep Other Joints Powered at Fixed Position

If you want other joints to **hold position** while tuning one joint:

### Option 1: Use Group Position Controller

```bash
# This is more complex - requires custom setup
# Not recommended for initial tuning
```

### Option 2: Use MoveIt with Constraints (Advanced)

```bash
# Plan trajectories that only move one joint
# More complex - use after individual tuning
```

**Recommendation:** Stick with individual controllers for isolation. It's simpler and more reliable.

---

## Summary Cheat Sheet

```bash
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  SINGLE JOINT TUNING CHEAT SHEET
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# 1. Launch system
ros2 launch humanoid_arm_bringup full_system.launch.py

# 2. Stop multi-joint controller
ros2 control switch_controllers \
  --stop-controllers joint_trajectory_controller

# 3. Start single-joint controller (example: base_rotation)
ros2 control switch_controllers \
  --start-controllers base_rotation_joint_position_controller

# 4. Launch PID tuner
ros2 run humanoid_arm_control pid_tuner_gui.py

# 5. Test isolated joint
python3 humanoid_arm_control/scripts/test_trajectory.py \
  --joint 0 --type step --amplitude 0.5 --reset

# 6. Switch to next joint
ros2 control switch_controllers \
  --stop-controllers base_rotation_joint_position_controller \
  --start-controllers shoulder_pitch_joint_position_controller

# 7. Repeat for each joint...

# 8. When done, return to multi-joint mode
ros2 control switch_controllers \
  --stop-controllers wrist_roll_joint_position_controller \
  --start-controllers joint_trajectory_controller
```

---

## Configuration Files

Individual joint controller configs are in [controllers.yaml:155-255](humanoid_arm_control/config/controllers.yaml#L155-255):

```yaml
base_rotation_joint_position_controller:
  ros__parameters:
    joints:
      - base_rotation_joint  # ONLY this joint!
    command_interfaces:
      - effort
    gains:
      base_rotation_joint:
        p: 200.0
        # ... (same baseline as main controller)
```

**Important:** Individual controllers have **same baseline PID values** as main controller. When you tune, update both:
1. Individual controller (for isolated tuning)
2. Main trajectory controller (for coordinated motion)

---

## PID Tuner GUI: Which Controller?

The PID tuner GUI automatically detects the **active** controller:

- If `base_rotation_joint_position_controller` is active → GUI shows only base_rotation
- If `joint_trajectory_controller` is active → GUI shows all 5 joints

**You don't need to specify!** Just switch controllers, and the GUI adapts.

---

Good luck with single-joint tuning! This method gives you **complete isolation** and **clean data** for each joint. 🎯
