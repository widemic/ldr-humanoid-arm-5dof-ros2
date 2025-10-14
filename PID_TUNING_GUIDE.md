# PID Tuning Guide - Isolated Single-Joint Method

**Engineering-Grade Approach:** Use `pid_tuning.launch.py` for proper isolation.

## ✅ Correct Workflow

### **Tune Joint 0 (base_rotation)**

```bash
# Terminal 1: Launch with joint 0 active (others unpowered)
ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint:=0

# Terminal 2: Launch PID tuner GUI (will automatically find active controller)
ros2 run humanoid_arm_control pid_tuner_gui.py

# Terminal 3: Test trajectory
python3 humanoid_arm_control/scripts/test_trajectory.py \
  --joint 0 --type step --amplitude 0.5 --reset
```

**Result:**
- ✅ Only joint 0 is powered and active
- ✅ Joints 1, 2, 3, 4 are unpowered (no interference)
- ✅ PID tuner GUI works immediately
- ✅ Clean, isolated tuning environment

---

### **Tune Joint 1 (shoulder_pitch)**

```bash
# Terminal 1: Launch with joint 1 active
ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint:=1

# Terminal 2: PID tuner GUI
ros2 run humanoid_arm_control pid_tuner_gui.py

# Terminal 3: Test
python3 humanoid_arm_control/scripts/test_trajectory.py \
  --joint 1 --type step --amplitude 0.5 --reset
```

---

### **Tune Joint 2 (elbow_pitch)**

```bash
ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint:=2
ros2 run humanoid_arm_control pid_tuner_gui.py
python3 humanoid_arm_control/scripts/test_trajectory.py --joint 2 --type step --amplitude 0.5 --reset
```

---

### **Tune Joint 3 (wrist_pitch)**

```bash
ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint:=3
ros2 run humanoid_arm_control pid_tuner_gui.py
python3 humanoid_arm_control/scripts/test_trajectory.py --joint 3 --type step --amplitude 0.5 --reset
```

---

### **Tune Joint 4 (wrist_roll)**

```bash
ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint:=4
ros2 run humanoid_arm_control pid_tuner_gui.py
python3 humanoid_arm_control/scripts/test_trajectory.py --joint 4 --type step --amplitude 0.5 --reset
```

---

## Joint Mapping

| joint:= | Joint Number | Joint Name | Controller |
|---------|--------------|------------|------------|
| 0 | 0 | base_rotation_joint | base_rotation_joint_position_controller |
| 1 | 1 | shoulder_pitch_joint | shoulder_pitch_joint_position_controller |
| 2 | 2 | elbow_pitch_joint | elbow_pitch_joint_position_controller |
| 3 | 3 | wrist_pitch_joint | wrist_pitch_joint_position_controller |
| 4 | 4 | wrist_roll_joint | wrist_roll_joint_position_controller |

---

## How It Works

### What `pid_tuning.launch.py joint:=0` Does:

1. **Starts Gazebo** with physics simulation
2. **Loads 5 individual joint controllers:**
   - `base_rotation_joint_position_controller` → **ACTIVE** ✅
   - `shoulder_pitch_joint_position_controller` → **INACTIVE** 💤
   - `elbow_pitch_joint_position_controller` → **INACTIVE** 💤
   - `wrist_pitch_joint_position_controller` → **INACTIVE** 💤
   - `wrist_roll_joint_position_controller` → **INACTIVE** 💤
3. **Only joint 0 is powered** → Clean isolation!
4. **PID tuner GUI connects automatically** → No manual switching needed!

---

## Additional Options

### Disable Gazebo GUI (Headless)

```bash
ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint:=0 gazebo_gui:=false
```

### Use Different World

```bash
ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint:=0 world_file:=contact_manipulation_arena.sdf
```

### Combined Options

```bash
ros2 launch humanoid_arm_bringup pid_tuning.launch.py \
  joint:=0 \
  gazebo_gui:=false \
  world_file:=empty_world.sdf
```

---

## Verification Commands

### Check Which Controllers Are Active

```bash
ros2 control list_controllers
```

**Expected output (joint:=0):**
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
base_rotation_joint_position_controller[joint_trajectory_controller/JointTrajectoryController] active
shoulder_pitch_joint_position_controller[joint_trajectory_controller/JointTrajectoryController] inactive
elbow_pitch_joint_position_controller[joint_trajectory_controller/JointTrajectoryController] inactive
wrist_pitch_joint_position_controller[joint_trajectory_controller/JointTrajectoryController] inactive
wrist_roll_joint_position_controller[joint_trajectory_controller/JointTrajectoryController] inactive
```

### Check Hardware Interfaces

```bash
ros2 control list_hardware_interfaces
```

Should show only joint 0 effort interface is claimed:
```
base_rotation_joint/effort [available] [claimed]
shoulder_pitch_joint/effort [available] [unclaimed]
elbow_pitch_joint/effort [available] [unclaimed]
wrist_pitch_joint/effort [available] [unclaimed]
wrist_roll_joint/effort [available] [unclaimed]
```

---

## Why This Approach is Better (Engineering Perspective)

### ❌ **Using full_system.launch.py:**
- Loads MoveIt (unnecessary overhead)
- Loads RViz (unnecessary overhead)
- Loads move_group node (running in background)
- Starts with multi-joint controller (need manual switching)
- More complexity, more things to go wrong

### ✅ **Using pid_tuning.launch.py:**
- **Minimal:** Only Gazebo + ros2_control
- **Isolated:** One joint active, others unpowered
- **Automatic:** Correct controller activated on launch
- **Clean:** No MoveIt interference
- **Fast:** Lighter, starts quicker
- **Professional:** Dedicated tool for dedicated task

---

## Tuning Workflow Summary

```bash
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  COMPLETE PID TUNING WORKFLOW
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# 1. Launch for joint 0
ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint:=0

# 2. Start GUI + test (in separate terminals)
ros2 run humanoid_arm_control pid_tuner_gui.py
python3 humanoid_arm_control/scripts/test_trajectory.py --joint 0 --type step --amplitude 0.5 --reset

# 3. Tune P/I/D until satisfied

# 4. Kill launch (Ctrl+C), move to next joint
ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint:=1

# 5. Repeat for joints 1, 2, 3, 4

# 6. When all joints tuned, test coordinated motion:
ros2 launch humanoid_arm_bringup full_system.launch.py
```

---

## Baseline PID Values

Start with these conservative values (already in controllers.yaml):

| Joint | P | D | I | i_clamp | ff_vel |
|-------|---|---|---|---------|--------|
| base_rotation | 200 | 15 | 2.0 | 30 | 0.0 |
| shoulder_pitch | 250 | 18 | 3.0 | 35 | 0.0 |
| elbow_pitch | 150 | 12 | 2.0 | 20 | 0.0 |
| wrist_pitch | 120 | 10 | 1.5 | 18 | 0.0 |
| wrist_roll | 80 | 7 | 1.0 | 8 | 0.0 |

See [PID_BASELINE.md](PID_BASELINE.md) for complete documentation.

---

## Troubleshooting

### Problem: PID tuner GUI shows "Failed to get parameters"

**Cause:** No active controller found.

**Solution:** Make sure you specified `joint:=X` argument:
```bash
# Wrong:
ros2 launch humanoid_arm_bringup pid_tuning.launch.py  # ❌ No joint specified

# Correct:
ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint:=0  # ✅ Joint 0 active
```

### Problem: Wrong joint moves

**Cause:** Joint number mismatch between launch and test script.

**Solution:** Ensure consistency:
```bash
# Launch joint 0
ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint:=0

# Test joint 0 (same number!)
python3 humanoid_arm_control/scripts/test_trajectory.py --joint 0 ...
```

### Problem: Arm droops (especially shoulder)

**Cause:** Unpowered joints affected by gravity.

**Solution:** This is expected! Only the active joint is powered. Support arm manually if needed.

---

## Related Documentation

- [PID_BASELINE.md](PID_BASELINE.md) - Baseline PID values and philosophy
- [ACTUATOR_LIMITS.md](ACTUATOR_LIMITS.md) - Motor specifications
- [SINGLE_JOINT_TUNING.md](SINGLE_JOINT_TUNING.md) - Alternative methods
- [TROUBLESHOOTING_PID_TUNER.md](TROUBLESHOOTING_PID_TUNER.md) - Common issues

---

## Summary

**Single command for isolated single-joint tuning:**

```bash
ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint:=0
```

**Clean, professional, engineering-grade approach.** ✅
