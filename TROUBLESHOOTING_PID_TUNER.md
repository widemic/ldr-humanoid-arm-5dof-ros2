# Troubleshooting PID Tuner GUI

## Error: "Failed to get parameters - timeout or error"

```
[ERROR] [1760048090.198270870] [pid_tuner_gui]: Failed to get parameters - timeout or error (done=False)
```

### **Cause**
The PID tuner GUI can't find an **active** controller to tune.

### **Quick Fix**

You need to **activate a controller** before launching the GUI.

---

## **Solution 1: Start a Single-Joint Controller First**

```bash
# Terminal 1: Launch PID tuning
ros2 launch humanoid_arm_bringup pid_tuning.launch.py

# Terminal 2: Activate a single-joint controller
ros2 control switch_controllers \
  --start-controllers base_rotation_joint_position_controller

# Terminal 3: NOW launch PID tuner GUI
ros2 run humanoid_arm_control pid_tuner_gui.py
```

**Why:** `pid_tuning.launch.py` loads all individual controllers as **inactive**. The GUI needs an **active** controller to read parameters from.

---

## **Solution 2: Use the Helper Script**

```bash
# Terminal 1: Launch PID tuning
ros2 launch humanoid_arm_bringup pid_tuning.launch.py

# Terminal 2: Use helper script (activates controller automatically)
./scripts/tune_single_joint.sh 0

# Terminal 3: Launch PID tuner GUI
ros2 run humanoid_arm_control pid_tuner_gui.py
```

The helper script automatically activates the controller for you!

---

## **Solution 3: Check Which Controllers Are Active**

```bash
# List all controllers and their status
ros2 control list_controllers
```

**Expected output:**
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
base_rotation_joint_position_controller[joint_trajectory_controller/JointTrajectoryController] active
shoulder_pitch_joint_position_controller[joint_trajectory_controller/JointTrajectoryController] inactive
...
```

**You need at least ONE controller to be `active`.**

---

## **What the PID Tuner GUI Looks For**

The GUI searches for active controllers in this order:

1. `joint_trajectory_controller` (multi-joint)
2. `base_rotation_joint_position_controller`
3. `shoulder_pitch_joint_position_controller`
4. `elbow_pitch_joint_position_controller`
5. `wrist_pitch_joint_position_controller`
6. `wrist_roll_joint_position_controller`

**If none are active → timeout error.**

---

## **Complete Workflow (Step-by-Step)**

### **Step 1: Launch Robot**

```bash
ros2 launch humanoid_arm_bringup pid_tuning.launch.py
```

Wait for this output:
```
[INFO] [gz_bridge]: Passing clock from Gazebo to ROS
[INFO] [controller_manager]: Loaded joint_state_broadcaster
[INFO] [controller_manager]: Loaded base_rotation_joint_position_controller (inactive)
[INFO] [controller_manager]: Loaded shoulder_pitch_joint_position_controller (inactive)
...
```

### **Step 2: Verify Controllers Are Loaded**

```bash
ros2 control list_controllers
```

Should show:
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
base_rotation_joint_position_controller[joint_trajectory_controller/JointTrajectoryController] inactive
shoulder_pitch_joint_position_controller[joint_trajectory_controller/JointTrajectoryController] inactive
elbow_pitch_joint_position_controller[joint_trajectory_controller/JointTrajectoryController] inactive
wrist_pitch_joint_position_controller[joint_trajectory_controller/JointTrajectoryController] inactive
wrist_roll_joint_position_controller[joint_trajectory_controller/JointTrajectoryController] inactive
```

### **Step 3: Activate Controller for Joint You Want to Tune**

```bash
# Example: Tune joint 0 (base_rotation)
ros2 control switch_controllers \
  --start-controllers base_rotation_joint_position_controller
```

Verify it's active:
```bash
ros2 control list_controllers | grep base_rotation
```

Should show:
```
base_rotation_joint_position_controller[joint_trajectory_controller/JointTrajectoryController] active
```

### **Step 4: Launch PID Tuner GUI**

```bash
ros2 run humanoid_arm_control pid_tuner_gui.py
```

Should now show:
```
[INFO] Using base_rotation_joint_position_controller
[DEBUG] Requesting parameters: ['gains.base_rotation_joint.p', ...]
[INFO] Current PID values:
  P: 200.0
  I: 2.0
  D: 15.0
  ...
```

### **Step 5: Test Trajectory**

```bash
python3 humanoid_arm_control/scripts/test_trajectory.py \
  --joint 0 --type step --amplitude 0.5 --reset
```

---

## **Common Mistakes**

### ❌ Mistake 1: Launching GUI Before Activating Controller

```bash
# Wrong order:
ros2 launch humanoid_arm_bringup pid_tuning.launch.py
ros2 run humanoid_arm_control pid_tuner_gui.py  # ❌ Error: no active controller
```

**Fix:**
```bash
# Correct order:
ros2 launch humanoid_arm_bringup pid_tuning.launch.py
ros2 control switch_controllers --start-controllers base_rotation_joint_position_controller
ros2 run humanoid_arm_control pid_tuner_gui.py  # ✅ Works
```

### ❌ Mistake 2: Using `full_system.launch.py`

`full_system.launch.py` starts `joint_trajectory_controller` (multi-joint), which works but controls all 5 joints together.

For **single-joint tuning**, use:
- `pid_tuning.launch.py` + individual controllers ✅

### ❌ Mistake 3: Forgetting to Check Controller Status

Always verify:
```bash
ros2 control list_controllers
```

If you see all `inactive`, activate one!

---

## **Alternative: Use CLI PID Tuner**

If the GUI still doesn't work, use the CLI version:

```bash
# Activate controller
ros2 control switch_controllers \
  --start-controllers base_rotation_joint_position_controller

# Set PID values directly
ros2 param set /base_rotation_joint_position_controller gains.base_rotation_joint.p 250.0
ros2 param set /base_rotation_joint_position_controller gains.base_rotation_joint.d 18.0
ros2 param set /base_rotation_joint_position_controller gains.base_rotation_joint.i 3.0

# Read current values
ros2 param get /base_rotation_joint_position_controller gains.base_rotation_joint.p
```

---

## **Check If PID Tuner GUI is Finding Controllers**

Add debug output to see what the GUI is doing:

```bash
# Run with verbose output
ros2 run humanoid_arm_control pid_tuner_gui.py --ros-args --log-level debug
```

Look for:
```
[DEBUG] Checking for controller: joint_trajectory_controller
[DEBUG] Checking for controller: base_rotation_joint_position_controller
[DEBUG] Found active controller: base_rotation_joint_position_controller
```

If you see:
```
[DEBUG] No active controllers found
```

Then you need to activate one first!

---

## **Quick Commands Reference**

```bash
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  PID TUNING QUICK START (CORRECT ORDER)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# 1. Launch robot (controllers load as inactive)
ros2 launch humanoid_arm_bringup pid_tuning.launch.py

# 2. Verify controllers loaded
ros2 control list_controllers

# 3. Activate controller for joint you want to tune
ros2 control switch_controllers \
  --start-controllers base_rotation_joint_position_controller

# 4. Verify it's active
ros2 control list_controllers | grep active

# 5. NOW launch PID tuner GUI
ros2 run humanoid_arm_control pid_tuner_gui.py

# 6. Test trajectory
python3 humanoid_arm_control/scripts/test_trajectory.py \
  --joint 0 --type step --amplitude 0.5 --reset
```

---

## **Summary**

**The error means:** No active controller found.

**Solution:** Activate a controller before launching the GUI.

**Easiest method:** Use `./scripts/tune_single_joint.sh 0` which does it automatically!

---

Good luck! The GUI should work once you have an active controller. 🎯
