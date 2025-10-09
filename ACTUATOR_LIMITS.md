# Actuator Limits Configuration

**Last Updated:** 2025-10-09

This document explains how actuator specifications are managed and referenced throughout the ROS2 workspace.

## Single Source of Truth

**Primary Reference:** [humanoid_arm_control/config/actuator_specs.yaml](humanoid_arm_control/config/actuator_specs.yaml)

All motor specifications (torque, speed, electrical parameters, PID gains) are defined in this single file and referenced throughout the workspace.

## Motor Specifications Summary

| Joint | Motor Model | Max Torque (Nm) | Max Speed (rad/s) | Max Speed (rpm) | Gear Ratio |
|-------|-------------|-----------------|-------------------|-----------------|------------|
| base_rotation_joint | ROBSTRIDE 04 | 120.0 | 20.94 | 200 | 9.0 |
| shoulder_pitch_joint | ROBSTRIDE 04 | 120.0 | 20.94 | 200 | 9.0 |
| elbow_pitch_joint | ROBSTRIDE 03 | 60.0 | 20.42 | 195 | 9.0 |
| wrist_pitch_joint | ROBSTRIDE 03 | 60.0 | 20.42 | 195 | 9.0 |
| wrist_roll_joint | ROBSTRIDE 02 | 17.0 | 42.94 | 410 | 7.75 |

## Configuration Files Reference

### 1. URDF (Robot Model)
**File:** [humanoid_arm_description/urdf/humanoid_arm_5dof_macro.urdf.xacro](humanoid_arm_description/urdf/humanoid_arm_5dof_macro.urdf.xacro)

**How it references actuator_specs.yaml:**
```xml
<!-- Line 4: Load actuator specifications from YAML -->
<xacro:property name="actuator_specs" value="${xacro.load_yaml('$(find humanoid_arm_control)/config/actuator_specs.yaml')}" />

<!-- Example joint limit using loaded specs (line 114-115) -->
<limit lower="-3.14" upper="3.14"
       effort="${actuator_specs['actuators']['base_rotation_joint']['motor']['max_torque']}"
       velocity="${actuator_specs['actuators']['base_rotation_joint']['motor']['max_speed']}"/>
```

**Status:** ✅ Fully automated - URDF loads directly from actuator_specs.yaml

---

### 2. MoveIt Joint Limits
**File:** [humanoid_arm_moveit_config/config/joint_limits.yaml](humanoid_arm_moveit_config/config/joint_limits.yaml)

**How it references actuator_specs.yaml:**
- **Manual reference** - Values copied from actuator_specs.yaml with comments
- Comments indicate source: `# ROBSTRIDE 04: 20.94 rad/s (200 rpm) - from actuator_specs.yaml`

**Current Values:**
```yaml
base_rotation_joint:
  max_velocity: 20.94  # ROBSTRIDE 04 (from actuator_specs.yaml)
shoulder_pitch_joint:
  max_velocity: 20.94  # ROBSTRIDE 04 (from actuator_specs.yaml)
elbow_pitch_joint:
  max_velocity: 20.42  # ROBSTRIDE 03 (from actuator_specs.yaml)
wrist_pitch_joint:
  max_velocity: 20.42  # ROBSTRIDE 03 (from actuator_specs.yaml)
wrist_roll_joint:
  max_velocity: 42.94  # ROBSTRIDE 02 (from actuator_specs.yaml)
```

**Status:** ✅ Updated with correct values and reference comments

---

### 3. ros2_control Hardware Interface
**File:** [humanoid_arm_description/config/ros2_control.yaml](humanoid_arm_description/config/ros2_control.yaml)

**How it references actuator_specs.yaml:**
- **Manual reference** - Values copied from actuator_specs.yaml with comments
- Header comment: `# NOTE: Velocity and acceleration values reference actuator_specs.yaml`

**Current Values:**
```yaml
joint_limits:
  base_rotation_joint:
    max_velocity: 20.94  # ROBSTRIDE 04: 20.94 rad/s (200 rpm) - from actuator_specs.yaml
  # ... (similar for other joints)

robstride_motors:
  base_rotation_joint:
    motor_type: "ROBSTRIDE 04"  # Updated from RMD-X8-PRO
    max_torque: 120.0  # Nm - from actuator_specs.yaml
    max_speed: 20.94   # rad/s (200 rpm) - from actuator_specs.yaml
  # ... (similar for other joints)
```

**Status:** ✅ Updated with correct values, motor types, and gear ratios

---

### 4. Controller PID Gains
**File:** [humanoid_arm_control/config/controllers.yaml](humanoid_arm_control/config/controllers.yaml)

**How it references actuator_specs.yaml:**
- PID gains in `actuator_specs.yaml` serve as **recommended starting values**
- `controllers.yaml` contains **actively tuned values** for simulation/hardware
- PID output (torque) is automatically limited by URDF effort limits

**Current PID Values (Line 75-103):**
```yaml
joint_trajectory_controller:
  gains:
    base_rotation_joint:
      p: 1200.0
      d: 50.0
      i: 20.0
      i_clamp: 100.0
      ff_velocity_scale: 1.0
    shoulder_pitch_joint:
      p: 1500.0  # Highest gains due to gravity loading
      d: 60.0
      i: 25.0
      i_clamp: 120.0
      ff_velocity_scale: 1.2
    # ... (other joints with scaled values)
```

**PID Output Limits:**
- PID controller calculates torque: `τ = P*e + D*ė + I*∫e + FF*v̇`
- Output is automatically clamped to URDF effort limits (from actuator_specs.yaml)
- Example: `base_rotation_joint` PID output limited to ±120 Nm (ROBSTRIDE 04 max)

**Status:** ✅ High gains tuned for aggressive tracking, torque limits enforced by URDF

---

## PID Tuning Safety

### Torque Saturation Check

With current gains, **maximum possible PID torque output** (worst case: full position error at velocity limits):

| Joint | Motor Max (Nm) | PID P Gain | Max Error (rad) | Theoretical Max τ (Nm) | Saturates? |
|-------|----------------|------------|-----------------|------------------------|------------|
| base_rotation | 120 | 1200 | 6.28 | 7536 | ✅ Yes (63×) |
| shoulder_pitch | 120 | 1500 | 3.6 | 5400 | ✅ Yes (45×) |
| elbow_pitch | 60 | 1000 | 6.28 | 6280 | ✅ Yes (105×) |
| wrist_pitch | 60 | 800 | 3.0 | 2400 | ✅ Yes (40×) |
| wrist_roll | 17 | 600 | 6.28 | 3768 | ✅ Yes (222×) |

**Conclusion:** High PID gains are **safe** because:
1. ✅ URDF enforces torque limits from actuator_specs.yaml
2. ✅ Controllers clamp output to effort limits automatically
3. ✅ Real hardware will never see torques exceeding motor ratings

### Why High Gains?

These aggressive gains are intentional for **effort-based control** in Gazebo:
- Effort interface → PID calculates torque → Gazebo applies physics → realistic dynamics
- High P gains = fast response, aggressive tracking
- Without high gains, tracking is too slow and MoveIt trajectories violate tolerances
- Torque saturation is expected and desirable for fast movements

## History of Corrections (2025-10-09)

### Previous (INCORRECT) Values
| Joint | Old max_velocity (ros2_control.yaml) | Old max_velocity (joint_limits.yaml) | Severity |
|-------|--------------------------------------|--------------------------------------|----------|
| base_rotation | 2.5 rad/s | 20.0 rad/s | ❌ 8× too slow |
| shoulder_pitch | 2.5 rad/s | 20.0 rad/s | ❌ 8× too slow |
| elbow_pitch | 1.4 rad/s | 20.0 rad/s | ❌ 14× too slow |
| wrist_pitch | 1.4 rad/s | 20.0 rad/s | ❌ 14× too slow |
| wrist_roll | 0.7 rad/s | 2.0 rad/s | ❌ 61× too slow! |

### Motor Type Naming
- **Old:** RMD-X8-PRO, RMD-X6, RMD-X4 (generic Robstride naming)
- **New:** ROBSTRIDE 04, ROBSTRIDE 03, ROBSTRIDE 02 (actual purchased models)

### Gear Ratios
- **Old:** Assumed 6.0, 8.0, 4.0
- **New:** Actual 9.0, 9.0, 9.0, 9.0, 7.75 (from datasheets in actuator_specs.yaml)

## Updating Actuator Specifications

### Step 1: Update actuator_specs.yaml
Edit [humanoid_arm_control/config/actuator_specs.yaml](humanoid_arm_control/config/actuator_specs.yaml):
```yaml
actuators:
  base_rotation_joint:
    motor:
      max_torque: 120.0  # Update this
      max_speed: 20.94   # And this
```

### Step 2: Rebuild (URDF automatically updates)
```bash
colcon build --packages-select humanoid_arm_description --symlink-install
source install/setup.bash
```

The URDF will automatically load new values on next launch! ✅

### Step 3: Manually sync other files (if needed)
Check and update if values changed significantly:
1. [joint_limits.yaml](humanoid_arm_moveit_config/config/joint_limits.yaml) - MoveIt limits
2. [ros2_control.yaml](humanoid_arm_description/config/ros2_control.yaml) - Hardware interface limits

### Step 4: Verify URDF generated correctly
```bash
xacro humanoid_arm_description/urdf/humanoid_arm_5dof_ros2_control.urdf.xacro > /tmp/test.urdf
grep "effort=" /tmp/test.urdf | head -5
```

Should show values from actuator_specs.yaml.

## Verification Checklist

Before PID tuning or hardware integration:

- [ ] `actuator_specs.yaml` has correct motor datasheets
- [ ] `joint_limits.yaml` max_velocity matches actuator_specs
- [ ] `ros2_control.yaml` max_velocity matches actuator_specs
- [ ] `ros2_control.yaml` motor types match actuator_specs
- [ ] URDF builds without errors: `colcon build --packages-select humanoid_arm_description`
- [ ] Test launch: `ros2 launch humanoid_arm_bringup full_system.launch.py`
- [ ] Verify limits: `ros2 control list_hardware_interfaces`

## Commands to Verify Limits at Runtime

```bash
# Check URDF-defined effort limits
ros2 topic echo /robot_description --once | grep -A 2 "limit effort"

# Check active controller parameters
ros2 param list /joint_trajectory_controller | grep gains

# Monitor actual torque commands
ros2 topic echo /joint_states --field effort

# Test velocity limits with joint commands
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory ...
```

## Future: Hardware Integration

When transitioning to real Robstride motors:

1. ✅ **actuator_specs.yaml already has correct values** (motor specs from datasheets)
2. ✅ **URDF already references actuator_specs.yaml** (torque/speed limits auto-enforced)
3. ⚠️ **PID gains will need retuning** (current gains are for Gazebo simulation)
4. ⚠️ **Implement CAN hardware interface** using specs in actuator_specs.yaml

The velocity/torque limits are **already correct** for real hardware! 🎉

## References

- **Robstride Datasheets:** See `actuator_specs.yaml` motor specifications
- **PID Tuning Guide:** [PID_README.md](PID_README.md)
- **ROS2 Control Documentation:** https://control.ros.org/
- **URDF Xacro Tutorial:** http://wiki.ros.org/xacro
