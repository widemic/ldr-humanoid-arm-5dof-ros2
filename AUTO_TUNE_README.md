# Automatic PID Tuning

Automatic PID gain calculation using relay feedback method (Ziegler-Nichols).

---

## How It Works

1. **Relay Test** - Applies on/off control to make the joint oscillate
2. **Oscillation Detection** - Measures period and amplitude of oscillations
3. **Calculate Gains** - Uses Ziegler-Nichols formulas to compute PID gains
4. **Test & Validate** - Applies gains and tests with step response

**Method:** Relay Feedback + Ziegler-Nichols
- Ultimate gain: `Ku = (4 * relay_amplitude) / (π * oscillation_amplitude)`
- P gain: `Kp = 0.6 * Ku`
- I gain: `Ki = 1.2 * Ku / Tu`
- D gain: `Kd = 0.075 * Ku * Tu`

---

## Quick Start

```bash
# 1. Launch calibration system for the joint you want to tune
ros2 launch humanoid_arm_bringup pid_calibration.launch.py joint:=base_rotation_joint

# 2. In another terminal, run auto-tuner
ros2 run humanoid_arm_control auto_tune_pid.py --joint base_rotation_joint

# 3. Wait ~30-60 seconds while it:
#    - Runs relay test
#    - Detects oscillations
#    - Calculates gains
#    - Tests performance

# 4. If successful, gains are automatically applied!
#    Test manually to verify
```

---

## Auto-Tune All Joints

```bash
# Tune each joint one by one

# Base rotation
ros2 launch humanoid_arm_bringup pid_calibration.launch.py joint:=base_rotation_joint
ros2 run humanoid_arm_control auto_tune_pid.py --joint base_rotation_joint

# Shoulder pitch
ros2 launch humanoid_arm_bringup pid_calibration.launch.py joint:=shoulder_pitch_joint
ros2 run humanoid_arm_control auto_tune_pid.py --joint shoulder_pitch_joint

# Elbow pitch
ros2 launch humanoid_arm_bringup pid_calibration.launch.py joint:=elbow_pitch_joint
ros2 run humanoid_arm_control auto_tune_pid.py --joint elbow_pitch_joint

# Wrist pitch
ros2 launch humanoid_arm_bringup pid_calibration.launch.py joint:=wrist_pitch_joint
ros2 run humanoid_arm_control auto_tune_pid.py --joint wrist_pitch_joint

# Wrist roll
ros2 launch humanoid_arm_bringup pid_calibration.launch.py joint:=wrist_roll_joint
ros2 run humanoid_arm_control auto_tune_pid.py --joint wrist_roll_joint
```

---

## What to Expect

### Phase 1: Relay Test (10-20 seconds)
```
[INFO] Running relay test...
[INFO] Oscillation found! Period=0.523s, Amp=0.145rad
```

The joint will oscillate as the auto-tuner applies on/off control.

### Phase 2: Calculation
```
[INFO] Ku=234.56, Tu=0.523
[INFO] Calculated: P=140.73, I=322.45, D=9.21
```

Computes PID gains from oscillation data.

### Phase 3: Testing (5 seconds)
```
[INFO] Testing step to 0.5rad...
[INFO] Overshoot=12.3%, Final error=0.0023rad
```

Tests with a step command to verify performance.

### Success
```
============================================================
AUTO-TUNE COMPLETE!
  P=140.73
  I=322.45
  D=9.21
============================================================
```

Gains are now active! Test manually to confirm good behavior.

---

## Save Tuned Gains

If you're happy with the auto-tuned gains, save them to `controllers.yaml`:

```yaml
# humanoid_arm_control/config/controllers.yaml

base_rotation_joint_position_controller:
  ros__parameters:
    gains:
      base_rotation_joint:
        p: 140.73      # Auto-tuned value
        i: 322.45      # Auto-tuned value
        d: 9.21        # Auto-tuned value
        i_clamp: 644.9 # Auto-calculated (I * 2)
        ff_velocity_scale: 0.1
```

Then rebuild:
```bash
colcon build --packages-select humanoid_arm_control
```

---

## Troubleshooting

### "Relay test failed"
- **Cause:** Oscillations not detected
- **Fix:**
  - Increase relay amplitude (edit script: `self.relay_amplitude = 0.5`)
  - Run longer (currently 30s)
  - Check joint can move freely in Gazebo

### "Overshoot too high" or "Final error large"
- **Cause:** Calculated gains too aggressive or too weak
- **Fix:**
  - Use conservative Z-N formulas (edit script)
  - Manual fine-tuning after auto-tune
  - Adjust I_clamp to prevent integral windup

### Joint oscillates violently
- **Cause:** Gains too high
- **Fix:**
  - Reduce gains manually: `ros2 run humanoid_arm_control pid_tuner_gui.py`
  - Reduce relay amplitude
  - Use "no overshoot" Z-N formulas

---

## Theory: Why This Works

**Relay Feedback Method:**
1. Apply on/off (relay) control around setpoint
2. System naturally oscillates at its "ultimate frequency"
3. Measure oscillation period (Tu) and amplitude (a)
4. Calculate "ultimate gain" Ku from relay amplitude and oscillation amplitude

**Ziegler-Nichols Tuning Rules:**
- Based on Ku and Tu, empirically derived PID formulas
- Classic Z-N (used here): good general performance, some overshoot
- No overshoot Z-N: more conservative, slower response
- Some overshoot Z-N: middle ground

**Why closed-loop?**
- Uses actual system response (includes all real dynamics)
- Automatically adapts to:
  - Joint inertia
  - Friction
  - Motor characteristics
  - Load variations
- No need to manually model the system!

---

## Comparison: Manual vs Auto-Tune

| Aspect | Manual Tuning | Auto-Tune |
|--------|---------------|-----------|
| Time | 5-15 min/joint | 30-60 sec/joint |
| Skill required | High | Low |
| Repeatability | Variable | Consistent |
| Optimality | Can be better | Good starting point |
| Safety | User controlled | Can be aggressive |
| Adaptability | No | Yes (re-run anytime) |

**Recommendation:**
- Use auto-tune for initial gains
- Fine-tune manually if needed
- Re-run auto-tune if hardware changes (motors, load, etc.)

---

## Advanced: Modify Tuning Rules

Edit `auto_tune_pid.py`, function `calculate_gains()`:

```python
# Current (Classic Ziegler-Nichols)
Kp = 0.6 * Ku
Ki = 1.2 * Ku / Tu
Kd = 0.075 * Ku * Tu

# Conservative (No Overshoot)
Kp = 0.2 * Ku
Ki = 0.4 * Ku / Tu
Kd = 0.066 * Ku * Tu

# Middle Ground (Some Overshoot)
Kp = 0.33 * Ku
Ki = 0.66 * Ku / Tu
Kd = 0.11 * Ku * Tu
```

Choose based on your priority:
- **Classic** - Fast response, some overshoot
- **Conservative** - No overshoot, slower
- **Middle** - Balanced

---

## See Also

- [PID_CALIBRATION_GUIDE.md](PID_CALIBRATION_GUIDE.md) - Manual PID tuning guide
- [PID_README.md](PID_README.md) - PID theory and tools
- [SOFTWARE_GUIDE.md](SOFTWARE_GUIDE.md) - All ROS2 tools
