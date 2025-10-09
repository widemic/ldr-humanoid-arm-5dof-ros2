# PID Baseline Configuration

**Date:** 2025-10-09
**Status:** Conservative baseline for systematic tuning

## Overview

This document describes the **baseline PID configuration** created for systematic tuning. These values are **conservative** and designed as a safe starting point, not optimized for performance.

## Baseline Philosophy

### Why Conservative?

1. **Safety first** - Start slow, increase gradually
2. **Predictable behavior** - Easier to see effects of parameter changes
3. **Avoid saturation** - Keep torque output within reasonable range most of the time
4. **Systematic tuning** - Clear baseline to measure improvements from

### Expected Behavior

With these baseline values:
- ✅ System should be **stable** (no wild oscillations)
- ✅ Response will be **slower** than optimal (intentional)
- ✅ You may see **slight overshoot** (5-10%)
- ✅ Tracking errors will be **larger** during motion (acceptable for baseline)
- ⚠️ Trajectories may be **too slow** for aggressive MoveIt planning (tune up from here)

## Baseline PID Values

### Summary Table

| Joint | Motor | Max Torque | P | D | I | i_clamp | ff_vel |
|-------|-------|------------|---|---|---|---------|--------|
| base_rotation | ROBSTRIDE 04 | 120 Nm | 200 | 15 | 2.0 | 30 | 0.0 |
| shoulder_pitch | ROBSTRIDE 04 | 120 Nm | 250 | 18 | 3.0 | 35 | 0.0 |
| elbow_pitch | ROBSTRIDE 03 | 60 Nm | 150 | 12 | 2.0 | 20 | 0.0 |
| wrist_pitch | ROBSTRIDE 03 | 60 Nm | 120 | 10 | 1.5 | 18 | 0.0 |
| wrist_roll | ROBSTRIDE 02 | 17 Nm | 80 | 7 | 1.0 | 8 | 0.0 |

### Design Rationale

#### P Gains (Proportional)
```
base_rotation:   P = 200  (1.67 Nm/rad error)
shoulder_pitch:  P = 250  (2.08 Nm/rad error) - higher for gravity
elbow_pitch:     P = 150  (2.50 Nm/rad error)
wrist_pitch:     P = 120  (2.00 Nm/rad error)
wrist_roll:      P = 80   (4.71 Nm/rad error) - lighter load
```

**Reasoning:**
- Conservative: 10-20% of previous aggressive values (1200 → 200)
- Scaled by motor torque: bigger motors get higher P
- Shoulder slightly higher to counteract gravity
- Should produce stable response without excessive oscillations

#### D Gains (Derivative)
```
D ≈ P/13 to P/14 ratio across all joints
```

**Reasoning:**
- Classic damping ratio (critically damped to slightly underdamped)
- Prevents overshoot while maintaining reasonable response
- Ratio maintained across joints for consistent feel

#### I Gains (Integral)
```
I = 1.0 to 3.0 (very small, P/100 to P/83 ratio)
```

**Reasoning:**
- Start very small - only for steady-state error correction
- Shoulder gets slightly higher (3.0) due to gravity sag
- Can increase later if position errors persist

#### i_clamp (Integral Clamp)
```
~25-33% of max motor torque
base_rotation:   30 Nm  (25% of 120 Nm)
shoulder_pitch:  35 Nm  (29% of 120 Nm)
elbow_pitch:     20 Nm  (33% of 60 Nm)
wrist_pitch:     18 Nm  (30% of 60 Nm)
wrist_roll:      8 Nm   (47% of 17 Nm)
```

**Reasoning:**
- Prevents integral windup
- Allows enough authority for steady-state correction
- Smaller motors get higher percentage (less margin)

#### ff_velocity_scale (Feedforward)
```
All joints: 0.0 (disabled)
```

**Reasoning:**
- Start with pure PID (no feedforward)
- Add later if velocity tracking lags
- Easier to tune P/I/D without feedforward interference

## Trajectory Constraints

### Tolerances (Relaxed for Baseline)

```yaml
goal_time: 5.0 seconds                    # Generous time allowance
stopped_velocity_tolerance: 2.0 rad/s     # Relaxed

base_rotation:   trajectory: 1.0 rad,  goal: 0.15 rad
shoulder_pitch:  trajectory: 1.0 rad,  goal: 0.15 rad
elbow_pitch:     trajectory: 1.0 rad,  goal: 0.15 rad
wrist_pitch:     trajectory: 1.0 rad,  goal: 0.15 rad
wrist_roll:      trajectory: 1.5 rad,  goal: 0.20 rad (fastest joint)
```

**Previous (aggressive):**
- goal_time: 3.0 seconds
- trajectory: 0.5 rad
- goal: 0.2 rad

**Changes:**
- Increased goal_time (3.0 → 5.0) - more time to reach goal
- Relaxed trajectory tolerance (0.5 → 1.0 rad) - won't abort during motion
- Tighter goal tolerance (0.2 → 0.15 rad) - still expect accurate final position

## Comparison: Previous vs Baseline

### Before (Aggressive, Possibly Unstable)

```yaml
base_rotation:   P=1200,  D=50,  I=20,  ff=1.0
shoulder_pitch:  P=1500,  D=60,  I=25,  ff=1.2
elbow_pitch:     P=1000,  D=40,  I=15,  ff=1.0
wrist_pitch:     P=800,   D=35,  I=12,  ff=0.8
wrist_roll:      P=600,   D=30,  I=10,  ff=0.6
```

**Characteristics:**
- Very aggressive (6-7.5× higher P gains)
- High feedforward
- Tight tolerances
- Fast goal_time (3 seconds)
- **May have been tuned for position interface, not effort control**

### After (Conservative Baseline)

```yaml
base_rotation:   P=200,  D=15,  I=2,   ff=0.0
shoulder_pitch:  P=250,  D=18,  I=3,   ff=0.0
elbow_pitch:     P=150,  D=12,  I=2,   ff=0.0
wrist_pitch:     P=120,  D=10,  I=1.5, ff=0.0
wrist_roll:      P=80,   D=7,   I=1,   ff=0.0
```

**Characteristics:**
- Conservative (83-87% reduction in P)
- No feedforward (start clean)
- Relaxed tolerances
- Longer goal_time (5 seconds)
- **Designed specifically for effort control**

### Gain Reduction Factor

| Joint | Old P | New P | Reduction |
|-------|-------|-------|-----------|
| base_rotation | 1200 | 200 | **6×** |
| shoulder_pitch | 1500 | 250 | **6×** |
| elbow_pitch | 1000 | 150 | **6.7×** |
| wrist_pitch | 800 | 120 | **6.7×** |
| wrist_roll | 600 | 80 | **7.5×** |

## Tuning Procedure from Baseline

### Step 1: Test Baseline (No Changes)

```bash
# Launch full system
ros2 launch humanoid_arm_bringup full_system.launch.py

# Test single joint with step response
python3 humanoid_arm_control/scripts/test_trajectory.py --joint 0 --type step --amplitude 0.5 --reset

# Observe:
# - Is it stable? (no wild oscillations)
# - Is it too slow? (sluggish response)
# - Any steady-state error? (final position offset)
# - Overshoot percentage?
```

### Step 2: Increase P (One Joint at a Time)

```bash
# Start PID tuner GUI
ros2 run humanoid_arm_control pid_tuner_gui.py

# Increase P in steps of 50:
# base_rotation: 200 → 250 → 300 → 350...

# Watch for:
# - Faster response (good)
# - Oscillations starting (means you've gone too far)
# - Overshoot increasing (may need more D)

# Rule of thumb: Stop when you see oscillations, then back off 20%
```

### Step 3: Tune D (After P is Set)

```bash
# If you see oscillations or overshoot:
# - Increase D in steps of 5
# - D should be roughly P/10 to P/20

# Too much D:
# - Sluggish response
# - "Sticky" feeling

# Too little D:
# - Oscillations
# - Overshoot
```

### Step 4: Add I (Only If Needed)

```bash
# If you have steady-state error (doesn't reach goal):
# - Increase I in steps of 0.5
# - Watch i_clamp - may need to increase proportionally

# Warning signs of too much I:
# - Slow oscillations
# - Overshoot that takes long to settle
# - "Wind-up" behavior
```

### Step 5: Add Feedforward (Advanced)

```bash
# If velocity tracking lags (moves slower than commanded):
# - Increase ff_velocity_scale in steps of 0.1
# - Start at 0.1, go up to 0.5 max

# Too much feedforward:
# - Overshoot at start of motion
# - Oscillations during movement
```

### Step 6: Tighten Constraints (After Gains Are Good)

```bash
# Once joints track well:
# - Reduce goal_time (5.0 → 4.0 → 3.0)
# - Tighten trajectory tolerance (1.0 → 0.8 → 0.6)
# - Tighten goal tolerance (0.15 → 0.12 → 0.10)

# Goal: Fast movements without trajectory aborts
```

## Testing Commands

### Single Joint Testing

```bash
# Step response (best for initial tuning)
python3 humanoid_arm_control/scripts/test_trajectory.py --joint 0 --type step --amplitude 0.5 --reset

# Sine wave (for tracking performance)
python3 humanoid_arm_control/scripts/test_trajectory.py --joint 0 --type sine --amplitude 0.5 --frequency 0.5 --reset

# Square wave (for aggressive transitions)
python3 humanoid_arm_control/scripts/test_trajectory.py --joint 0 --type square --amplitude 0.5 --frequency 0.2 --reset

# Continuous testing (for real-time tuning)
python3 humanoid_arm_control/scripts/continuous_test.py --joint 0 --type sine --amplitude 0.5
```

### Multi-Joint Testing

```bash
# MoveIt planning (after individual joints are tuned)
# Use RViz MotionPlanning plugin to test coordinated motion
```

### Monitoring

```bash
# Watch joint states
ros2 topic echo /joint_states

# Watch trajectory errors
ros2 topic echo /joint_trajectory_controller/controller_state --field error.positions

# Watch actual torque commands
ros2 topic echo /joint_states --field effort
```

## Expected Results with Baseline

### Good Signs ✅

- Stable, no oscillations
- Slight overshoot (5-15%) acceptable
- Reaches goal within tolerance
- Smooth acceleration/deceleration
- Torque commands reasonable (<50% of max most of the time)

### Bad Signs ❌ (Need Tuning)

- Very slow response (increase P)
- Large overshoot >20% (increase D)
- Steady-state error >0.1 rad (increase I slightly)
- Oscillations (reduce P or increase D)
- Trajectory aborts with error -4 (relax tolerances)

## Saving Your Tuned Values

When you find good values:

```bash
# PID tuner GUI saves automatically when you click "Save to YAML"
# Or manually edit:
nano humanoid_arm_control/config/controllers.yaml

# Rebuild and restart:
colcon build --packages-select humanoid_arm_control --symlink-install
source install/setup.bash
ros2 launch humanoid_arm_bringup full_system.launch.py
```

## Documentation

Remember to document your final tuned values:

```yaml
# In controllers.yaml, add comments like:
gains:
  base_rotation_joint:
    p: 450.0    # Tuned 2025-10-09: Good response, 10% overshoot
    d: 35.0     # Tuned 2025-10-09: Minimal oscillations
    i: 3.0      # Tuned 2025-10-09: No steady-state error
```

## Next Steps

1. ✅ Baseline is set (current state)
2. ⏭️ Test baseline with step response on each joint
3. ⏭️ Increase P gains systematically until oscillations appear
4. ⏭️ Tune D to eliminate oscillations
5. ⏭️ Add I if steady-state errors persist
6. ⏭️ Test multi-joint trajectories with MoveIt
7. ⏭️ Tighten constraints once performance is good
8. ⏭️ Add feedforward for final velocity tracking improvement

## References

- **Main PID Guide:** [PID_README.md](PID_README.md)
- **Actuator Specs:** [actuator_specs.yaml](humanoid_arm_control/config/actuator_specs.yaml)
- **Controllers Config:** [controllers.yaml](humanoid_arm_control/config/controllers.yaml)
- **Tuning Tools:** PID_README.md section "PID Tuning Tools"

---

**Good luck with tuning! Start slow, increase gradually, and document your findings.** 🎯
