# PID Tuning Guide

This guide covers tools for tuning the PID controllers for the humanoid arm's joint trajectory controller.

## Tools

### 1. PID Tuner GUI

Interactive GUI for real-time PID parameter adjustment using sliders.

**Launch:**
```bash
# Make sure robot is running first
ros2 launch humanoid_arm_bringup robot.launch.py

# In another terminal, launch the PID tuner
ros2 run humanoid_arm_control pid_tuner_gui.py
```

**Features:**
- Real-time parameter updates (no restart needed)
- Sliders for P, I, D, I Clamp, and Feedforward Velocity gains
- Select any of the 5 joints
- Apply/Reset buttons
- Visual feedback on parameter changes

**Usage:**
1. Select joint from dropdown
2. Adjust sliders to tune parameters
3. Click "Apply Changes" to update controller
4. Test robot motion and iterate

### 2. Trajectory Test Generator

Generates test trajectories for systematic PID tuning and system identification.

**Launch:**
```bash
# Make sure robot is running first
ros2 launch humanoid_arm_bringup robot.launch.py

# Generate test trajectory
ros2 run humanoid_arm_control test_trajectory.py [OPTIONS]
```

**Trajectory Types:**

#### Step Response
Tests how quickly the joint responds to a position change.
```bash
ros2 run humanoid_arm_control test_trajectory.py \
  --joint 0 \
  --type step \
  --amplitude 0.5 \
  --duration 2.0 \
  --reset
```

**Note:** The `--reset` flag moves all joints to zero position before testing. This ensures consistent, repeatable tests from a known starting position. **Highly recommended for PID tuning!**

**Tip:** Use shorter durations (3-5 seconds) for faster iteration and easier visualization:
```bash
# Quick 3-second test for faster tuning
ros2 run humanoid_arm_control test_trajectory.py --joint 0 --type sine --amplitude 0.5 --frequency 1.0 --duration 3.0 --reset
```

#### Square Wave
Tests tracking performance with rapid direction changes.
```bash
ros2 run humanoid_arm_control test_trajectory.py \
  --joint 0 \
  --type square \
  --amplitude 0.5 \
  --period 2.0 \
  --cycles 3
```

#### Sine Wave
Tests smooth tracking and reveals oscillation issues.
```bash
ros2 run humanoid_arm_control test_trajectory.py \
  --joint 0 \
  --type sine \
  --amplitude 0.5 \
  --frequency 0.5 \
  --duration 10.0
```

#### Chirp (Frequency Sweep)
Sweeps through frequencies for system identification.
```bash
ros2 run humanoid_arm_control test_trajectory.py \
  --joint 0 \
  --type chirp \
  --amplitude 0.5 \
  --duration 10.0
```

**Joint Index Reference:**
- `0` = base_rotation_joint
- `1` = shoulder_pitch_joint
- `2` = elbow_pitch_joint
- `3` = wrist_pitch_joint
- `4` = wrist_roll_joint

## Manual PID Tuning Procedure

### Ziegler-Nichols Method (Simplified)

This is a systematic approach to finding good PID values from scratch.

**Step 1: Zero out I and D**
```bash
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.i 0.0
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.d 0.0
```

**Step 2: Find Critical P Gain (Ku)**

Start with P=10 and gradually increase until the system oscillates continuously:
```bash
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.p 10.0
ros2 run humanoid_arm_control test_trajectory.py --joint 0 --type step --amplitude 0.5

# If no oscillation, increase P
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.p 50.0
ros2 run humanoid_arm_control test_trajectory.py --joint 0 --type step --amplitude 0.5

# Keep increasing until sustained oscillation occurs
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.p 100.0
# ... continue until oscillation found
```

Record **Ku** (the P value where oscillation starts) and **Tu** (oscillation period in seconds).

**Step 3: Calculate PID Values**

For a **PD controller** (recommended for position control):
- P = 0.8 × Ku
- I = 0 (or very small: 0.1 × Ku / Tu)
- D = 0.1 × Ku × Tu

For a **PID controller** (if steady-state error exists):
- P = 0.6 × Ku
- I = 1.2 × Ku / Tu
- D = 0.075 × Ku × Tu

**Step 4: Apply and Fine-Tune**
```bash
# Example: If Ku=500 and Tu=0.4 seconds
# PD controller: P=400, I=0, D=20

ros2 param set /joint_trajectory_controller gains.base_rotation_joint.p 400.0
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.i 0.0
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.d 20.0
```

Test and adjust:
- If too sluggish: increase P by 20%
- If oscillating: increase D by 50%
- If steady-state error: add small I (2-5)

### Manual Tuning Method (Trial and Error)

**Step 1: Start with baseline values**
```bash
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.p 100.0
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.i 0.0
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.d 0.0
```

**Step 2: Increase P until fast response**
```bash
ros2 run humanoid_arm_control test_trajectory.py --joint 0 --type step --amplitude 0.5

# Increase P in steps
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.p 150.0
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.p 200.0
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.p 300.0
# Stop when response is fast but has overshoot/oscillation
```

**Step 3: Add D to dampen oscillations**
```bash
# Start with D = P / 10
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.d 30.0

# Test and increase if still oscillating
ros2 run humanoid_arm_control test_trajectory.py --joint 0 --type step --amplitude 0.5

# Increase D in small steps
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.d 40.0
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.d 50.0
```

**Step 4: Add I if needed (for steady-state error)**
```bash
# Only if joint doesn't reach target exactly
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.i 2.0

# Test - if overshoots, reduce I
ros2 run humanoid_arm_control test_trajectory.py --joint 0 --type step --amplitude 0.5
```

**Step 5: Set I clamp to prevent windup**
```bash
# Set to 2-4x the I gain
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.i_clamp 8.0
```

**Step 6: Verify with square wave**
```bash
ros2 run humanoid_arm_control test_trajectory.py --joint 0 --type square --amplitude 0.5 --period 2.0 --cycles 3
```

### Using the GUI Method

1. **Launch GUI:**
   ```bash
   ros2 run humanoid_arm_control pid_tuner_gui.py
   ```

2. **Select joint** from dropdown

3. **Set baseline:** P=100, I=0, D=0, click "Apply Changes"

4. **Run test trajectory** in another terminal

5. **Adjust sliders** while watching robot behavior

6. **Click "Apply Changes"** after each adjustment

7. **Iterate** until satisfactory response

## PID Tuning Workflow

### 1. Start with Step Response
```bash
ros2 run humanoid_arm_control test_trajectory.py --joint 0 --type step --amplitude 0.5
```

Monitor the response:
- **Slow rise, no overshoot** → Increase P gain
- **Fast rise, large overshoot** → Decrease P or increase D
- **Oscillation** → Increase D gain
- **Steady-state error** → Increase I gain (use sparingly)

### 2. Test with Square Wave
```bash
ros2 run humanoid_arm_control test_trajectory.py --joint 0 --type square --amplitude 0.5 --period 2.0 --cycles 3
```

Look for:
- Ability to track rapid changes
- Overshoot on transitions
- Settling time between transitions

### 3. Verify with Sine Wave
```bash
ros2 run humanoid_arm_control test_trajectory.py --joint 0 --type sine --amplitude 0.5 --frequency 0.5
```

Check for:
- Phase lag (increase P if too slow)
- Amplitude error (should track closely)
- Smooth motion without jitter

### 4. Use PlotJuggler for Visualization
```bash
ros2 run plotjuggler plotjuggler
```

**Setup PlotJuggler:**

1. **Start streaming:**
   - Click "Streaming" → "Start: ROS2 Topic Subscriber"
   - Click "OK" to load all available topics

2. **Add important topics to plot:**
   - In left panel, find and drag these to the plot area:
     - `/joint_trajectory_controller/controller_state/reference/positions[0]` - Desired position for joint 0
     - `/joint_trajectory_controller/controller_state/feedback/positions[0]` - Actual position for joint 0
     - `/joint_trajectory_controller/controller_state/error/positions[0]` - Tracking error for joint 0
   - Change `[0]` to `[1]`, `[2]`, `[3]`, or `[4]` for other joints

3. **Arrange plots:**
   - Right-click plot → "Split Horizontal" or "Split Vertical"
   - Drag different signals to each subplot
   - Recommended layout:
     - Top: Desired vs Actual position (overlay)
     - Bottom: Position error

4. **Adjust time window:**
   - Look for the time buffer slider at the top (usually shows "20" seconds)
   - For 10 second trajectories: set to 15-20 seconds
   - For longer trajectories: increase to 30-60 seconds
   - Or click "Pause" during playback, then zoom with scroll wheel

5. **Enable time tracking:**
   - Click the "Real-time streaming" button (play icon)
   - The plot will auto-scroll as new data arrives

6. **Analyze response:**
   - **Overshoot** = actual exceeds desired
   - **Rise time** = time to reach 90% of target
   - **Settling time** = time until error stays within tolerance
   - **Steady-state error** = persistent error after settling

**Alternative: rqt_plot (simpler but less features)**
```bash
# Plot tracking error for base joint
rqt_plot /joint_trajectory_controller/controller_state/error/positions[0]

# Plot desired vs actual for base joint
rqt_plot /joint_trajectory_controller/controller_state/reference/positions[0] /joint_trajectory_controller/controller_state/feedback/positions[0]
```

**Monitor raw data in terminal:**
```bash
# Watch controller state continuously
ros2 topic echo /joint_trajectory_controller/controller_state

# Watch just the error values
ros2 topic echo /joint_trajectory_controller/controller_state --field error.positions

# Monitor joint states
ros2 topic echo /joint_states
```

## Current Tuned Values

### Base Rotation Joint
```yaml
p: 600.0
d: 20.0
i: 2.0
i_clamp: 20.0
ff_velocity_scale: 0.1
```

**Constraints:**
- `trajectory`: 0.5 rad (path following tolerance)
- `goal`: 0.1 rad (final position tolerance)
- `goal_time`: 3.0 seconds

### Other Joints
See [humanoid_arm_control/config/controllers.yaml](humanoid_arm_control/config/controllers.yaml) for default values.

## Common Issues

### PATH_TOLERANCE_VIOLATED
**Problem:** Joint can't keep up with trajectory during motion.

**Solutions:**
1. Increase P and D gains (make controller more aggressive)
2. Increase `trajectory` tolerance in constraints
3. Slow down trajectory planning in MoveIt

### Goal Not Reached
**Problem:** Joint stops short of target position.

**Solutions:**
1. Increase P gain for faster response
2. Add small I gain for steady-state correction
3. Increase `goal` tolerance if acceptable
4. Increase `goal_time` for longer movements

### Oscillation
**Problem:** Joint oscillates around target.

**Solutions:**
1. Increase D gain for more damping
2. Decrease P gain if very aggressive
3. Reduce I gain to prevent windup

### Slow Response
**Problem:** Joint moves too slowly.

**Solutions:**
1. Increase P gain significantly
2. Check joint velocity limits in URDF
3. Verify no friction/damping issues in simulation

## Parameter Effects

| Parameter | Effect | When to Increase | When to Decrease |
|-----------|--------|------------------|------------------|
| **P (Proportional)** | Response speed | Slow tracking | Overshoot/oscillation |
| **I (Integral)** | Steady-state error | Doesn't reach target | Windup/overshoot |
| **D (Derivative)** | Damping | Oscillation | Sluggish response |
| **I Clamp** | Limits integral term | I windup issues | Need more I effect |
| **FF Velocity** | Feedforward | Large tracking errors | Instability |

## Tips

1. **Always start with I=0** and tune P and D first
2. **Use small I gains** (1-5) to avoid windup
3. **Test with realistic trajectories** from your application
4. **Monitor tracking errors** using PlotJuggler during tuning
5. **Save good values** to controllers.yaml after tuning
6. **Tune each joint separately** with all others at zero

## Monitoring Commands

```bash
# Check current PID gains
ros2 param get /joint_trajectory_controller gains.base_rotation_joint.p

# Set PID gains manually
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.p 600.0

# Monitor joint states
ros2 topic echo /joint_states

# Monitor controller state
ros2 topic echo /joint_trajectory_controller/controller_state

# List all controller parameters
ros2 param list /joint_trajectory_controller | grep gains
```