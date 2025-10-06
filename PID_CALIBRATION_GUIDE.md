# PID Calibration Guide

Simple guide for tuning PID gains for each joint using Gazebo simulation and PlotJuggler visualization.

---

## Quick Start

```bash
# 1. Launch PID calibration (choose joint to tune)
ros2 launch humanoid_arm_bringup pid_calibration.launch.py joint:=base_rotation_joint

# 2. Wait ~10 seconds for everything to load
#    - Gazebo opens (physics simulation)
#    - rqt_joint_trajectory_controller GUI opens (send commands)
#    - PlotJuggler opens (visualize oscillations)

# 3. Send commands and observe oscillations
#    In rqt_joint_trajectory_controller:
#    - Select your controller from dropdown (e.g., base_rotation_joint_position_controller)
#    - Move slider to send position commands
#    - Watch joint move in Gazebo

# 4. Plot tracking performance in PlotJuggler
#    Click "Streaming" → "Start" → "ROS2 Topics"
#    Add these topics:
#    - /joint_states/position[0]  (actual position)
#    - /base_rotation_joint_position_controller/controller_state/reference/positions[0]  (desired position)
#    - /base_rotation_joint_position_controller/controller_state/error/positions[0]  (tracking error)

# 5. Tune PID gains in another terminal
ros2 run humanoid_arm_control pid_tuner_gui.py
```

---

## Available Joints

Tune one joint at a time:

```bash
# Base rotation (1st joint)
ros2 launch humanoid_arm_bringup pid_calibration.launch.py joint:=base_rotation_joint

# Shoulder pitch (2nd joint)
ros2 launch humanoid_arm_bringup pid_calibration.launch.py joint:=shoulder_pitch_joint

# Elbow pitch (3rd joint)
ros2 launch humanoid_arm_bringup pid_calibration.launch.py joint:=elbow_pitch_joint

# Wrist pitch (4th joint)
ros2 launch humanoid_arm_bringup pid_calibration.launch.py joint:=wrist_pitch_joint

# Wrist roll (5th joint)
ros2 launch humanoid_arm_bringup pid_calibration.launch.py joint:=wrist_roll_joint
```

---

## What Gets Launched

1. **Gazebo Harmonic** - Physics simulation with realistic dynamics (gravity, inertia, friction)
2. **Single Joint Controller** - Only ONE joint has active control (others are passive)
3. **rqt_joint_trajectory_controller** - GUI to send position commands
4. **PlotJuggler** - Real-time plotting of joint behavior

**Important:** Only the selected joint moves! Other joints stay passive (no oscillations, no interference).

---

## PID Tuning Workflow

### Step 1: Observe Current Behavior

1. Launch calibration for the joint you want to tune
2. In `rqt_joint_trajectory_controller`, select the controller from dropdown
3. Move the slider to a new position (e.g., 0.5 radians)
4. Watch the joint move in Gazebo
5. In PlotJuggler, observe:
   - **Oscillations** - Does it wiggle before settling?
   - **Overshoot** - Does it go past the target?
   - **Settling time** - How long to reach steady state?
   - **Steady-state error** - Does it reach exactly the target?

### Step 2: Adjust PID Gains

Open PID tuner in another terminal:
```bash
ros2 run humanoid_arm_control pid_tuner_gui.py
```

The tuner auto-detects which controller is running.

**Tuning process:**

1. **Start with P only** (set I=0, D=0):
   - Increase P until you see small oscillations
   - Too low P → slow, sluggish response
   - Too high P → large oscillations

2. **Add D (damping)**:
   - Increase D to reduce oscillations
   - D = P/10 to P/20 is a good starting point
   - Too much D → very slow response

3. **Add I (if needed)**:
   - Only if there's steady-state error (doesn't reach target exactly)
   - Start small: I = P/50 or less
   - Too much I → oscillations and instability

4. **Adjust feedforward (optional)**:
   - `ff_velocity_scale` helps with tracking moving targets
   - Start with 0.1, adjust if needed

### Step 3: Test with Different Commands

Send various commands to verify tuning:

- **Small step** (0.2 rad) - Should track smoothly
- **Large step** (1.0 rad) - Check for overshoot
- **Sine wave** - Use test trajectory script (see below)

### Step 4: Save Good Values

When satisfied with behavior, note the PID gains and update `controllers.yaml`:

```yaml
# humanoid_arm_control/config/controllers.yaml

base_rotation_joint_position_controller:
  ros__parameters:
    gains:
      base_rotation_joint:
        p: 150.0      # Your tuned value
        d: 8.0        # Your tuned value
        i: 5.0        # Your tuned value
        i_clamp: 20.0
        ff_velocity_scale: 0.1
```

Then rebuild:
```bash
colcon build --packages-select humanoid_arm_control
```

---

## PlotJuggler Setup

### Topics to Plot

**Essential:**
- `/joint_states/position[X]` - Actual joint position (X = joint index 0-4)
- `/<joint>_position_controller/controller_state/reference/positions[0]` - Desired position
- `/<joint>_position_controller/controller_state/error/positions[0]` - Tracking error

**Optional:**
- `/joint_states/velocity[X]` - Joint velocity
- `/<joint>_position_controller/controller_state/error/velocities[0]` - Velocity error

### Joint Index Mapping

- `position[0]` = base_rotation_joint
- `position[1]` = shoulder_pitch_joint
- `position[2]` = elbow_pitch_joint
- `position[3]` = wrist_pitch_joint
- `position[4]` = wrist_roll_joint

### Useful PlotJuggler Features

- **Streaming** → Start ROS2 Topics
- **Drag topics** to plot area
- **Right-click plot** → Split horizontal/vertical for multiple plots
- **Zoom** with mouse wheel
- **Pan** by dragging
- **Buffer size** - Increase to see more history (default 60s)

---

## Test Trajectory Scripts

Generate test trajectories for systematic tuning:

```bash
# Step response (good for overshoot/oscillation testing)
python3 humanoid_arm_control/scripts/test_trajectory.py \
  --joint 0 \
  --type step \
  --amplitude 0.5 \
  --reset

# Sine wave (good for tracking performance)
python3 humanoid_arm_control/scripts/test_trajectory.py \
  --joint 0 \
  --type sine \
  --amplitude 0.3 \
  --frequency 0.5 \
  --reset

# Square wave (tests settling time)
python3 humanoid_arm_control/scripts/test_trajectory.py \
  --joint 0 \
  --type square \
  --amplitude 0.4 \
  --frequency 0.2 \
  --reset
```

**Note:** Joint index (--joint) maps to:
- 0 = base_rotation_joint
- 1 = shoulder_pitch_joint
- 2 = elbow_pitch_joint
- 3 = wrist_pitch_joint
- 4 = wrist_roll_joint

---

## Common Issues

### "Controller failed to load"
- Check that you spelled the joint name correctly
- Verify controllers.yaml has the controller definition
- Rebuild: `colcon build --packages-select humanoid_arm_control`

### "No oscillations visible"
- You might be tuning with mock hardware (not Gazebo)
- Verify Gazebo is running (you should see the robot)
- Check that effort interface is claimed: `ros2 control list_hardware_interfaces`

### "Joint doesn't move"
- Verify controller is active: `ros2 control list_controllers`
- Check you selected the right controller in rqt_joint_trajectory_controller
- Make sure you're sending commands (move the slider)

### "PlotJuggler shows nothing"
- Click "Streaming" → "Start"
- Select "ROS2 Topics"
- Drag topics to plot area
- Check topics exist: `ros2 topic list`

### "All joints oscillate"
- You probably launched `full_system.launch.py` instead
- Use `pid_calibration.launch.py` for single-joint tuning
- Only ONE joint should have an active controller

---

## Current PID Gains (Default)

Starting values (not tuned yet):

| Joint | P | D | I | I_clamp | FF_vel |
|-------|-----|-----|-----|---------|--------|
| base_rotation | 150 | 8 | 5 | 20 | 0.1 |
| shoulder_pitch | 180 | 10 | 8 | 25 | 0.12 |
| elbow_pitch | 120 | 7 | 6 | 18 | 0.1 |
| wrist_pitch | 100 | 6 | 5 | 15 | 0.08 |
| wrist_roll | 80 | 5 | 4 | 12 | 0.06 |

**These are starting guesses - tune them for your system!**

---

## Tips

✅ **Do:**
- Tune one joint at a time
- Start with small P, gradually increase
- Use PlotJuggler to see what's happening
- Test with different command amplitudes
- Save good values immediately

❌ **Don't:**
- Tune with mock hardware (no physics = no real PID effects)
- Set gains too high (can cause violent oscillations)
- Forget to source workspace after rebuilding
- Try to tune all joints at once

---

## Next Steps

After tuning all 5 joints:

1. Update all PID values in `controllers.yaml`
2. Test with `full_system.launch.py` (all joints together)
3. Verify no unwanted coupling between joints
4. Test with MoveIt motion planning
5. Ready for real hardware when motors arrive!

---

## See Also

- [PID_README.md](PID_README.md) - Detailed PID theory and methods
- [SOFTWARE_GUIDE.md](SOFTWARE_GUIDE.md) - All ROS2 tools reference
- [CLAUDE.md](CLAUDE.md) - Complete project documentation
