# Trajectory Timing and Motion Planning

## Overview

This document explains how `time_from_start` works in ROS2 trajectory controllers and how to calculate optimal timing based on motor specifications and load dynamics.

## Understanding `time_from_start`

### What It Is

`time_from_start` is **NOT a speed parameter directly**, but it **controls speed indirectly** by setting a deadline for reaching the target position.

### The Concept

Think of it like telling someone: *"Go from here to that point over there, and you have X seconds to do it."*

- You **didn't specify the speed**
- They **calculate the required speed** based on distance and time

### The Formula

```
Required Velocity = Distance / Time
```

### Code Example

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

point = JointTrajectoryPoint()
point.positions = [1.5]  # Target: 1.5 radians
point.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1 seconds
```

**What happens:**
- Current position: 0.0 rad
- Target position: 1.5 rad
- Time allowed: 0.1 seconds
- **Required velocity**: (1.5 - 0.0) / 0.1 = **15 rad/s**

## The Speed-Time Tradeoff

### Short Time (Fast Motion)

```python
point.time_from_start = Duration(sec=0, nanosec=50000000)  # 0.05s
```

**Characteristics:**
- ✅ Fast response
- ✅ Allows motor to reach high speeds
- ⚠️ Requires high acceleration
- ⚠️ May cause large tracking errors
- ⚠️ May cause overshoot and oscillations
- ⚠️ Higher motor current (more heat)

**Use cases:**
- Emergency stops
- Quick reflexes
- Light loads
- Short distances

### Long Time (Slow, Precise Motion)

```python
point.time_from_start = Duration(sec=2, nanosec=0)  # 2.0s
```

**Characteristics:**
- ✅ Smooth motion
- ✅ Minimal tracking error
- ✅ Lower accelerations
- ✅ Better precision
- ✅ Lower motor current
- ❌ Slower response

**Use cases:**
- Precision tasks
- Heavy loads
- Contact manipulation
- When accuracy > speed

## Why Not Just Set Speed Directly?

Trajectory controllers use **time-based trajectories** instead of pure velocity commands for several reasons:

1. **Synchronization** - Multiple joints can reach targets at the same time
2. **Smooth motion** - Controller can plan acceleration/deceleration curves
3. **Predictability** - You know exactly when motion will complete
4. **Coordination** - Enables complex multi-joint trajectories with MoveIt

## Example Comparison

Moving 1 radian (≈57°):

| `time_from_start` | Required Velocity | Acceleration | Precision | Motor Load |
|-------------------|-------------------|--------------|-----------|------------|
| 0.05s | 20 rad/s | Very High | ⚠️ May overshoot | Very High |
| 0.1s | 10 rad/s | High | ⚠️ Moderate error | High |
| 0.5s | 2 rad/s | Medium | ✅ Good | Medium |
| 1.0s | 1 rad/s | Low | ✅ Excellent | Low |
| 2.0s | 0.5 rad/s | Very Low | ✅✅ Surgical | Very Low |

## Calculating Optimal Time

### Basic Calculation (Velocity Limited)

The simplest approach considers only the motor's maximum velocity:

```python
import yaml

# Load motor specs
with open('actuator_specs.yaml', 'r') as f:
    specs = yaml.safe_load(f)

def calculate_min_time_simple(joint_name, current_pos, target_pos):
    """Calculate minimum time based on velocity limit only"""
    distance = abs(target_pos - current_pos)
    max_speed = specs['actuators'][joint_name]['motor']['max_speed']

    # Add 20% safety margin
    min_time = (distance / max_speed) * 1.2

    # Enforce minimum (for very small movements)
    return max(0.1, min_time)

# Example
joint = 'base_rotation_joint'
current = 0.0
target = 3.14  # 180 degrees
min_time = calculate_min_time_simple(joint, current, target)
print(f"Minimum safe time: {min_time:.2f} seconds")
```

### Advanced Calculation (Acceleration Limited)

Real motors have limited acceleration due to torque and inertia. This creates a **trapezoidal velocity profile**:

1. **Acceleration phase** - Speed up to max velocity
2. **Cruise phase** - Maintain max velocity
3. **Deceleration phase** - Slow down to target

```python
import math

def calculate_min_time_accurate(joint_name, current_pos, target_pos, load_inertia=None):
    """
    Calculate minimum time accounting for acceleration limits.

    Args:
        joint_name: Name of joint (e.g., 'base_rotation_joint')
        current_pos: Current position (rad)
        target_pos: Target position (rad)
        load_inertia: Total reflected inertia at motor (kg·m²)
                      If None, uses conservative estimate

    Returns:
        min_time: Minimum time (seconds)
    """
    distance = abs(target_pos - current_pos)

    # Get motor specs
    max_torque = specs['actuators'][joint_name]['motor']['max_torque']
    max_speed = specs['actuators'][joint_name]['motor']['max_speed']

    # Estimate load inertia if not provided
    if load_inertia is None:
        # Conservative estimate: use typical values
        load_inertia_estimates = {
            'base_rotation_joint': 0.15,     # Entire arm
            'shoulder_pitch_joint': 0.10,    # Forearm + wrist + hand
            'elbow_pitch_joint': 0.05,       # Wrist + hand
            'wrist_pitch_joint': 0.02,       # Hand only
            'wrist_roll_joint': 0.005,       # End effector
        }
        load_inertia = load_inertia_estimates.get(joint_name, 0.05)

    # Calculate maximum acceleration
    # Torque = Inertia × Acceleration → Acceleration = Torque / Inertia
    max_acceleration = max_torque / load_inertia  # rad/s²

    # Time to reach max speed from standstill
    t_accel = max_speed / max_acceleration

    # Distance covered during acceleration (v = a*t, d = 0.5*a*t²)
    d_accel = 0.5 * max_acceleration * t_accel**2

    # Check if we reach max speed (trapezoidal) or not (triangular)
    if distance < 2 * d_accel:
        # SHORT MOVE - Triangular profile (never reaches max speed)
        # Accelerate to peak, then immediately decelerate
        # Total distance = 2 × (0.5 × a × t²) = a × t²
        # So: t = sqrt(distance / acceleration)
        min_time = 2 * math.sqrt(distance / max_acceleration)
    else:
        # LONG MOVE - Trapezoidal profile
        # Accel phase + cruise phase + decel phase
        d_cruise = distance - 2 * d_accel
        t_cruise = d_cruise / max_speed
        min_time = 2 * t_accel + t_cruise

    # Add 20% safety margin for PID tracking error
    min_time *= 1.2

    # Enforce absolute minimum (for stability)
    return max(0.05, min_time)


# Example usage
joint = 'base_rotation_joint'
current = 0.0
target = 3.14  # 180 degrees

# Simple calculation (velocity only)
simple_time = calculate_min_time_simple(joint, current, target)
print(f"Simple calculation: {simple_time:.3f} seconds")

# Accurate calculation (with acceleration)
accurate_time = calculate_min_time_accurate(joint, current, target)
print(f"Accurate calculation: {accurate_time:.3f} seconds")

# With known inertia
accurate_time_known = calculate_min_time_accurate(joint, current, target, load_inertia=0.15)
print(f"With known inertia: {accurate_time_known:.3f} seconds")
```

### Motion Profile Visualization

```
Trapezoidal Profile (Long Move):
Velocity
   ↑
   │       ┌─────────┐  ← Max speed (cruise phase)
   │      ╱           ╲
   │     ╱             ╲
   │    ╱               ╲
   │   ╱                 ╲
   └──┴─────────────────┴──→ Time
      ↑                  ↑
   Accel              Decel

Triangular Profile (Short Move):
Velocity
   ↑
   │       ┌  ← Peak speed (< max speed)
   │      ╱ ╲
   │     ╱   ╲
   │    ╱     ╲
   │   ╱       ╲
   └──┴─────────┴──→ Time
```

## Load-Dependent Timing

Different joints carry different loads, affecting their dynamic performance:

### Load Characteristics

| Joint | Moves | Estimated Inertia | Max Accel (120 Nm) | Max Accel (60 Nm) |
|-------|-------|-------------------|-------------------|-------------------|
| Base Rotation | Entire arm (~6 kg) | 0.15 kg·m² | 800 rad/s² | - |
| Shoulder Pitch | Forearm + wrist (~4 kg) | 0.10 kg·m² | 1200 rad/s² | - |
| Elbow Pitch | Wrist + hand (~2 kg) | 0.05 kg·m² | - | 1200 rad/s² |
| Wrist Pitch | Hand only (~1 kg) | 0.02 kg·m² | - | 3000 rad/s² |
| Wrist Roll | End effector (~0.5 kg) | 0.005 kg·m² | - | 3400 rad/s² (17 Nm) |

### Calculating Reflected Inertia

The total inertia "seen" by the motor includes:

1. **Motor rotor inertia** (from actuator specs)
2. **Gearbox inertia** (usually negligible)
3. **Reflected load inertia** = Load inertia × gear_ratio²

```python
def calculate_reflected_inertia(joint_name):
    """Calculate total inertia reflected to motor shaft"""

    # Motor rotor inertia (from specs)
    rotor_inertia = specs['actuators'][joint_name]['motor']['rotor_inertia']

    # Gear ratio
    gear_ratio = specs['actuators'][joint_name]['motor']['gear_ratio']

    # Load inertia at output (measured or from URDF)
    load_inertia_output = get_link_inertia(joint_name)  # kg·m² at joint

    # Reflect to motor side (through gearbox)
    reflected_load = load_inertia_output / (gear_ratio ** 2)

    # Total inertia at motor
    total_inertia = rotor_inertia + reflected_load

    return total_inertia
```

### Enhanced Specs File

You could add this to `actuator_specs.yaml`:

```yaml
actuators:
  base_rotation_joint:
    motor:
      # ... existing motor specs ...

    load:
      # Measured or estimated load characteristics
      reflected_inertia: 0.15      # kg·m² (total at motor)
      static_friction: 2.0          # Nm (coulomb friction)
      viscous_friction: 0.5         # Nm·s/rad (damping)

      # For advanced motion planning
      max_safe_acceleration: 600    # rad/s² (conservative)
      max_safe_jerk: 5000           # rad/s³ (rate of acceleration change)
```

## Practical Implementation for GUI

### Simple Version (Current)

```python
# In joint_control.py
def send_joint_command(self, controller_name, joint_name, position):
    # Fixed time for all movements
    point.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1s
```

### Smart Version (Distance-Based)

```python
class JointControlNode(Node):
    def __init__(self):
        super().__init__('joint_control_gui')

        # Load actuator specs once at startup
        self.load_actuator_specs()

    def load_actuator_specs(self):
        """Load motor specifications from YAML"""
        import yaml
        from ament_index_python.packages import get_package_share_directory

        specs_path = get_package_share_directory('humanoid_arm_control')
        specs_path += '/config/actuator_specs.yaml'

        with open(specs_path, 'r') as f:
            self.actuator_specs = yaml.safe_load(f)

    def calculate_movement_time(self, joint_name, current_pos, target_pos):
        """Calculate optimal time based on distance and motor limits"""
        distance = abs(target_pos - current_pos)

        # Get motor max speed
        max_speed = self.actuator_specs['actuators'][joint_name]['motor']['max_speed']

        # Calculate time with safety margin
        min_time = (distance / max_speed) * 1.2

        # Clamp to reasonable bounds
        return max(0.05, min(2.0, min_time))

    def send_joint_command(self, controller_name, joint_name, position):
        """Send trajectory with calculated timing"""
        # Get current position
        current_pos = self.joint_states.get(joint_name, {}).get('position', 0.0)

        # Calculate optimal time
        optimal_time = self.calculate_movement_time(joint_name, current_pos, position)

        # Create trajectory
        traj = JointTrajectory()
        traj.joint_names = [joint_name]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(
            sec=int(optimal_time),
            nanosec=int((optimal_time % 1) * 1e9)
        )

        traj.points = [point]
        self.joint_pubs[controller_name].publish(traj)
```

### Advanced Version (With Speed Control)

Add a speed adjustment slider to the GUI:

```python
class JointControlGUI(QtWidgets.QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node

        # Speed scale: 0.1 (very slow) to 2.0 (very fast)
        self.speed_scale = 1.0  # Default: normal speed

        # Add speed slider to UI
        self.speedSlider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speedSlider.setMinimum(10)   # 0.1x speed
        self.speedSlider.setMaximum(200)  # 2.0x speed
        self.speedSlider.setValue(100)    # 1.0x speed
        self.speedSlider.valueChanged.connect(self.on_speed_changed)

    def on_speed_changed(self, value):
        """Update speed scale when slider moves"""
        self.speed_scale = value / 100.0
        self.speedLabel.setText(f"Speed: {self.speed_scale:.1f}x")

    def send_joint_command_with_speed(self, controller_name, joint_name, position):
        """Send trajectory with user-adjustable speed"""
        current_pos = self.node.joint_states.get(joint_name, {}).get('position', 0.0)

        # Calculate base time
        base_time = self.node.calculate_movement_time(joint_name, current_pos, position)

        # Apply speed scale (higher scale = shorter time = faster)
        actual_time = base_time / self.speed_scale

        # Send trajectory
        # ... (same as before, using actual_time)
```

## Best Practices

### General Guidelines

1. **For GUI sliders** (continuous adjustments):
   - Use fixed short time (0.1-0.2s) for responsiveness
   - OR calculate based on distance

2. **For programmed trajectories**:
   - Always calculate based on motor limits
   - Account for load inertia if known
   - Add 20% safety margin

3. **For precision tasks**:
   - Use longer times (1-5s)
   - Consider load and friction
   - Test and tune on real hardware

4. **For emergency stops**:
   - Use shortest safe time
   - May exceed normal limits briefly
   - Monitor motor temperature

### Safety Considerations

⚠️ **Too short:**
- Exceeds velocity limits → Controller clamps velocity
- High acceleration → May cause oscillations
- Tracking errors → PATH_TOLERANCE_VIOLATED
- Motor overheating

⚠️ **Too long:**
- Unnecessarily slow
- Lower system throughput
- May feel unresponsive

✅ **Just right:**
- Reaches ~80-90% of max velocity on large moves
- Smooth acceleration curve
- Minimal tracking error
- Sustainable motor operation

## Testing and Tuning

### Step 1: Measure Actual Performance

```bash
# Monitor joint velocities
ros2 topic echo /joint_states --field velocity

# Monitor tracking errors
ros2 topic echo /base_rotation_joint_position_controller/controller_state --field error
```

### Step 2: Plot Motion Profile

```python
import matplotlib.pyplot as plt

# Record position over time
positions = []
times = []

# ... collect data from /joint_states ...

# Plot
plt.plot(times, positions)
plt.xlabel('Time (s)')
plt.ylabel('Position (rad)')
plt.title('Joint Motion Profile')
plt.show()

# Calculate velocity (derivative)
velocities = np.diff(positions) / np.diff(times)
plt.plot(times[1:], velocities)
plt.axhline(y=max_speed, color='r', linestyle='--', label='Max Speed')
plt.legend()
plt.show()
```

### Step 3: Adjust Based on Results

- **If velocity saturates** → Time is too short, increase it
- **If motion is jerky** → Time is too short, increase it
- **If motion is too slow** → Time is too long, decrease it
- **If tracking error is large** → Time is too short OR PID gains need tuning

## Related Files

- `humanoid_arm_control/config/actuator_specs.yaml` - Motor specifications (max torque, max speed)
- `humanoid_arm_control/scripts/joint_control.py` - GUI that sends trajectory commands
- `humanoid_arm_control/config/controllers.yaml` - PID gains and trajectory constraints
- `MOTOR_SPECIFICATIONS.md` - Detailed motor specs and advanced control features
- `PID_README.md` - PID tuning guide

## Future Enhancements

When real motors arrive and you need production-level control:

1. ✅ **Measure actual load inertias** - Use motor current feedback during test moves
2. ✅ **Implement adaptive timing** - Calculate time based on distance and load
3. ✅ **Add speed control slider** - Let users adjust motion speed preference
4. ✅ **Profile different move types** - Fast for free space, slow for contact tasks
5. ✅ **Integrate with MoveIt** - Use proper time parameterization for complex trajectories
6. ✅ **Add jerk limiting** - Limit rate of acceleration change for smoother motion
7. ✅ **Thermal awareness** - Reduce limits when motor is hot

## Summary

**Key Takeaway:** `time_from_start` is a **deadline**, not a speed setting. The controller calculates required velocity to meet the deadline based on distance.

**Quick Reference:**
- **Fast (0.05-0.1s)**: Responsive, but may exceed limits on large moves
- **Normal (0.2-0.5s)**: Good balance for most tasks
- **Slow (1-5s)**: Precise, smooth, safe

**Best Practice:** Calculate time based on distance and motor `max_speed` from `actuator_specs.yaml` with 20% safety margin.
