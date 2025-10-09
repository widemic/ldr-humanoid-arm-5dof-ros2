# Motor Specifications and Advanced Control Features

## Overview

This document describes the motor specifications system and advanced control features that can be implemented for realistic motor behavior.

## Single Source of Truth

**File:** `humanoid_arm_control/config/actuator_specs.yaml`

This YAML file is the single source of truth for all motor specifications. It is automatically loaded by xacro in URDF files.

### Currently Used Parameters

The following parameters are automatically applied to the robot:

- **`motor.max_torque`** → URDF `<limit effort="..."/>` and ros2_control effort command interface limits
- **`motor.max_speed`** → URDF `<limit velocity="..."/>` and ros2_control velocity command interface limits

### How to Update Motor Specs

1. Edit `humanoid_arm_control/config/actuator_specs.yaml`
2. Rebuild: `colcon build --packages-select humanoid_arm_description --symlink-install`
3. Restart Gazebo

All URDF joint limits and ros2_control command interfaces will automatically update.

## Motor Lineup

Current motors (Robstride series):

| Joint | Motor | Max Torque | Max Speed | Rated Torque | Rated Speed |
|-------|-------|------------|-----------|--------------|-------------|
| Base Rotation | Robstride 04 | 120 Nm | 20.94 rad/s (200 rpm) | 40 Nm | 17.49 rad/s (167 rpm) |
| Shoulder Pitch | Robstride 04 | 120 Nm | 20.94 rad/s (200 rpm) | 40 Nm | 17.49 rad/s (167 rpm) |
| Elbow Pitch | Robstride 03 | 60 Nm | 20.42 rad/s (195 rpm) | 20 Nm | 18.85 rad/s (180 rpm) |
| Wrist Pitch | Robstride 03 | 60 Nm | 20.42 rad/s (195 rpm) | 20 Nm | 18.85 rad/s (180 rpm) |
| Wrist Roll | Robstride 02 | 17 Nm | 42.94 rad/s (410 rpm) | 6 Nm | 37.70 rad/s (360 rpm) |

## Torque-Speed Tradeoff

### The Problem

Real brushless motors cannot deliver maximum torque at maximum speed simultaneously due to power constraints.

**Example: Robstride 04**
- At **low speed (0-107 rpm)**: Can deliver full 120 Nm
- At **high speed (200 rpm)**: Can only deliver ~40 Nm (rated continuous)
- Power limit: ~1200W rated

**Current Implementation:**
- Simulation uses `max_torque=120 Nm` and `max_speed=20.94 rad/s`
- Gazebo will allow 120 Nm at 20.94 rad/s (physically impossible, would require ~2500W)
- This is conservative for PID tuning and motion planning

### Why This Matters

1. **Unrealistic Performance**: Simulation may show trajectories that work but fail on real hardware
2. **PID Tuning**: Gains tuned in simulation may be too aggressive for real motor limits
3. **Motion Planning**: MoveIt may plan trajectories that exceed combined torque-speed limits

## Advanced Implementation Options

These features are **documented but not implemented**. They can be added when needed for realistic behavior or hardware integration.

### 1. Power Limiting (Recommended)

Enforce the physical power constraint: `|torque × velocity| ≤ max_power`

**Implementation:**
1. Add to `actuator_specs.yaml`:
   ```yaml
   motor:
     max_power: 1200.0  # Watts (for Robstride 04)
   ```

2. In controller (e.g., `joint_trajectory_controller` or custom controller):
   ```cpp
   double get_limited_effort(double desired_effort, double current_velocity) {
     if (abs(current_velocity) < 0.01) {
       return std::clamp(desired_effort, -max_torque, max_torque);
     }
     double power_limited_torque = max_power / abs(current_velocity);
     double max_allowed = std::min(max_torque, power_limited_torque);
     return std::clamp(desired_effort, -max_allowed, max_allowed);
   }
   ```

**Benefits:**
- Simple to implement
- Physically accurate for most scenarios
- Smooth limiting behavior

### 2. Torque-Speed Curve (Most Accurate)

Define piecewise linear curve of maximum torque vs velocity.

**Implementation:**
1. Add to `actuator_specs.yaml`:
   ```yaml
   motor:
     torque_speed_curve:  # [(velocity rad/s, max_torque Nm), ...]
       - [0.0, 120.0]      # Stall torque
       - [11.2, 120.0]     # Peak torque region (0-107 rpm)
       - [20.94, 40.0]     # Max speed at rated torque
   ```

2. In controller:
   ```cpp
   double interpolate_max_torque(double velocity,
                                  const std::vector<std::pair<double, double>>& curve) {
     // Linear interpolation between curve points
     velocity = abs(velocity);
     for (size_t i = 0; i < curve.size() - 1; i++) {
       if (velocity >= curve[i].first && velocity <= curve[i+1].first) {
         double t = (velocity - curve[i].first) /
                    (curve[i+1].first - curve[i].first);
         return curve[i].second + t * (curve[i+1].second - curve[i].second);
       }
     }
     return curve.back().second;  // Beyond max speed
   }
   ```

**Benefits:**
- Most accurate representation
- Captures motor characteristics precisely
- Can model different motor behaviors (e.g., constant power region)

**Drawbacks:**
- Requires accurate motor data
- More complex implementation

### 3. Thermal Model (Advanced)

Track motor temperature and reduce limits when overheating.

**Implementation:**
1. Add to `actuator_specs.yaml`:
   ```yaml
   motor:
     thermal_resistance: 0.8        # °C/W (motor to ambient)
     thermal_capacitance: 150.0     # J/°C (motor thermal mass)
     thermal_time_constant: 900     # seconds (15 min)
     max_temperature: 85.0          # °C
     ambient_temperature: 25.0      # °C
     copper_loss_constant: 0.18     # Ohm (winding resistance)
     iron_loss_constant: 0.01       # Nm·s (hysteresis/eddy current)
   ```

2. Thermal dynamics:
   ```cpp
   // Simplified first-order thermal model
   double power_loss = i_squared_r_loss + mechanical_loss;
   double heat_flow = power_loss - (temperature - ambient) / thermal_resistance;
   temperature += heat_flow * dt / thermal_capacitance;

   // Reduce limits based on temperature
   double thermal_derating = 1.0;
   if (temperature > max_temperature * 0.8) {
     thermal_derating = std::max(0.5,
       1.0 - (temperature - max_temperature * 0.8) / (max_temperature * 0.2));
   }
   max_torque_current = max_torque * thermal_derating;
   ```

**Benefits:**
- Prevents motor damage in long-duration tasks
- Realistic behavior for continuous operation
- Can simulate duty cycle limitations

**Drawbacks:**
- Complex implementation
- Requires accurate thermal constants
- Adds computational overhead

### 4. Conservative Approach (Simplest)

Just use rated (continuous) values instead of peak values.

**Implementation:**
1. In `actuator_specs.yaml`, set limits to rated values:
   ```yaml
   motor:
     max_torque: 40.0      # Use rated_torque instead of max_torque
     max_speed: 17.49      # Use rated_speed instead of max_speed
   ```

**Benefits:**
- Guarantees continuous operation without overheating
- Simple - no code changes needed
- Safe for hardware

**Drawbacks:**
- Underutilizes motor capabilities
- Can't take advantage of peak torque for short bursts

## Parameters Reference

### Currently Used (Automatically Applied)

| Parameter | Description | Used In |
|-----------|-------------|---------|
| `max_torque` | Peak torque (Nm) | URDF `<limit effort>`, ros2_control effort limits |
| `max_speed` | Maximum velocity (rad/s) | URDF `<limit velocity>`, ros2_control velocity limits |

### For Future Hardware Implementation

| Parameter | Description | Use Case |
|-----------|-------------|----------|
| `can_id` | CAN bus address | Hardware communication |
| `gear_ratio` | Gearbox reduction ratio | Torque/velocity conversion |
| `gear_efficiency` | Gearbox efficiency (0-1) | Power loss calculation |
| `motor_constant_kt` | Torque constant (Nm/A) | Current control, FOC |
| `motor_constant_ke` | Back-EMF constant (V·s/rad) | Velocity estimation, FOC |
| `motor_resistance` | Phase resistance (Ohm) | Current control, thermal model |
| `motor_inductance` | Phase inductance (H) | Current control, FOC |
| `rotor_inertia` | Rotor inertia (kg·m²) | Dynamics simulation |
| `encoder_resolution` | Counts per revolution | Position feedback precision |
| `encoder_accuracy` | Position error (rad) | Control accuracy limits |
| `friction_viscous` | Viscous damping (Nm·s/rad) | Dynamics (note: URDF has separate `<dynamics>`) |
| `friction_coulomb` | Static friction (Nm) | Dynamics (note: URDF has separate `<dynamics>`) |
| `backlash` | Gearbox backlash (rad) | Position accuracy |

### Documentation Only (Not Currently Used)

| Parameter | Description |
|-----------|-------------|
| `rated_torque` | Continuous torque rating |
| `rated_speed` | Speed at rated torque |
| `rated_voltage` | Nominal supply voltage |
| `rated_current` | Continuous phase current |
| `peak_current` | Maximum phase current |
| `thermal_time_constant` | Motor cooling rate |
| `max_temperature` | Temperature limit |
| PID gains | Documented separately per joint |
| Feedforward gains | For trajectory following |
| Communication settings | Timeout, retry, update rate |

## Recommendations

### For Simulation and PID Tuning (Current Setup)
✅ **Use max values** - Simple, works well for controller development

### Before Hardware Integration
⚠️ **Implement power limiting** - Prevents unrealistic commands

### For Production Use
🔧 **Implement torque-speed curve** - Most accurate motor behavior

### For Long-Duration Tasks
🔥 **Consider thermal model** - Prevents motor damage

## Related Files

- `humanoid_arm_control/config/actuator_specs.yaml` - Motor specifications (single source of truth)
- `humanoid_arm_description/urdf/humanoid_arm_5dof_macro.urdf.xacro` - Loads specs for joint limits
- `humanoid_arm_description/urdf/humanoid_arm_5dof.ros2_control.xacro` - Loads specs for command interfaces
- `humanoid_arm_control/config/controllers.yaml` - PID gains (separate from motor specs)
- `CLAUDE.md` - General project documentation
- `PID_README.md` - PID tuning guide

## Future Work

When real Robstride motors arrive:

1. **Verify specifications** - Test actual torque-speed curves
2. **Implement CAN communication** - Use `can_id` and communication settings
3. **Add power limiting** - Start with option 1 (simple power limit)
4. **Tune PID gains** - May need adjustment for real hardware
5. **Consider thermal protection** - Especially for continuous operation

## Questions?

See `CLAUDE.md` for general project information and contact details.
