# Clock Synchronization Fix for full_system.launch.py

## Problem

When launching `full_system.launch.py` on a fresh clone, you may encounter controller spawning errors related to clock synchronization:

```
[spawner_joint_state_broadcaster]: Failed to activate controller : joint_state_broadcaster
```

This happens because:
1. Gazebo Harmonic takes time to start and publish on `/clock`
2. The `ros_gz_bridge` parameter bridge needs time to relay the clock
3. Controller spawners with `use_sim_time=true` timeout waiting for clock messages

## Solution Applied

### Changes Made:

1. **[controllers.py:36](humanoid_arm_bringup/launch/modules/providers/controllers.py#L36)** - Fixed spawner arguments
   - Changed from `parameters=[{'use_sim_time': use_sim_time}]` to `arguments=['--ros-args', '-p', 'use_sim_time:=true']`
   - This ensures proper ROS parameter passing to the spawner

2. **[full_system.launch.py:108](humanoid_arm_bringup/launch/full_system.launch.py#L108)** - Increased controller spawn delay
   - Changed from 5.0 to 8.0 seconds
   - Allows Gazebo clock to stabilize before spawning controllers

3. **[full_system.launch.py:125](humanoid_arm_bringup/launch/full_system.launch.py#L125)** - Adjusted MoveIt delay
   - Changed from 7.0 to 12.0 seconds
   - Waits for controllers to be fully active

4. **[full_system.launch.py:138](humanoid_arm_bringup/launch/full_system.launch.py#L138)** - Adjusted RViz delay
   - Changed from 9.0 to 14.0 seconds
   - Waits for MoveIt to be ready

## Testing

After applying the fix:

```bash
# Rebuild the workspace
colcon build --packages-select humanoid_arm_bringup --symlink-install
source install/setup.bash

# Test the launch
ros2 launch humanoid_arm_bringup full_system.launch.py
```

You should see:
- Gazebo starts and publishes `/clock` (around 3-5 seconds)
- Robot spawns in Gazebo (around 3 seconds)
- Controllers load successfully (around 8 seconds)
- MoveIt starts (around 12 seconds)
- RViz opens with MoveIt plugin (around 14 seconds)

## Verification

To verify the fix is working:

```bash
# In terminal 1: Launch system
ros2 launch humanoid_arm_bringup full_system.launch.py

# In terminal 2 (after ~10 seconds): Check controllers
ros2 control list_controllers

# Expected output:
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active
```

## Alternative: Manual Sequential Launch

If you still encounter issues, you can launch components manually in sequence:

```bash
# Terminal 1: Start Gazebo
ros2 launch humanoid_arm_bringup gazebo.launch.py

# Wait for "Loaded level [default]" message, then Terminal 2:
ros2 control load_controller joint_state_broadcaster
ros2 control load_controller joint_trajectory_controller
ros2 control set_controller_state joint_state_broadcaster active
ros2 control set_controller_state joint_trajectory_controller active

# Terminal 3: Start MoveIt + RViz
ros2 launch humanoid_arm_moveit_config demo.launch.py
```

## Root Cause Details

The issue stems from ROS 2's strict handling of `use_sim_time`:

1. When a node has `use_sim_time:=true`, it **blocks** waiting for `/clock` messages
2. If `/clock` isn't publishing when the node starts, it times out
3. Gazebo's `/clock` publication has a startup delay:
   - Gazebo server loads (~2-3 seconds)
   - Physics engine initializes (~1-2 seconds)
   - Clock topic becomes active (~3-5 seconds total)

The spawner needs clock synchronization to:
- Timestamp controller activation events
- Synchronize with hardware interface (gz_ros2_control)
- Coordinate with trajectory execution

## System-Specific Factors

Clock sync timing can vary based on:
- **CPU speed**: Slower systems take longer for Gazebo startup
- **Disk I/O**: SSD vs HDD affects world loading time
- **GPU**: GUI rendering can delay clock publication
- **System load**: Background processes slow initialization

If 8 seconds isn't enough for your system, increase the delay in [full_system.launch.py:108](humanoid_arm_bringup/launch/full_system.launch.py#L108):

```python
controllers_delayed = TimerAction(
    period=10.0,  # Increase to 10 or 12 for slower systems
    actions=controller_nodes
)
```

## Future Improvements

A more robust solution would use event-based triggering:

1. Launch Gazebo and clock bridge
2. Use `WaitForTopics` action to detect `/clock` publication
3. Trigger controller spawning only after `/clock` is active
4. Chain remaining components (MoveIt, RViz) similarly

This would eliminate hard-coded delays and adapt to system performance automatically.

## Reference

- ROS 2 Clock Documentation: https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Clock-and-Time.html
- Gazebo-ROS Integration: https://gazebosim.org/docs/harmonic/ros2_integration
