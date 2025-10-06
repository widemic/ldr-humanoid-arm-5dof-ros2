# Launch Modules - Reusable Python Components

Clean, composable Python modules for building ROS2 launch files. Think of these as building blocks that you can mix and match to create different launch configurations.

## Philosophy

**DRY (Don't Repeat Yourself)** - Write once, use everywhere.

Each module handles ONE concern and provides a clean Python API. Launch files become simple composition of these modules.

---

## Modules

### `robot_description.py`
Handles robot URDF loading and robot_state_publisher.

**Functions:**
- `load_urdf(use_fake_hardware, robot_controller)` → Load and process URDF
- `get_robot_state_publisher(robot_description, use_sim_time)` → Robot state publisher node
- `get_control_node(robot_description, use_fake_hardware)` → ros2_control_node (for mock hardware)

**Usage:**
```python
robot_desc = robot_description.load_urdf(use_fake_hardware=True)
rsp = robot_description.get_robot_state_publisher(robot_desc)
```

---

### `controllers.py`
Spawns ros2_control controllers with proper sequencing.

**Functions:**
- `spawn_controller(controller_name, use_sim_time)` → Spawn any controller
- `get_joint_state_broadcaster(use_sim_time)` → Joint state broadcaster (always first!)
- `get_trajectory_controller(use_sim_time)` → Joint trajectory controller
- `get_position_controller(use_sim_time)` → Position controller
- `get_velocity_controller(use_sim_time)` → Velocity controller
- `sequence_after(trigger_node, nodes_to_start)` → Start nodes after trigger exits
- `get_standard_controllers(primary_controller, use_sim_time)` → Standard setup with sequencing
- `get_all_controllers(use_sim_time)` → All controllers (for testing)

**Usage:**
```python
# Simple: get standard setup
controller_nodes = controllers.get_standard_controllers(
    primary_controller='joint_trajectory_controller',
    use_sim_time=False
)

# Advanced: custom sequencing
joint_state_bc = controllers.get_joint_state_broadcaster()
trajectory_ctrl = controllers.get_trajectory_controller()
sequenced = controllers.sequence_after(joint_state_bc, trajectory_ctrl)
```

---

### `sensors.py`
Spawns sensor broadcasters (F/T sensors, IMU).

**Functions:**
- `get_tcp_ft_broadcaster(use_sim_time)` → TCP force/torque sensor
- `get_forearm_ft_broadcaster(use_sim_time)` → Forearm force/torque sensor
- `get_wrist_ft_broadcaster(use_sim_time)` → Wrist force/torque sensor
- `get_imu_broadcaster(use_sim_time)` → IMU sensor
- `get_all_ft_broadcasters(use_sim_time)` → All F/T sensors
- `get_all_sensors(use_sim_time, after_node)` → All sensors with optional sequencing

**Usage:**
```python
# Get all sensors, start after joint_state_broadcaster
sensors = sensors.get_all_sensors(
    use_sim_time=True,
    after_node=joint_state_broadcaster
)
```

---

### `gazebo.py`
Launches Gazebo Harmonic simulation.

**Functions:**
- `get_world_path(world_file)` → Absolute path to world file
- `get_gazebo_server(world_file, gui, verbose, run)` → Launch Gazebo
- `spawn_robot(robot_name, z_position, use_sim_time)` → Spawn robot in Gazebo
- `get_clock_bridge()` → Bridge /clock topic
- `get_standard_gazebo_setup(world_file, robot_name, gui)` → Standard Gazebo setup

**Usage:**
```python
# Simple: standard setup
gazebo_nodes = gazebo.get_standard_gazebo_setup(
    world_file='empty_world.sdf',
    gui=True
)
# Returns [gazebo_server, spawn_robot, clock_bridge]

# Advanced: custom configuration
gazebo_server = gazebo.get_gazebo_server(
    world_file='contact_manipulation_arena.sdf',
    gui=False,  # Headless
    verbose=2
)
```

---

### `rviz.py`
Launches RViz2 visualization.

**Functions:**
- `get_config_path(config_file, package)` → Absolute path to .rviz file
- `get_rviz(config_file, package, use_sim_time, additional_parameters)` → Launch RViz
- `get_rviz_moveit(robot_description, robot_description_semantic, kinematics_config)` → RViz with MoveIt
- `get_basic_rviz(use_sim_time)` → Basic robot view

**Usage:**
```python
# Simple: basic RViz
rviz_node = rviz.get_basic_rviz(use_sim_time=False)

# With MoveIt
rviz_node = rviz.get_rviz_moveit(
    robot_description=robot_desc,
    robot_description_semantic=srdf_dict,
    kinematics_config=kinematics_dict,
    use_sim_time=True
)
```

---

## Example: Building a Launch File

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from modules import robot_description, controllers, sensors, rviz

def generate_launch_description():
    # Arguments
    args = [
        DeclareLaunchArgument('use_rviz', default_value='true'),
    ]

    # Robot
    robot_desc = robot_description.load_urdf(use_fake_hardware=True)
    rsp = robot_description.get_robot_state_publisher(robot_desc)
    control_node = robot_description.get_control_node(robot_desc)

    # Controllers
    ctrl_nodes = controllers.get_standard_controllers()

    # Sensors
    sensor_nodes = sensors.get_all_sensors(after_node=ctrl_nodes[0])

    # RViz
    rviz_node = rviz.get_basic_rviz()

    # Compose
    return LaunchDescription(args + [
        control_node,
        rsp,
    ] + ctrl_nodes + [sensor_nodes, rviz_node])
```

That's it! ~20 lines instead of 200.

---

## Benefits

✅ **No code duplication** - Robot description loaded once, used everywhere
✅ **Easy to read** - Clear sections with descriptive function names
✅ **Easy to maintain** - Fix bug once, works everywhere
✅ **Easy to test** - Test modules independently
✅ **Easy to extend** - Add new functions without touching existing code
✅ **Type hints** - Functions document their parameters
✅ **Reusable** - Use in new launch files without copy-paste

---

## Adding New Modules

1. Create `new_module.py` in `modules/`
2. Write functions that return Nodes or Actions
3. Add to `__init__.py`
4. Import in launch files: `from modules import new_module`

**Example:**
```python
# modules/my_new_feature.py
def get_my_node(param1: str, param2: bool = False):
    """Description of what this does."""
    return Node(
        package='my_package',
        executable='my_executable',
        parameters=[{'param1': param1, 'param2': param2}]
    )
```

---

## Migration from Old Launch Files

Old launch files are backed up as `.backup` files. To migrate:

1. Identify repeated code blocks
2. Extract to module function
3. Replace in all launch files with module call
4. Delete backup files once tested

---

## Philosophy Notes

This is **NOT a ROS2 standard**, but it's standard **Python**:

- **Modules** - Organize related functions
- **Functions** - Do one thing well
- **DRY** - Don't repeat yourself
- **Composition** - Build complex from simple

ROS2 typically uses `IncludeLaunchDescription` for composition, but that still has duplication. This approach is more Pythonic and eliminates ~80% of repeated code.

---

## Questions?

**Q: Can I still use IncludeLaunchDescription?**
A: Yes! These modules work alongside it.

**Q: Can I launch module functions directly with `ros2 launch`?**
A: No, they're Python functions, not launch files. Use them in launch files.

**Q: Is this maintainable by others?**
A: Yes! Clear function names, docstrings, and this README make it easy to understand.

**Q: What if I need custom behavior?**
A: Just use the base functions and customize parameters, or create your own function.

**Q: Can I mix with old-style launch code?**
A: Absolutely. Modules return standard ROS2 Nodes/Actions.

---

Happy launching! 🚀
