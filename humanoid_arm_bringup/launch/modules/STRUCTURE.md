# Module Structure - Type-Organized

Visual overview of the module organization.

## Directory Tree

```
humanoid_arm_bringup/launch/modules/
│
├── __init__.py                      # Main module entry point
├── README.md                        # Module documentation
├── IMPORT_STYLES.md                 # Import style guide
├── STRUCTURE.md                     # This file
│
├── providers/                       # 🔧 Core System Providers
│   ├── __init__.py
│   ├── robot_description.py         # URDF, robot_state_publisher
│   ├── controllers.py               # ros2_control controllers
│   ├── sensors.py                   # Sensor broadcasters
│   └── gazebo.py                    # Simulation environment
│
├── gui/                             # 🖥️ GUI Tools
│   ├── __init__.py
│   ├── rviz.py                      # RViz2 visualization
│   └── rqt.py                       # RQt debugging tools
│
└── utilities/                       # 🛠️ Utilities
    └── __init__.py                  # (Reserved for future)
```

---

## Module Categories

### 🔧 providers/ - Core System Providers

**Purpose:** Fundamental robot functionality - the "backend" of your robot system.

| Module | Purpose | Key Functions |
|--------|---------|---------------|
| `robot_description.py` | Robot model | `load_urdf()`, `get_robot_state_publisher()` |
| `controllers.py` | Motion control | `get_standard_controllers()`, `spawn_controller()` |
| `sensors.py` | Sensor data | `get_all_sensors()`, `get_imu_broadcaster()` |
| `gazebo.py` | Simulation | `get_gazebo_server()`, `spawn_robot()` |

**Analogy:** Like Angular **services** - provide data and functionality.

---

### 🖥️ gui/ - Graphical User Interface

**Purpose:** Visualization and user interaction - the "frontend" of your robot system.

| Module | Purpose | Key Functions |
|--------|---------|---------------|
| `rviz.py` | 3D visualization | `get_basic_rviz()`, `get_rviz_moveit()` |
| `rqt.py` | Debug/monitor tools | `get_rqt_joint_trajectory_controller()`, `get_rqt_plot()` |

**Analogy:** Like Angular **components** - user-facing interfaces.

---

### 🛠️ utilities/ - Helper Functions

**Purpose:** Common utilities and helper functions (future expansion).

**Planned additions:**
- `timing.py` - Timer and delay helpers
- `sequencing.py` - Event sequencing utilities
- `validation.py` - Parameter validation
- `logging.py` - Logging configuration

**Analogy:** Like Angular **pipes/utils** - helper functions.

---

## Import Patterns

### Pattern 1: By Category (Recommended)
```python
from modules.providers import robot_description, controllers
from modules.gui import rviz, rqt
```
✅ Clear organization by type

### Pattern 2: Direct (Backward Compatible)
```python
from modules import robot_description, controllers, rviz, rqt
```
✅ Shortest, works with existing files

### Pattern 3: Namespace
```python
from modules import providers, gui

# Use: providers.robot_description.load_urdf()
```
✅ Most explicit

---

## Module Relationships

```
┌─────────────────────────────────────────┐
│          Launch File                    │
│  (robot.launch.py, gazebo.launch.py)   │
└──────────────┬──────────────────────────┘
               │
               │ imports
               ▼
┌──────────────────────────────────────────┐
│             modules/                     │
├──────────────────────────────────────────┤
│                                          │
│  ┌──────────────┐  ┌─────────────┐     │
│  │  providers/  │  │    gui/     │     │
│  ├──────────────┤  ├─────────────┤     │
│  │ robot_desc   │  │   rviz      │     │
│  │ controllers  │  │   rqt       │     │
│  │ sensors      │  └─────────────┘     │
│  │ gazebo       │                       │
│  └──────────────┘                       │
│                                          │
│           ▼ returns                      │
│     Nodes, Actions                       │
│                                          │
└──────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────┐
│        LaunchDescription                 │
│     (Composed and returned)              │
└──────────────────────────────────────────┘
```

---

## Adding New Modules

### Adding to Existing Category

**Example: Add `moveit.py` to providers/**

1. Create file:
```bash
touch providers/moveit.py
```

2. Write functions:
```python
# providers/moveit.py
def get_move_group(use_sim_time=True):
    return IncludeLaunchDescription(...)
```

3. Update `providers/__init__.py`:
```python
from . import moveit

__all__ = [..., 'moveit']
```

4. Use in launch files:
```python
from modules.providers import moveit
move_group = moveit.get_move_group()
```

---

### Adding New Category

**Example: Add `hardware/` category**

1. Create folder and `__init__.py`:
```bash
mkdir hardware
touch hardware/__init__.py
```

2. Add modules:
```bash
touch hardware/robstride.py
touch hardware/can_bus.py
```

3. Update category `__init__.py`:
```python
# hardware/__init__.py
from . import robstride, can_bus
__all__ = ['robstride', 'can_bus']
```

4. Update main `modules/__init__.py`:
```python
from . import hardware
__all__ = [..., 'hardware']
```

5. Use:
```python
from modules.hardware import robstride
motor_node = robstride.get_motor_interface()
```

---

## Benefits of This Structure

### For Development
✅ **Easy to find** - Know where to look for functionality
✅ **Easy to add** - Clear place for new modules
✅ **Clear purpose** - Folder name indicates module type

### For Collaboration
✅ **Self-documenting** - Structure explains organization
✅ **Consistent** - All modules follow same pattern
✅ **Scalable** - Easy to expand without confusion

### For Maintenance
✅ **Logical grouping** - Related modules together
✅ **Separation of concerns** - Each folder has specific purpose
✅ **Type-safe-ish** - Folders indicate module category

---

## Comparison with Other Frameworks

### Django (Python Web Framework)
```
myapp/
├── models.py       # Data models
├── views.py        # Request handlers
├── urls.py         # URL routing
└── serializers.py  # Data serialization
```
**Similar:** Organize by purpose/type

### Angular (TypeScript Framework)
```
src/app/
├── services/       # Data providers
├── components/     # UI components
├── pipes/          # Data transformers
└── guards/         # Route protection
```
**Similar:** Organize by type (what we did!)

### ROS2 Standard
```
my_package/
├── src/            # C++ source
├── include/        # C++ headers
├── scripts/        # Python scripts
└── launch/         # Launch files
```
**Different:** Organize by language/purpose

**Our approach:** Inspired by web frameworks, applied to ROS2 launch files!

---

## Summary

```
modules/
├── providers/    🔧 Core functionality (backend)
├── gui/          🖥️ User interfaces (frontend)
└── utilities/    🛠️ Helper functions (utils)
```

**Type-organized, web-framework-inspired, Python-idiomatic!**

---

## Next Steps

Explore the modules:
- Read **`README.md`** for function documentation
- Read **`IMPORT_STYLES.md`** for import patterns
- Check individual module files for details

Start using:
```python
from modules.providers import robot_description, controllers
from modules.gui import rviz, rqt

# Build your launch file!
```

🚀 Happy launching!
