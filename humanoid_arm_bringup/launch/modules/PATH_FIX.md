# Module Path Fix

## The Problem

ROS2 launch files couldn't find the `modules` package because Python didn't know where to look.

**Error:**
```
ModuleNotFoundError: No module named 'modules'
```

## The Solution

Add the launch directory to Python's module search path at the beginning of each launch file:

```python
#!/usr/bin/env python3

import sys
from pathlib import Path

# Add modules directory to Python path
sys.path.insert(0, str(Path(__file__).parent))

# Now imports work!
from modules import robot_description, controllers
```

## Why This Works

- `Path(__file__).parent` gets the launch file's directory
- `sys.path.insert(0, ...)` adds that directory to Python's search path
- Now Python can find `modules/` in the same directory

## Applied To

All launch files now have this fix:
- ✅ `robot.launch.py`
- ✅ `gazebo.launch.py`
- ✅ `full_system.launch.py`
- ✅ `gazebo_rqt.launch.py`

## Creating New Launch Files

Always add this at the top (after docstring, before other imports):

```python
#!/usr/bin/env python3

"""Your docstring here"""

import sys
from pathlib import Path

# Add modules directory to Python path
sys.path.insert(0, str(Path(__file__).parent))

# Now you can import modules
from modules import ...
```

## Alternative Solutions (Not Used)

### Option 1: PYTHONPATH Environment Variable
```bash
export PYTHONPATH=$PYTHONPATH:/path/to/humanoid_arm_bringup/launch
ros2 launch humanoid_arm_bringup gazebo_rqt.launch.py
```
❌ Requires manual setup, not portable

### Option 2: Package Installation
Make modules a proper Python package and install it:
```bash
cd humanoid_arm_bringup/launch
pip install -e .
```
❌ Overkill for launch file modules

### Option 3: Use sys.path in sitecustomize.py
❌ System-wide change, not project-specific

### Option 4: Relative Imports
```python
from .modules import robot_description  # Doesn't work in launch files
```
❌ ROS2 launch doesn't support relative imports properly

## Our Solution (Best)

✅ **Per-file path insertion**
- Simple
- Portable
- No external dependencies
- Works immediately
- Clear and explicit

## Testing

```bash
# Should work now
ros2 launch humanoid_arm_bringup gazebo_rqt.launch.py

# All other launch files too
ros2 launch humanoid_arm_bringup robot.launch.py
ros2 launch humanoid_arm_bringup gazebo.launch.py
ros2 launch humanoid_arm_bringup full_system.launch.py
```

---

**Fixed!** 🎉
