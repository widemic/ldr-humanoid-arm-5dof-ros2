# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a 5-DOF (5 Degrees of Freedom) humanoid robotic arm originally designed in SolidWorks and exported using SW2URDF. Despite the "ros2" in the repository name, this is currently a **ROS1 (catkin-based) project**.

## Build and Development Commands

### Build Commands
```bash
# Standard catkin build (run from workspace root)
catkin_make

# Alternative with catkin tools
catkin build
```

### Launch Commands
```bash
# Launch RViz visualization with joint control GUI
roslaunch "Assem1 - Copy - Copy" display.launch

# Launch Gazebo simulation
roslaunch "Assem1 - Copy - Copy" gazebo.launch
```

### Testing
No formal test suite exists. Verify functionality by:
- Launching RViz and manipulating joints via GUI
- Running Gazebo simulation and checking robot model loads correctly

## Architecture Overview

### Package Structure
- **Build System**: Catkin (ROS1)
- **Main Package**: `"Assem1 - Copy - Copy"` (located in `/src/`)
- **Robot Model**: 13 total joints, 5 actuated revolute joints (true 5-DOF)

### Key Directories
```
src/
├── urdf/           # Robot description files
├── meshes/         # 3D models (15 STL files from SolidWorks)
├── launch/         # RViz and Gazebo launch files
├── config/         # Joint configuration YAML
└── textures/       # Empty (reserved for future textures)
```

### Actuated Joints (5-DOF)
1. `joint2`: Base rotation (-3.14 to 3.14 rad)
2. `joint3`: Shoulder (-0.55 to 3.1 rad)
3. `joint6`: Elbow (-3.14 to 3.14 rad)
4. `joint9`: Wrist 1 (-0.31 to 2.8 rad)
5. `joint11`: Wrist 2 (-3.14 to 3.14 rad)

### Key Configuration Files
- **URDF**: `src/urdf/Assem1 - Copy - Copy.urdf` - Main robot description
- **Joint Config**: `src/config/joint_names_Assem1 - Copy - Copy.yaml` - Controller joint mapping
- **Package Definition**: `src/package.xml` - ROS package metadata
- **Build Config**: `src/CMakeLists.txt` - Catkin build setup

## Current Limitations

1. **ROS1 Only**: Needs migration to ROS2 despite repository name
2. **No Controllers**: Only URDF description, no motion control implementation
3. **Missing Documentation**: No README or setup instructions
4. **Naming Issues**: Package uses temporary SolidWorks export name
5. **No Source Code**: No custom nodes or control logic

## Development Notes

- Robot was exported from SolidWorks using SW2URDF exporter
- All meshes are in STL format with original SolidWorks naming
- Package metadata (author, maintainer) needs updating from "TODO" placeholders
- Fixed joints (joint1, joint4, joint5, joint7, joint8, joint10, joint12) are structural only
- Joint limits and safety constraints are defined in URDF

## Future Development Areas

- Implement ROS2 conversion
- Add joint controllers and motion planning
- Create proper package naming and documentation
- Implement safety systems and sensor integration
- Add comprehensive testing framework