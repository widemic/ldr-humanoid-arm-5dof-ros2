#!/bin/bash
# Automated setup script for humanoid arm ROS2 workspace
# Run this after installing ROS2 Jazzy and major dependencies

set -e  # Exit on error

echo "========================================="
echo "Humanoid Arm ROS2 Workspace Setup"
echo "========================================="
echo ""

# Check if ROS2 is installed
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ROS2 not found. Please install ROS2 Jazzy first."
    echo "See INSTALL.md for instructions."
    exit 1
fi

echo "✓ ROS2 installation detected"
echo ""

# Check if we're in the workspace root
if [ ! -d "src" ]; then
    echo "ERROR: No 'src' directory found. Run this script from workspace root."
    exit 1
fi

echo "Step 1: Updating rosdep database..."
rosdep update

echo ""
echo "Step 2: Installing workspace dependencies..."
rosdep install --from-paths src --ignore-src -r -y

echo ""
echo "Step 3: Building workspace with colcon..."
colcon build --symlink-install

echo ""
echo "========================================="
echo "Setup Complete!"
echo "========================================="
echo ""
echo "To use the workspace, run:"
echo "  source install/setup.bash"
echo ""
echo "Or add to ~/.bashrc for automatic loading:"
echo "  echo 'source $(pwd)/install/setup.bash' >> ~/.bashrc"
echo ""
echo "Quick test:"
echo "  ros2 launch humanoid_arm_description display.launch.py"
echo ""
