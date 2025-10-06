#!/bin/bash

################################################################################
# Complete Installation Script for Humanoid Arm 5-DOF ROS2 Project
#
# This script installs:
# 1. ROS2 Jazzy (binary packages)
# 2. ros2_control (from source)
# 3. MoveIt2 (from source)
# 4. Gazebo Harmonic and additional tools
# 5. Project dependencies
#
# Requirements:
# - Ubuntu 24.04 (Noble Numbat)
# - Internet connection
# - ~20GB free disk space
# - 8GB+ RAM recommended
#
# Usage:
#   chmod +x install_all.sh
#   ./install_all.sh
#
################################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running on Ubuntu 24.04
check_ubuntu_version() {
    log_info "Checking Ubuntu version..."

    if [ -f /etc/os-release ]; then
        . /etc/os-release
        if [ "$VERSION_ID" != "24.04" ]; then
            log_error "This script requires Ubuntu 24.04 (Noble Numbat)"
            log_error "Current version: $VERSION_ID"
            exit 1
        fi
    else
        log_error "Cannot detect Ubuntu version"
        exit 1
    fi

    log_success "Ubuntu 24.04 detected"
}

# Update system
update_system() {
    log_info "Updating system packages..."
    sudo apt update
    sudo apt upgrade -y
    log_success "System updated"
}

# Set locale
setup_locale() {
    log_info "Setting up locale..."
    sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    log_success "Locale configured"
}

################################################################################
# SECTION 1: ROS2 Jazzy Installation (Binary)
################################################################################

install_ros2_jazzy() {
    log_info "===== Installing ROS2 Jazzy (Binary Packages) ====="

    # Enable universe repository
    log_info "Enabling Ubuntu Universe repository..."
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe -y
    sudo apt update

    # Add ROS2 GPG key
    log_info "Adding ROS2 GPG key..."
    sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    # Add ROS2 repository
    log_info "Adding ROS2 repository..."
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update

    # Install ROS2 Desktop Full
    log_info "Installing ROS2 Jazzy Desktop Full (this may take a while)..."
    sudo apt install -y ros-jazzy-desktop-full

    # Install development tools
    log_info "Installing ROS2 development tools..."
    sudo apt install -y ros-dev-tools

    # Install additional build tools
    sudo apt install -y \
        python3-colcon-mixin \
        python3-pytest-cov \
        python3-pytest-repeat \
        python3-pytest-rerunfailures

    # Initialize rosdep
    log_info "Initializing rosdep..."
    if [ ! -d "/etc/ros/rosdep" ]; then
        sudo rosdep init
    else
        log_warning "rosdep already initialized, skipping..."
    fi
    rosdep update

    # Setup colcon mixins
    log_info "Setting up colcon mixins..."
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml || true
    colcon mixin update default || true

    log_success "ROS2 Jazzy installed successfully"
}

################################################################################
# SECTION 2: Gazebo Harmonic Installation
################################################################################

install_gazebo_harmonic() {
    log_info "===== Installing Gazebo Harmonic ====="

    # Add Gazebo repository
    log_info "Adding Gazebo repository..."
    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt update

    # Install Gazebo Harmonic
    log_info "Installing Gazebo Harmonic..."
    sudo apt install -y gz-harmonic

    # Install ROS-Gazebo integration
    log_info "Installing ROS-Gazebo integration packages..."
    sudo apt install -y \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-image \
        ros-jazzy-gz-ros2-control

    log_success "Gazebo Harmonic installed successfully"
}

################################################################################
# SECTION 3: Additional ROS2 Packages
################################################################################

install_additional_ros2_packages() {
    log_info "===== Installing Additional ROS2 Packages ====="

    # URDF and Robot Description
    log_info "Installing URDF and robot description tools..."
    sudo apt install -y \
        ros-jazzy-xacro \
        ros-jazzy-urdf \
        liburdfdom-tools \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-joint-state-publisher-gui

    # Visualization
    log_info "Installing visualization tools..."
    sudo apt install -y \
        ros-jazzy-rviz2 \
        ros-jazzy-rviz-common \
        ros-jazzy-rviz-default-plugins

    # TF2
    log_info "Installing TF2 tools..."
    sudo apt install -y \
        ros-jazzy-tf2-ros \
        ros-jazzy-tf2-tools \
        ros-jazzy-tf2-geometry-msgs

    # Joystick and Teleoperation
    log_info "Installing joystick and teleoperation support..."
    sudo apt install -y \
        ros-jazzy-joy \
        ros-jazzy-teleop-twist-joy \
        joystick \
        jstest-gtk

    # Diagnostic and Monitoring Tools
    log_info "Installing diagnostic tools..."
    sudo apt install -y \
        ros-jazzy-rqt \
        ros-jazzy-rqt-common-plugins \
        ros-jazzy-rqt-joint-trajectory-controller \
        ros-jazzy-rqt-controller-manager \
        ros-jazzy-plotjuggler-ros

    # Bag recording
    log_info "Installing bag recording tools..."
    sudo apt install -y \
        ros-jazzy-rosbag2 \
        ros-jazzy-rosbag2-storage-default-plugins

    log_success "Additional ROS2 packages installed successfully"
}

################################################################################
# SECTION 4: Python Dependencies
################################################################################

install_python_dependencies() {
    log_info "===== Installing Python Dependencies ====="

    # Core Python tools
    sudo apt install -y \
        python3-pip \
        python3-dev \
        python3-venv

    # Python GUI libraries
    log_info "Installing Python GUI libraries..."
    sudo apt install -y \
        python3-tk \
        python3-matplotlib \
        python3-pyqt5 \
        qttools5-dev-tools

    # Scientific computing
    log_info "Installing scientific computing libraries..."
    sudo apt install -y \
        python3-numpy \
        python3-scipy

    # Development tools
    pip3 install --user pylint black flake8

    log_success "Python dependencies installed successfully"
}

################################################################################
# SECTION 5: Development Tools
################################################################################

install_development_tools() {
    log_info "===== Installing Development Tools ====="

    # Build tools
    sudo apt install -y \
        build-essential \
        cmake \
        git \
        gitk \
        git-gui

    # C++ development
    log_info "Installing C++ development tools..."
    sudo apt install -y \
        clang-format \
        clangd \
        gdb \
        ros-jazzy-ament-clang-format \
        ros-jazzy-ament-clang-tidy

    # Python linting
    sudo apt install -y \
        python3-flake8 \
        ros-jazzy-ament-flake8 \
        ros-jazzy-ament-pep257

    # Documentation
    sudo apt install -y \
        doxygen \
        graphviz \
        ros-jazzy-ament-cmake-doxygen

    # System monitoring
    sudo apt install -y \
        htop \
        iotop \
        tmux

    log_success "Development tools installed successfully"
}

################################################################################
# SECTION 6: CAN Bus Tools (for Robstride Motors)
################################################################################

install_can_tools() {
    log_info "===== Installing CAN Bus Tools ====="

    sudo apt install -y can-utils

    log_success "CAN bus tools installed successfully"
}

################################################################################
# SECTION 7: ros2_control (from source)
################################################################################

install_ros2_control() {
    log_info "===== Building ros2_control from Source ====="

    # Create workspace
    local WS_DIR="$HOME/ros2_control_ws"
    log_info "Creating workspace at $WS_DIR..."
    mkdir -p "$WS_DIR/src"
    cd "$WS_DIR"

    # Source ROS2
    source /opt/ros/jazzy/setup.bash

    # Create repos file
    log_info "Creating ros2_control.repos file..."
    cat > ros2_control.repos << 'EOF'
repositories:
  ros2_control:
    type: git
    url: https://github.com/ros-controls/ros2_control.git
    version: jazzy
  ros2_controllers:
    type: git
    url: https://github.com/ros-controls/ros2_controllers.git
    version: jazzy
  control_msgs:
    type: git
    url: https://github.com/ros-controls/control_msgs.git
    version: jazzy
  realtime_tools:
    type: git
    url: https://github.com/ros-controls/realtime_tools.git
    version: jazzy
  control_toolbox:
    type: git
    url: https://github.com/ros-controls/control_toolbox.git
    version: jazzy
EOF

    # Clone repositories
    log_info "Cloning ros2_control repositories..."
    vcs import src < ros2_control.repos

    # Install dependencies
    log_info "Installing ros2_control dependencies..."
    rosdep install --from-paths src --ignore-src -y -r

    # Build
    log_info "Building ros2_control (this will take 5-15 minutes)..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)

    log_success "ros2_control built successfully"
}

################################################################################
# SECTION 8: MoveIt2 (from source)
################################################################################

install_moveit2() {
    log_info "===== Building MoveIt2 from Source ====="

    # Create workspace
    local WS_DIR="$HOME/moveit2_ws"
    log_info "Creating workspace at $WS_DIR..."
    mkdir -p "$WS_DIR/src"
    cd "$WS_DIR/src"

    # Source ROS2 and ros2_control
    source /opt/ros/jazzy/setup.bash
    source "$HOME/ros2_control_ws/install/setup.bash"

    # Clone MoveIt2
    log_info "Cloning MoveIt2 repositories..."
    git clone https://github.com/moveit/moveit2.git -b jazzy

    # Import dependencies
    log_info "Importing MoveIt2 dependencies..."
    cd moveit2
    vcs import < moveit2.repos --recursive || true

    cd "$WS_DIR"

    # Install dependencies
    log_info "Installing MoveIt2 dependencies..."
    rosdep install --from-paths src --ignore-src -y -r

    # Install additional planners
    log_info "Installing additional motion planners..."
    sudo apt install -y \
        ros-jazzy-pilz-industrial-motion-planner || true

    # Build
    log_info "Building MoveIt2 (this will take 20-60 minutes)..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)

    log_success "MoveIt2 built successfully"
}

################################################################################
# SECTION 9: Workspace Setup
################################################################################

setup_project_workspace() {
    log_info "===== Setting Up Project Workspace ====="

    local WS_DIR="$HOME/ros2_ws"

    if [ ! -d "$WS_DIR" ]; then
        log_info "Creating project workspace at $WS_DIR..."
        mkdir -p "$WS_DIR/src"
        cd "$WS_DIR"

        # Build empty workspace
        source /opt/ros/jazzy/setup.bash
        source "$HOME/ros2_control_ws/install/setup.bash" || true
        source "$HOME/moveit2_ws/install/setup.bash" || true
        colcon build

        log_success "Project workspace created"
    else
        log_warning "Project workspace already exists at $WS_DIR"
    fi
}

################################################################################
# SECTION 10: Bashrc Configuration
################################################################################

configure_bashrc() {
    log_info "===== Configuring ~/.bashrc ====="

    local BASHRC="$HOME/.bashrc"

    # Backup bashrc
    cp "$BASHRC" "$BASHRC.backup.$(date +%Y%m%d_%H%M%S)"

    # Remove old ROS entries
    sed -i '/opt\/ros\/jazzy/d' "$BASHRC"
    sed -i '/ros2_control_ws/d' "$BASHRC"
    sed -i '/moveit2_ws/d' "$BASHRC"
    sed -i '/ros2_ws\/install/d' "$BASHRC"
    sed -i '/colcon_argcomplete/d' "$BASHRC"

    # Add new entries
    log_info "Adding ROS2 environment to ~/.bashrc..."
    cat >> "$BASHRC" << 'EOF'

# ROS2 Jazzy Environment
source /opt/ros/jazzy/setup.bash

# ros2_control workspace
if [ -f "$HOME/ros2_control_ws/install/setup.bash" ]; then
    source $HOME/ros2_control_ws/install/setup.bash
fi

# MoveIt2 workspace
if [ -f "$HOME/moveit2_ws/install/setup.bash" ]; then
    source $HOME/moveit2_ws/install/setup.bash
fi

# Project workspace
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source $HOME/ros2_ws/install/setup.bash
fi

# Colcon tab completion
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

EOF

    log_success "~/.bashrc configured"
}

################################################################################
# SECTION 11: Post-Installation Configuration
################################################################################

configure_joystick_permissions() {
    log_info "===== Configuring Joystick Permissions ====="

    # Add user to input group
    sudo usermod -a -G input "$USER"

    # Create udev rule for joystick
    log_info "Creating udev rule for joystick..."
    sudo bash -c 'cat > /etc/udev/rules.d/99-joystick.rules << EOF
KERNEL=="js*", MODE="0666"
EOF'

    sudo udevadm control --reload-rules
    sudo udevadm trigger

    log_success "Joystick permissions configured"
}

################################################################################
# SECTION 12: Verification
################################################################################

verify_installation() {
    log_info "===== Verifying Installation ====="

    # Source ROS2
    source /opt/ros/jazzy/setup.bash

    # Check ROS2
    log_info "Checking ROS2 installation..."
    if ros2 --version &> /dev/null; then
        log_success "ROS2 Jazzy: $(ros2 --version)"
    else
        log_error "ROS2 verification failed"
    fi

    # Check Gazebo
    log_info "Checking Gazebo installation..."
    if gz sim --version &> /dev/null; then
        log_success "Gazebo: $(gz sim --version | head -n1)"
    else
        log_error "Gazebo verification failed"
    fi

    # Check workspaces
    log_info "Checking workspaces..."

    if [ -d "$HOME/ros2_control_ws/install" ]; then
        log_success "ros2_control workspace: $HOME/ros2_control_ws"
    else
        log_warning "ros2_control workspace not found"
    fi

    if [ -d "$HOME/moveit2_ws/install" ]; then
        log_success "MoveIt2 workspace: $HOME/moveit2_ws"
    else
        log_warning "MoveIt2 workspace not found"
    fi

    if [ -d "$HOME/ros2_ws" ]; then
        log_success "Project workspace: $HOME/ros2_ws"
    else
        log_warning "Project workspace not found"
    fi
}

################################################################################
# MAIN INSTALLATION FLOW
################################################################################

main() {
    echo "================================================================================"
    echo "  Humanoid Arm 5-DOF ROS2 - Complete Installation Script"
    echo "================================================================================"
    echo ""
    echo "This script will install:"
    echo "  - ROS2 Jazzy (binary)"
    echo "  - Gazebo Harmonic"
    echo "  - ros2_control (from source)"
    echo "  - MoveIt2 (from source)"
    echo "  - All dependencies and tools"
    echo ""
    echo "Estimated time: 30-90 minutes (depending on CPU and internet speed)"
    echo "Disk space required: ~20GB"
    echo ""

    read -p "Continue with installation? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        log_warning "Installation cancelled by user"
        exit 0
    fi

    echo ""
    log_info "Starting installation..."
    echo ""

    # System checks
    check_ubuntu_version

    # Installation steps
    update_system
    setup_locale

    install_ros2_jazzy
    install_gazebo_harmonic
    install_additional_ros2_packages
    install_python_dependencies
    install_development_tools
    install_can_tools

    install_ros2_control
    install_moveit2

    setup_project_workspace
    configure_bashrc
    configure_joystick_permissions

    # Verification
    verify_installation

    echo ""
    echo "================================================================================"
    log_success "Installation Complete!"
    echo "================================================================================"
    echo ""
    echo "Next steps:"
    echo ""
    echo "1. Log out and log back in (for group permissions to take effect)"
    echo ""
    echo "2. Clone the humanoid arm project:"
    echo "   cd ~/ros2_ws/src"
    echo "   git clone <repository-url> humanoid_arm_5dof_ros2"
    echo ""
    echo "3. Install project dependencies:"
    echo "   cd ~/ros2_ws"
    echo "   source /opt/ros/jazzy/setup.bash"
    echo "   source ~/ros2_control_ws/install/setup.bash"
    echo "   source ~/moveit2_ws/install/setup.bash"
    echo "   rosdep install --from-paths src --ignore-src -r -y"
    echo ""
    echo "4. Build the project:"
    echo "   colcon build --symlink-install"
    echo ""
    echo "5. Test the installation:"
    echo "   source install/setup.bash"
    echo "   ros2 launch humanoid_arm_description display.launch.py"
    echo ""
    echo "Documentation:"
    echo "  - README.md - Usage and quick start"
    echo "  - CLAUDE.md - Development guide"
    echo "  - PID_README.md - PID tuning guide"
    echo ""
    echo "Installation logs saved to: /tmp/ros2_install_$(date +%Y%m%d).log"
    echo ""
}

# Run main function and log output
main 2>&1 | tee "/tmp/ros2_install_$(date +%Y%m%d_%H%M%S).log"
