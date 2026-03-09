#!/bin/bash
# Continue ROS2 build from where it stopped

set -e

INSTALL_DIR="/Volumes/2nd-HD/ros2_humble"
WORKSPACE_DIR="$INSTALL_DIR/ros2_ws"
SRC_DIR="$WORKSPACE_DIR/src"

echo "Continuing ROS2 Humble build..."
echo "======================================"

# Step 2: Setup Python environment
echo "Setting up Python environment..."
PYTHON_EXE=$(brew --prefix python@3.10)/bin/python3.10

if [ ! -d "$INSTALL_DIR/venv" ]; then
    $PYTHON_EXE -m venv "$INSTALL_DIR/venv"
fi

source "$INSTALL_DIR/venv/bin/activate"
pip install --upgrade pip setuptools wheel
pip install colcon-common-extensions colcon-mixin vcstool

# Step 3: Create workspace
echo "Creating workspace..."
mkdir -p "$SRC_DIR"
cd "$WORKSPACE_DIR"

# Step 4: Download ROS2 source
echo "Downloading ROS2 source code..."
if [ ! -f ros2.repos ]; then
    wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
fi

if [ ! -d src/ros2 ]; then
    vcs import src < ros2.repos
fi

# Step 5: Install Python dependencies
echo "Installing Python dependencies..."
pip install -U \
    argcomplete catkin_pkg coverage cryptography empy \
    flake8 importlib-metadata lark lxml mock mypy netifaces \
    numpy pydocstyle pydot pyyaml setuptools vcstool wheel

# Step 6: Build ROS2
echo "Building ROS2 (this will take 1-3 hours)..."
colcon build \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_OSX_ARCHITECTURES=arm64 \
        -DBUILD_TESTING=OFF \
        -DINSTALL_EXAMPLES=OFF \
    --packages-up-to ros_base

echo "✓ Build complete!"

# Create setup script
cat > "$INSTALL_DIR/setup.bash" << 'EOF'
#!/bin/bash
source "/Volumes/2nd-HD/ros2_humble/venv/bin/activate"
source "/Volumes/2nd-HD/ros2_humble/ros2_ws/install/setup.bash"
export ROS_DISTRO=humble
EOF

chmod +x "$INSTALL_DIR/setup.bash"

echo ""
echo "ROS2 Humble build complete!"
echo "Run: source /Volumes/2nd-HD/ros2_humble/setup.bash"
