#!/bin/bash
# Fix and complete ROS2 build

set -e

INSTALL_DIR="/Volumes/2nd-HD/ros2_humble"
WORKSPACE_DIR="$INSTALL_DIR/ros2_ws"

echo "Completing ROS2 build..."
echo "======================================"

# Source the virtual environment
source "$INSTALL_DIR/venv/bin/activate"
cd "$WORKSPACE_DIR"

# Fix setuptools version
echo "Fixing setuptools version..."
pip install 'setuptools<80,>=30.3.0'

# Check available packages
echo "Checking available packages..."
ls src/ros2/ | head -20

# Build with correct package name (ros-base not ros_base)
echo ""
echo "Starting ROS2 build (this will take 1-3 hours)..."
echo "======================================"

# Build core packages first
colcon build \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_OSX_ARCHITECTURES=arm64 \
        -DBUILD_TESTING=OFF \
        -DINSTALL_EXAMPLES=OFF \
        -DCMAKE_OSX_DEPLOYMENT_TARGET=14.0 \
    --packages-up-to ament_cmake ros_core \
    --event-handlers console_direct+

echo ""
echo "Core packages built!"
echo "Building remaining packages..."

# Build the rest
colcon build \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_OSX_ARCHITECTURES=arm64 \
        -DBUILD_TESTING=OFF \
        -DINSTALL_EXAMPLES=OFF \
        -DCMAKE_OSX_DEPLOYMENT_TARGET=14.0 \
    --event-handlers console_direct+

echo ""
echo "Build complete!"

# Create setup script
cat > "$INSTALL_DIR/setup.bash" << 'EOF'
#!/bin/bash
source "/Volumes/2nd-HD/ros2_humble/venv/bin/activate"
source "/Volumes/2nd-HD/ros2_humble/ros2_ws/install/setup.bash"
export ROS_DISTRO=humble
export ROS_PYTHON_VERSION=3
EOF

chmod +x "$INSTALL_DIR/setup.bash"

echo ""
echo "ROS2 Humble build complete!"
echo "Run: source /Volumes/2nd-HD/ros2_humble/setup.bash"
