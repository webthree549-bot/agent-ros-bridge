#!/bin/bash
# Complete ROS2 build - build all packages

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
pip install 'setuptools<80,>=30.3.0' --force-reinstall

echo ""
echo "Starting ROS2 build (this will take 1-3 hours)..."
echo "======================================"

# Build all packages
colcon build \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_OSX_ARCHITECTURES=arm64 \
        -DBUILD_TESTING=OFF \
        -DINSTALL_EXAMPLES=OFF \
        -DCMAKE_OSX_DEPLOYMENT_TARGET=14.0 \
    --event-handlers console_direct+ \
    2>&1 | tee /tmp/ros2_build_final.log

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
echo "======================================"
echo "ROS2 Humble build complete!"
echo "======================================"
echo ""
echo "To use ROS2, run:"
echo "  source /Volumes/2nd-HD/ros2_humble/setup.bash"
echo ""
echo "Test installation:"
echo "  ros2 --help"
