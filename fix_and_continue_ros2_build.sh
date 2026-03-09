#!/bin/bash
# Fix ROS2 build - retry failed clones and continue

set -e

INSTALL_DIR="/Volumes/2nd-HD/ros2_humble"
WORKSPACE_DIR="$INSTALL_DIR/ros2_ws"
SRC_DIR="$WORKSPACE_DIR/src"

echo "Fixing ROS2 build issues..."
echo "======================================"

# Source the virtual environment
source "$INSTALL_DIR/venv/bin/activate"
cd "$WORKSPACE_DIR"

# Retry vcs import to fix failed clones
echo "Retrying failed git clones..."
vcs import "$SRC_DIR" < ros2.repos --retry 3 --skip-existing

# Check for any remaining empty directories
echo "Checking for failed clones..."
find "$SRC_DIR" -type d -empty | while read dir; do
    echo "Empty directory found: $dir - will be removed and retried"
    rm -rf "$dir"
done

# Retry import again for any removed directories
vcs import "$SRC_DIR" < ros2.repos --retry 3

echo "Git clones complete!"

# Now install Python dependencies
echo "Installing Python dependencies..."
pip install -U \
    argcomplete catkin_pkg coverage cryptography empy \
    flake8 importlib-metadata lark lxml mock mypy netifaces \
    numpy pydocstyle pydot pyyaml setuptools vcstool wheel 2>&1 | tail -10

echo "Python dependencies installed!"

# Start the build
echo ""
echo "Starting ROS2 build (this will take 1-3 hours)..."
echo "======================================"

colcon build \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_OSX_ARCHITECTURES=arm64 \
        -DBUILD_TESTING=OFF \
        -DINSTALL_EXAMPLES=OFF \
        -DCMAKE_OSX_DEPLOYMENT_TARGET=14.0 \
    --packages-up-to ros_base \
    --event-handlers console_direct+ 2>&1 | tee /tmp/ros2_build_progress.log

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
