#!/bin/bash
#
# ROS2 Humble Docker Setup for macOS
# Faster alternative to building from source
#

set -e

echo "ROS2 Humble Docker Setup"
echo "======================================"

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "❌ Docker not found. Please install Docker Desktop for Mac:"
    echo "   https://www.docker.com/products/docker-desktop"
    exit 1
fi

echo "✅ Docker found"

# Check if Docker is running
if ! docker info &> /dev/null; then
    echo "❌ Docker is not running. Please start Docker Desktop."
    exit 1
fi

echo "✅ Docker is running"

# Create workspace directory
WORKSPACE_DIR="/Volumes/2nd-HD/ros2_humble_docker"
mkdir -p "$WORKSPACE_DIR"
cd "$WORKSPACE_DIR"

echo ""
echo "Workspace: $WORKSPACE_DIR"

# Create Dockerfile
cat > Dockerfile << 'EOF'
FROM osrf/ros:humble-desktop-full

# Install additional packages for Agent ROS Bridge
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-turtlebot3* \
    ros-humble-nav2* \
    ros-humble-gazebo-* \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir \
    setuptools==59.6.0 \
    vcstool \
    rosdep

# Source ROS2 automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set environment variables
ENV ROS_DISTRO=humble
ENV TURTLEBOT3_MODEL=burger

WORKDIR /workspace

CMD ["/bin/bash"]
EOF

echo ""
echo "Building Docker image (this will take 10-20 minutes first time)..."
docker build -t agent-ros-bridge:humble .

echo ""
echo "✅ Docker image built successfully!"

# Create helper scripts
mkdir -p "$WORKSPACE_DIR/scripts"

# Start script
cat > "$WORKSPACE_DIR/scripts/start_ros2.sh" << 'EOF'
#!/bin/bash
# Start ROS2 Humble Docker container

WORKSPACE_DIR="/Volumes/2nd-HD/ros2_humble_docker"

docker run -it --rm \
    --name ros2_humble \
    --privileged \
    -e DISPLAY=host.docker.internal:0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$WORKSPACE_DIR/workspace:/workspace" \
    -p 11311:11311 \
    -p 8765:8765 \
    agent-ros-bridge:humble
EOF

chmod +x "$WORKSPACE_DIR/scripts/start_ros2.sh"

# Gazebo script
cat > "$WORKSPACE_DIR/scripts/start_gazebo.sh" << 'EOF'
#!/bin/bash
# Start Gazebo with TurtleBot3 in Docker

docker run -it --rm \
    --name ros2_gazebo \
    --privileged \
    -e DISPLAY=host.docker.internal:0 \
    -e TURTLEBOT3_MODEL=burger \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -p 11311:11311 \
    agent-ros-bridge:humble \
    bash -c "source /opt/ros/humble/setup.bash && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
EOF

chmod +x "$WORKSPACE_DIR/scripts/start_gazebo.sh"

# Nav2 script
cat > "$WORKSPACE_DIR/scripts/start_nav2.sh" << 'EOF'
#!/bin/bash
# Start Nav2 navigation in Docker

docker run -it --rm \
    --name ros2_nav2 \
    --privileged \
    -e DISPLAY=host.docker.internal:0 \
    -e TURTLEBOT3_MODEL=burger \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    agent-ros-bridge:humble \
    bash -c "source /opt/ros/humble/setup.bash && ros2 launch nav2_bringup navigation_launch.py"
EOF

chmod +x "$WORKSPACE_DIR/scripts/start_nav2.sh"

# Test script
cat > "$WORKSPACE_DIR/scripts/test_ros2.sh" << 'EOF'
#!/bin/bash
# Test ROS2 installation in Docker

docker run -it --rm \
    --name ros2_test \
    agent-ros-bridge:humble \
    bash -c "source /opt/ros/humble/setup.bash && ros2 --help && echo '✅ ROS2 Humble is working!'"
EOF

chmod +x "$WORKSPACE_DIR/scripts/test_ros2.sh"

# Create workspace directory for shared files
mkdir -p "$WORKSPACE_DIR/workspace"

echo ""
echo "======================================"
echo "✅ ROS2 Humble Docker setup complete!"
echo "======================================"
echo ""
echo "Usage:"
echo ""
echo "1. Test ROS2:"
echo "   $WORKSPACE_DIR/scripts/test_ros2.sh"
echo ""
echo "2. Start interactive ROS2 shell:"
echo "   $WORKSPACE_DIR/scripts/start_ros2.sh"
echo ""
echo "3. Start Gazebo with TurtleBot3:"
echo "   $WORKSPACE_DIR/scripts/start_gazebo.sh"
echo ""
echo "4. Start Nav2 navigation:"
echo "   $WORKSPACE_DIR/scripts/start_nav2.sh"
echo ""
echo "Note: For GUI applications (Gazebo, RViz), you need X11 forwarding:"
echo "   brew install xquartz"
echo "   open -a XQuartz"
echo "   xhost +localhost"
echo ""
echo "Workspace directory: $WORKSPACE_DIR"
