#!/bin/bash
#
# Simple ROS2 Humble Docker Setup (No Build Required)
# Uses official ROS2 image directly
#

set -e

echo "Simple ROS2 Humble Docker Setup"
echo "======================================"

# Check Docker
if ! command -v docker &> /dev/null; then
    echo "❌ Docker not found"
    exit 1
fi

if ! docker info &> /dev/null; then
    echo "❌ Docker not running"
    exit 1
fi

echo "✅ Docker ready"

# Pull the ROS2 Humble image (arm64 for Apple Silicon)
echo ""
echo "Pulling ROS2 Humble image (this may take 5-10 minutes)..."
docker pull --platform linux/arm64 osrf/ros:humble-desktop

# Create a simple startup script
cat > /Volumes/2nd-HD/ros2_humble/start_ros2.sh << 'EOF'
#!/bin/bash
# Start ROS2 Humble container

docker run -it --rm \
    --name ros2_humble \
    --platform linux/arm64 \
    -v /Volumes/2nd-HD/ros2_humble/workspace:/workspace \
    -p 11311:11311 \
    osrf/ros:humble-desktop \
    bash -c "
        echo 'ROS2 Humble Ready!'
        echo 'Try: ros2 run demo_nodes_cpp talker'
        bash
    "
EOF

chmod +x /Volumes/2nd-HD/ros2_humble/start_ros2.sh

echo ""
echo "======================================"
echo "✅ ROS2 Humble Docker setup complete!"
echo "======================================"
echo ""
echo "To start ROS2:"
echo "  /Volumes/2nd-HD/ros2_humble/start_ros2.sh"
echo ""
echo "Note: This uses the official ROS2 image."
echo "TurtleBot3/Nav2 packages can be installed manually inside the container."
