#!/bin/bash
#
# Setup Nav2 in Docker Container
# 
# This script installs Nav2 and TurtleBot3 simulation packages
# inside the Docker container for real Gazebo validation.

set -e

echo "=========================================="
echo "Setting up Nav2 for Real Gazebo Validation"
echo "=========================================="
echo ""

# Check if container is running
if ! docker ps | grep -q ros2_humble; then
    echo "❌ Docker container 'ros2_humble' not running"
    echo "   Start it first: ./scripts/docker-manager.sh start"
    exit 1
fi

echo "📦 Installing packages in Docker container..."
echo ""

# Install Nav2 and TurtleBot3 packages
docker exec ros2_humble bash -c "
    apt-get update && apt-get install -y \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-nav2-simple-commander \
        ros-humble-turtlebot3-gazebo \
        ros-humble-turtlebot3-navigation2 \
        ros-humble-turtlebot3-cartographer \
    && rm -rf /var/lib/apt/lists/*
"

echo ""
echo "✅ Nav2 and TurtleBot3 packages installed!"
echo ""
echo "You can now run Gate 2 validation:"
echo "  docker exec -it ros2_humble bash"
echo "  python scripts/run_gate2_real_gazebo.py --num-scenarios 100"
echo ""
