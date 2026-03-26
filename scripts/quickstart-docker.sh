#!/bin/bash
# Quick start script for Agent ROS Bridge with pre-built Docker image
# Uses agent-ros-bridge:jazzy-with-nav2 (Nav2 and Gazebo pre-installed)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONTAINER_NAME="ros2_jazzy"
IMAGE_NAME="agent-ros-bridge:jazzy-with-nav2"

echo "=========================================="
echo "Agent ROS Bridge - Quick Start"
echo "=========================================="
echo ""
echo "This script uses the pre-built image with:"
echo "  - ROS2 Jazzy"
echo "  - Nav2 navigation stack"
echo "  - Gazebo Harmonic"
echo "  - TurtleBot3 models"
echo ""

# Check if image exists
if ! docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE_NAME}$"; then
    echo "❌ Image '${IMAGE_NAME}' not found!"
    echo ""
    echo "You need to build it first. Run:"
    echo "  ./scripts/build-docker-image.sh"
    echo ""
    echo "Or pull from registry (if available):"
    echo "  docker pull yourusername/${IMAGE_NAME}"
    exit 1
fi

echo "✅ Image found: ${IMAGE_NAME}"

# Check if container already running
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "✅ Container '${CONTAINER_NAME}' is already running"
    echo ""
    echo "Enter with:"
    echo "  docker exec -it ${CONTAINER_NAME} bash"
    exit 0
fi

# Check if container exists but stopped
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "🔄 Starting existing container..."
    docker start ${CONTAINER_NAME}
    echo "✅ Container started"
    echo ""
    echo "Enter with:"
    echo "  docker exec -it ${CONTAINER_NAME} bash"
    exit 0
fi

# Create new container
echo "🚀 Creating new container..."
docker run -d \
  --name ${CONTAINER_NAME} \
  --hostname ros2-jazzy \
  --privileged \
  --restart unless-stopped \
  -p 8765:8765 \
  -p 11311:11311 \
  -p 9090:9090 \
  -v "${PROJECT_ROOT}:/workspace:rw" \
  -v /dev/shm:/dev/shm \
  -e "ROS_DOMAIN_ID=0" \
  ${IMAGE_NAME} \
  bash -c "source /opt/ros/jazzy/setup.bash && tail -f /dev/null"

echo ""
echo "✅ Container created and running!"
echo ""
echo "Quick commands:"
echo "  Enter:     docker exec -it ${CONTAINER_NAME} bash"
echo "  Stop:      docker stop ${CONTAINER_NAME}"
echo "  Restart:   docker restart ${CONTAINER_NAME}"
echo "  Logs:      docker logs ${CONTAINER_NAME}"
echo ""
echo "Inside container:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  python3 scripts/test_real_gazebo_integration.py"
