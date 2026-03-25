#!/bin/bash
# Build ROS2 Docker image for Agent ROS Bridge

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

IMAGE_NAME="agent-ros-bridge"
TAG="ros2-jazzy"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Building ROS2 Docker image...${NC}"
echo "Image: ${IMAGE_NAME}:${TAG}"
echo ""

# Check if Dockerfile exists
if [ ! -f "${PROJECT_ROOT}/docker/Dockerfile.ros2" ]; then
    echo -e "${YELLOW}Dockerfile not found, creating minimal ROS2 image...${NC}"
    mkdir -p "${PROJECT_ROOT}/docker"
    cat > "${PROJECT_ROOT}/docker/Dockerfile.ros2" << 'EOF'
FROM ros:jazzy-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-demo-nodes-py \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Source ROS2 in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Set environment
ENV ROS_DISTRO=jazzy
ENV DEBIAN_FRONTEND=noninteractive

CMD ["bash"]
EOF
fi

# Build image
echo "Building Docker image..."
docker build \
    -f "${PROJECT_ROOT}/docker/Dockerfile.ros2" \
    -t "${IMAGE_NAME}:${TAG}" \
    "${PROJECT_ROOT}"

echo ""
echo -e "${GREEN}✅ Image built successfully: ${IMAGE_NAME}:${TAG}${NC}"
echo ""
echo "To start the container:"
echo "  ./scripts/docker/start-ros2.sh"
