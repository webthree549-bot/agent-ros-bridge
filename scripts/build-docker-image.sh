#!/bin/bash
# Docker Image Build Guide for Agent ROS Bridge
# ROS2 Jazzy + Nav2 + Gazebo

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_NAME="agent-ros-bridge"
TAG="ros2-jazzy"
FULL_IMAGE="${IMAGE_NAME}:${TAG}"

echo "=========================================="
echo "Building Agent ROS Bridge Docker Image"
echo "=========================================="
echo ""
echo "This will build a Docker image with:"
echo "  - ROS2 Jazzy (Ubuntu 24.04)"
echo "  - Nav2 navigation stack"
echo "  - Gazebo Harmonic simulation"
echo "  - TurtleBot3 robot models"
echo ""
echo "Estimated size: 8-10 GB"
echo "Estimated time: 30-60 minutes (first build)"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Cancelled"
    exit 0
fi

echo ""
echo "Step 1: Checking Docker..."
if ! command -v docker &> /dev/null; then
    echo "❌ Docker not found. Please install Docker first."
    exit 1
fi

if ! docker info &> /dev/null; then
    echo "❌ Docker daemon not running. Please start Docker."
    exit 1
fi

echo "✅ Docker is available"

echo ""
echo "Step 2: Choosing build strategy..."
echo ""
echo "Option 1: Use official ROS2 Jazzy image (RECOMMENDED)"
echo "  - Pros: Pre-built, reliable, faster"
echo "  - Cons: Large download (~8 GB)"
echo ""
echo "Option 2: Build from scratch using Dockerfile"
echo "  - Pros: Customizable, smaller if optimized"
echo "  - Cons: Slower, may have network issues"
echo ""
echo "Option 3: Use existing container (if available)"
echo "  - Pros: Instant"
echo "  - Cons: May not have all dependencies"
echo ""

read -p "Choose option (1/2/3): " option

case $option in
    1)
        echo ""
        echo "Step 3: Pulling official ROS2 Jazzy image..."
        echo "This will download ~8 GB. Press Ctrl+C to cancel."
        sleep 3
        
        docker pull osrf/ros:jazzy-desktop-full
        
        echo ""
        echo "Step 4: Installing additional packages..."
        
        # Create a temporary Dockerfile for additional packages
        cat > /tmp/Dockerfile.agentros << 'DOCKERFILE'
FROM osrf/ros:jazzy-desktop-full

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install additional packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-turtlebot3-navigation2 \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-demo-nodes-py \
    && rm -rf /var/lib/apt/lists/*

# Setup workspace
RUN mkdir -p /workspace
WORKDIR /workspace

# Source ROS2 in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc

# Default command
CMD ["bash"]
DOCKERFILE
        
        echo "Building custom layer on top of official image..."
        docker build -f /tmp/Dockerfile.agentros -t ${FULL_IMAGE} /tmp
        
        rm /tmp/Dockerfile.agentros
        
        echo ""
        echo "✅ Image built successfully: ${FULL_IMAGE}"
        ;;
        
    2)
        echo ""
        echo "Step 3: Building from Dockerfile..."
        
        if [ -f "${PROJECT_ROOT}/docker/Dockerfile.ros2.jazzy" ]; then
            echo "Using existing Dockerfile: docker/Dockerfile.ros2.jazzy"
            docker build -f ${PROJECT_ROOT}/docker/Dockerfile.ros2.jazzy -t ${FULL_IMAGE} ${PROJECT_ROOT}
        elif [ -f "${PROJECT_ROOT}/docker/Dockerfile.simulation" ]; then
            echo "Using existing Dockerfile: docker/Dockerfile.simulation"
            docker build -f ${PROJECT_ROOT}/docker/Dockerfile.simulation -t ${FULL_IMAGE} ${PROJECT_ROOT}
        else
            echo "❌ No Dockerfile found in docker/"
            echo "Creating minimal Dockerfile..."
            
            cat > /tmp/Dockerfile.minimal << 'DOCKERFILE'
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-demo-nodes-py \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /workspace
CMD ["bash"]
DOCKERFILE
            
            docker build -f /tmp/Dockerfile.minimal -t ${FULL_IMAGE} /tmp
            rm /tmp/Dockerfile.minimal
        fi
        
        echo ""
        echo "✅ Image built successfully: ${FULL_IMAGE}"
        ;;
        
    3)
        echo ""
        echo "Step 3: Using existing container..."
        
        # Check for existing ros2_humble or ros2_jazzy container
        existing=$(docker ps -a --format '{{.Names}}' | grep -E "ros2_(humble|jazzy)" | head -1)
        
        if [ -n "$existing" ]; then
            echo "Found existing container: $existing"
            read -p "Start this container? (y/n) " -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                docker start $existing
                echo "✅ Container started: $existing"
                echo "Enter with: docker exec -it $existing bash"
            fi
        else
            echo "❌ No existing container found"
            echo "Run Option 1 or 2 first"
            exit 1
        fi
        ;;
        
    *)
        echo "Invalid option"
        exit 1
        ;;
esac

echo ""
echo "=========================================="
echo "Build Complete!"
echo "=========================================="
echo ""
echo "Image: ${FULL_IMAGE}"
echo ""
echo "Next steps:"
echo ""
echo "1. Start container:"
echo "   docker run -d --name ros2_jazzy --privileged \\"
echo "     -p 8765:8765 -p 11311:11311 \\"
echo "     -v $(pwd):/workspace:rw \\"
echo "     ${FULL_IMAGE}"
echo ""
echo "2. Enter container:"
echo "   docker exec -it ros2_jazzy bash"
echo ""
echo "3. Source ROS2:"
echo "   source /opt/ros/jazzy/setup.bash"
echo ""
echo "4. Run tests:"
echo "   python3 scripts/test_real_gazebo_integration.py"
echo ""
