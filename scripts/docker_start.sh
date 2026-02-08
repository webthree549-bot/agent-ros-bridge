#!/bin/bash
set -eo pipefail

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PROJECT_ROOT"

log() { echo -e "${BLUE}[DOCKER] $1${NC}"; }
success() { echo -e "${GREEN}[SUCCESS] $1${NC}"; }
warn() { echo -e "${YELLOW}[WARN] $1${NC}"; }
error() { echo -e "${RED}[ERROR] $1${NC}"; exit 1; }

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
CONTAINER_NAME="ros2-${ROS_DISTRO}-bridge"
IMAGE_NAME="ros:${ROS_DISTRO}-ros-base"

# Parse args
while [[ $# -gt 0 ]]; do
    case "$1" in
        --jazzy) ROS_DISTRO="jazzy"; shift ;;
        --humble) ROS_DISTRO="humble"; shift ;;
        --noetic) ROS_DISTRO="noetic"; shift ;;
        --stop) STOP_CONTAINER="true"; shift ;;
        --rm) REMOVE_CONTAINER="true"; shift ;;
        *) shift ;;
    esac
done

# Update names based on distro
if [[ "$ROS_DISTRO" == "noetic" ]]; then
    CONTAINER_NAME="ros1-noetic-bridge"
    IMAGE_NAME="ros:noetic-ros-base"
fi

log "============================================="
log "OpenClaw-ROS Docker Manager - v1.0.0"
log "============================================="
log "ROS Distro: $ROS_DISTRO"
log "Container: $CONTAINER_NAME"
log ""

# Stop container if requested
if [[ "${STOP_CONTAINER:-false}" == "true" ]]; then
    log "Stopping container $CONTAINER_NAME..."
    docker stop "$CONTAINER_NAME" 2>/dev/null || warn "Container not running"
    success "Container stopped"
    exit 0
fi

# Remove container if requested
if [[ "${REMOVE_CONTAINER:-false}" == "true" ]]; then
    log "Removing container $CONTAINER_NAME..."
    docker rm -f "$CONTAINER_NAME" 2>/dev/null || warn "Container not found"
    success "Container removed"
    exit 0
fi

# Check if container already exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        log "Container $CONTAINER_NAME is already running"
        log "Attaching to container..."
        docker exec -it "$CONTAINER_NAME" bash
        exit 0
    else
        log "Container $CONTAINER_NAME exists but is stopped"
        log "Starting container..."
        docker start "$CONTAINER_NAME"
        docker exec -it "$CONTAINER_NAME" bash
        exit 0
    fi
fi

# Pull image if needed
log "Checking for image $IMAGE_NAME..."
if ! docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE_NAME}$"; then
    log "Pulling Docker image $IMAGE_NAME..."
    docker pull "$IMAGE_NAME" || error "Failed to pull image. Check Docker and network."
fi

# Detect OS for network mode
OS=$(uname -s)
if [[ "$OS" == "Darwin" ]]; then
    # macOS - use port mapping (host network doesn't work)
    log "Detected macOS - using port mapping for OpenClaw connection"
    NETWORK_ARGS="-p 9999:9999 -p 9090:9090"
else
    # Linux - can use host network
    log "Detected Linux - using host networking"
    NETWORK_ARGS="--network host"
fi

# Create and start container
log "Creating container $CONTAINER_NAME..."
docker run -it \
    --name "$CONTAINER_NAME" \
    --hostname "ros-bridge" \
    -v "$PROJECT_ROOT:/app" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -e "DISPLAY=${DISPLAY:-}" \
    -e "ROS_DISTRO=$ROS_DISTRO" \
    -e "MOCK_MODE=${MOCK_MODE:-false}" \
    $NETWORK_ARGS \
    --privileged \
    "$IMAGE_NAME" \
    bash -c "
        echo '============================================='
        echo 'OpenClaw-ROS Bridge Container'
        echo 'ROS Distro: $ROS_DISTRO'
        echo '============================================='
        echo ''
        echo 'Available commands:'
        echo '  ros2 topic list          # List ROS topics'
        echo '  ros2 node list           # List ROS nodes'
        echo '  cd /app && ./scripts/build.sh    # Build project'
        echo '  cd /app && ./scripts/run_demo.sh --greenhouse --mock'
        echo ''
        source /opt/ros/$ROS_DISTRO/setup.bash
        cd /app
        bash
    "

success "Container session ended"
log "To reattach: $0"
log "To stop: $0 --stop"
log "To remove: $0 --rm"
