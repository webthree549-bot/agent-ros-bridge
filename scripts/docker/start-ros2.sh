#!/bin/bash
# Start ROS2 Docker container for Agent ROS Bridge

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

CONTAINER_NAME="ros2_humble"
IMAGE_NAME="agent-ros-bridge:ros2-humble"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if container already exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo -e "${GREEN}✅ Container '${CONTAINER_NAME}' is already running${NC}"
        exit 0
    else
        echo -e "${YELLOW}Starting existing container '${CONTAINER_NAME}'...${NC}"
        docker start "${CONTAINER_NAME}"
        echo -e "${GREEN}✅ Container started${NC}"
        exit 0
    fi
fi

# Check if image exists
if ! docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE_NAME}$"; then
    echo -e "${YELLOW}Image '${IMAGE_NAME}' not found. Building...${NC}"
    "${SCRIPT_DIR}/build-ros2-image.sh"
fi

# Create and start container
echo -e "${GREEN}Creating ROS2 container...${NC}"
docker run -d \
    --name "${CONTAINER_NAME}" \
    --hostname "${CONTAINER_NAME}" \
    -v "${PROJECT_ROOT}:/workspace:ro" \
    -w /workspace \
    "${IMAGE_NAME}" \
    bash -c "while true; do sleep 1; done"

echo ""
echo -e "${GREEN}✅ Container '${CONTAINER_NAME}' is running${NC}"
echo ""
echo "Useful commands:"
echo "  docker exec -it ${CONTAINER_NAME} bash    # Enter container"
echo "  docker logs ${CONTAINER_NAME}             # View logs"
echo "  docker stop ${CONTAINER_NAME}             # Stop container"
