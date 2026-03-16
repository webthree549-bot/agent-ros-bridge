#!/bin/bash
# Docker manager for Agent ROS Bridge

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONTAINER_NAME="ros2_humble"
IMAGE_NAME="agent-ros-bridge:ros2-humble"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

show_help() {
    cat << EOF
Docker Manager for Agent ROS Bridge

Usage: $0 <command>

Commands:
    start       Start the ROS2 Docker container
    stop        Stop the ROS2 Docker container
    restart     Restart the ROS2 Docker container
    status      Check container status
    build       Build the ROS2 Docker image
    shell       Open a shell in the container
    logs        View container logs
    clean       Remove container and image

Examples:
    $0 start
    $0 shell
    $0 stop

EOF
}

cmd_start() {
    "${SCRIPT_DIR}/docker/start-ros2.sh"
}

cmd_stop() {
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo -e "${YELLOW}Stopping container '${CONTAINER_NAME}'...${NC}"
        docker stop "${CONTAINER_NAME}"
        echo -e "${GREEN}✅ Container stopped${NC}"
    else
        echo -e "${YELLOW}Container '${CONTAINER_NAME}' is not running${NC}"
    fi
}

cmd_restart() {
    cmd_stop
    sleep 1
    cmd_start
}

cmd_status() {
    echo -e "${BLUE}Container Status:${NC}"
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo -e "${GREEN}  ✅ ${CONTAINER_NAME} is running${NC}"
        docker ps --filter "name=${CONTAINER_NAME}" --format "  Status: {{.Status}}"
        
        # Check Nav2
        if docker exec "${CONTAINER_NAME}" bash -c "ros2 pkg list | grep -q nav2" 2>/dev/null; then
            echo -e "${GREEN}  ✅ Nav2 is installed${NC}"
        else
            echo -e "${YELLOW}  ⚠️  Nav2 is not installed${NC}"
        fi
    else
        echo -e "${RED}  ❌ ${CONTAINER_NAME} is not running${NC}"
    fi
    
    echo ""
    echo -e "${BLUE}Image Status:${NC}"
    if docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE_NAME}$"; then
        echo -e "${GREEN}  ✅ ${IMAGE_NAME} exists${NC}"
    else
        echo -e "${RED}  ❌ ${IMAGE_NAME} not found${NC}"
    fi
}

cmd_build() {
    "${SCRIPT_DIR}/docker/build-ros2-image.sh"
}

cmd_shell() {
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo -e "${YELLOW}Container not running. Starting...${NC}"
        cmd_start
    fi
    
    echo -e "${BLUE}Opening shell in ${CONTAINER_NAME}...${NC}"
    docker exec -it "${CONTAINER_NAME}" bash
}

cmd_logs() {
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        docker logs "${CONTAINER_NAME}" "$@"
    else
        echo -e "${RED}Container '${CONTAINER_NAME}' does not exist${NC}"
    fi
}

cmd_clean() {
    echo -e "${YELLOW}This will remove the container and image.${NC}"
    read -p "Are you sure? (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        cmd_stop 2>/dev/null || true
        docker rm "${CONTAINER_NAME}" 2>/dev/null || true
        docker rmi "${IMAGE_NAME}" 2>/dev/null || true
        echo -e "${GREEN}✅ Cleaned up${NC}"
    else
        echo "Cancelled"
    fi
}

# Main
case "${1:-}" in
    start)
        cmd_start
        ;;
    stop)
        cmd_stop
        ;;
    restart)
        cmd_restart
        ;;
    status)
        cmd_status
        ;;
    build)
        cmd_build
        ;;
    shell)
        cmd_shell
        ;;
    logs)
        shift
        cmd_logs "$@"
        ;;
    clean)
        cmd_clean
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        echo -e "${RED}Unknown command: ${1:-}${NC}"
        show_help
        exit 1
        ;;
esac
