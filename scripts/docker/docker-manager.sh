#!/bin/bash
# Unified Docker Manager for Agent ROS Bridge
# Replaces: docker-manager.sh (root), scripts/docker-manager.sh, scripts/docker/start-ros2.sh
#
# Usage: ./scripts/docker/docker-manager.sh <command>

set -e

# Use absolute path to avoid $(pwd) confusion
PROJECT_ROOT="/Users/webthree/.openclaw/workspace/agent-ros-bridge"

# Configuration
CONTAINER_NAME="${CONTAINER_NAME:-ros2_jazzy}"
# Using pre-built image with Nav2 and Gazebo pre-installed
# To build from scratch instead, use: agent-ros-bridge:ros2-jazzy
IMAGE_NAME="${IMAGE_NAME:-agent-ros-bridge:jazzy-with-nav2}"
DOCKER_NETWORK="${DOCKER_NETWORK:-ros2-network}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

check_docker() {
    if ! command -v docker &> /dev/null; then
        log_error "Docker not found. Please install Docker."
        exit 1
    fi
    if ! docker info &> /dev/null; then
        log_error "Docker daemon not running. Please start Docker."
        exit 1
    fi
}

cmd_build() {
    log_info "Building Docker image: ${IMAGE_NAME}"
    "${SCRIPT_DIR}/build-ros2-image.sh"
}

cmd_start() {
    check_docker
    
    # Check if already running
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        log_success "Container '${CONTAINER_NAME}' is already running"
        cmd_status
        return 0
    fi
    
    # Check if container exists but stopped
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        log_info "Starting existing container '${CONTAINER_NAME}'..."
        docker start "${CONTAINER_NAME}"
        log_success "Container started"
        return 0
    fi
    
    # Build image if needed
    if ! docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE_NAME}$"; then
        log_warn "Image '${IMAGE_NAME}' not found."
        read -p "Build it now? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            cmd_build
        else
            log_error "Cannot start without image. Run: $0 build"
            exit 1
        fi
    fi
    
    # Detect platform for display/GPU
    PLATFORM="${PLATFORM:-$(uname -s)}"
    DISPLAY_ARGS=""
    GPU_ARGS=""
    
    if [ "$PLATFORM" = "Linux" ] && [ -n "$DISPLAY" ]; then
        DISPLAY_ARGS="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix"
        log_info "X11 forwarding enabled"
    fi
    
    # Run container
    log_info "Creating ROS2 Jazzy container..."
    docker run -d \
        --name "${CONTAINER_NAME}" \
        --hostname "${CONTAINER_NAME}" \
        --network "${DOCKER_NETWORK}" 2>/dev/null || true \
        --privileged \
        --restart unless-stopped \
        -p 8765:8765 \
        -p 8080:8080 \
        -p 11311:11311 \
        -p 9090:9090 \
        -v "${PROJECT_ROOT}:/workspace:rw" \
        -v /dev/shm:/dev/shm \
        $DISPLAY_ARGS \
        $GPU_ARGS \
        "${IMAGE_NAME}" \
        bash -c "
            echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
            echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
            echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
            tail -f /dev/null
        "
    
    log_success "Container '${CONTAINER_NAME}' is running"
    cmd_status
}

cmd_stop() {
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        log_info "Stopping container '${CONTAINER_NAME}'..."
        docker stop "${CONTAINER_NAME}"
        log_success "Container stopped"
    else
        log_warn "Container '${CONTAINER_NAME}' is not running"
    fi
}

cmd_restart() {
    cmd_stop
    sleep 1
    cmd_start
}

cmd_status() {
    echo "=========================================="
    echo "Docker Container Status"
    echo "=========================================="
    
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo -e "${GREEN}Container: ${CONTAINER_NAME} - RUNNING${NC}"
        docker ps --filter "name=${CONTAINER_NAME}" --format "  Status: {{.Status}}"
        docker ps --filter "name=${CONTAINER_NAME}" --format "  Ports: {{.Ports}}"
        
        # Check ROS2
        if docker exec "${CONTAINER_NAME}" bash -c "source /opt/ros/jazzy/setup.bash && ros2 --version" &> /dev/null; then
            echo -e "  ${GREEN}ROS2: Available${NC}"
        else
            echo -e "  ${YELLOW}ROS2: Initializing...${NC}"
        fi
        
        # Check Nav2
        if docker exec "${CONTAINER_NAME}" bash -c "source /opt/ros/jazzy/setup.bash && ros2 pkg list | grep -q nav2" 2>/dev/null; then
            echo -e "  ${GREEN}Nav2: Installed${NC}"
        else
            echo -e "  ${YELLOW}Nav2: Not installed${NC}"
        fi
    else
        echo -e "${RED}Container: ${CONTAINER_NAME} - NOT RUNNING${NC}"
    fi
    
    echo ""
    echo "Image: ${IMAGE_NAME}"
    if docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE_NAME}$"; then
        echo -e "  ${GREEN}Status: Available${NC}"
    else
        echo -e "  ${YELLOW}Status: Not built${NC}"
    fi
    echo "=========================================="
}

cmd_shell() {
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        log_info "Opening shell in container..."
        docker exec -it "${CONTAINER_NAME}" bash
    else
        log_error "Container not running. Start it with: $0 start"
        exit 1
    fi
}

cmd_logs() {
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        docker logs "${CONTAINER_NAME}" "$@"
    else
        log_error "Container does not exist"
        exit 1
    fi
}

cmd_test() {
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        log_info "Starting container first..."
        cmd_start
    fi
    
    log_info "Running tests inside container..."
    docker exec -it "${CONTAINER_NAME}" bash -c "
        cd /workspace && \
        source /opt/ros/jazzy/setup.bash && \
        pip3 install -e '.[dev]' 2>/dev/null && \
        python3 -m pytest tests/ -v --tb=short -x
    "
}

cmd_clean() {
    log_warn "This will remove the container and image. Continue? (y/n)"
    read -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        log_info "Stopping and removing container..."
        docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
        
        log_info "Removing image..."
        docker rmi "${IMAGE_NAME}" 2>/dev/null || true
        
        log_success "Cleanup complete"
    else
        log_info "Cancelled"
    fi
}

show_help() {
    cat << EOF
Agent ROS Bridge - Docker Manager

Usage: $0 <command> [options]

Commands:
    build       Build the Docker image
    start       Start the container (creates if needed)
    stop        Stop the running container
    restart     Restart the container
    status      Show container and image status
    shell       Open interactive shell in container
    logs        View container logs (pass -f to follow)
    test        Run tests inside container
    clean       Remove container and image

Environment Variables:
    CONTAINER_NAME    Default: ros2_jazzy
    IMAGE_NAME        Default: agent-ros-bridge:ros2-jazzy
    DOCKER_NETWORK    Default: ros2-network

Examples:
    $0 start                    # Start container
    $0 shell                    # Enter container
    $0 test                     # Run tests
    $0 logs -f                  # Follow logs

EOF
}

# Main command handler
case "${1:-}" in
    build)
        cmd_build
        ;;
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
    shell)
        cmd_shell
        ;;
    logs)
        shift
        cmd_logs "$@"
        ;;
    test)
        cmd_test
        ;;
    clean)
        cmd_clean
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        show_help
        exit 1
        ;;
esac
