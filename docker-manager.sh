#!/bin/bash
# docker-manager.sh - Manage ROS2 Docker container for development and CI
#
# Usage:
#   ./docker-manager.sh start     # Start container with auto-setup
#   ./docker-manager.sh stop      # Stop container
#   ./docker-manager.sh status    # Check container status
#   ./docker-manager.sh setup     # Initial setup (pull image, create container)
#   ./docker-manager.sh test      # Run tests inside container
#   ./docker-manager.sh shell     # Open shell in container
#   ./docker-manager.sh ci        # CI mode (headless, no GUI)

set -e

# Configuration
CONTAINER_NAME="ros2_humble"
IMAGE_NAME="osrf/ros:humble-desktop"
WORKSPACE_DIR="$(pwd)"
DOCKER_NETWORK="ros2-network"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if Docker is available
check_docker() {
    if ! command -v docker &> /dev/null; then
        log_error "Docker not found. Please install Docker."
        exit 1
    fi
    
    if ! docker info &> /dev/null; then
        log_error "Docker daemon not running. Please start Docker."
        exit 1
    fi
    
    log_success "Docker is available"
}

# Setup: Pull image and create container
setup() {
    log_info "Setting up ROS2 Docker environment..."
    
    # Pull image if not exists
    if ! docker image inspect "$IMAGE_NAME" &> /dev/null; then
        log_info "Pulling ROS2 Humble image (this may take a while)..."
        docker pull "$IMAGE_NAME"
    fi
    
    # Create network if not exists
    if ! docker network inspect "$DOCKER_NETWORK" &> /dev/null; then
        log_info "Creating Docker network: $DOCKER_NETWORK"
        docker network create "$DOCKER_NETWORK"
    fi
    
    log_success "Setup complete"
}

# Start container
start() {
    check_docker
    
    # Check if already running
    if docker ps | grep -q "$CONTAINER_NAME"; then
        log_success "Container $CONTAINER_NAME is already running"
        show_status
        return 0
    fi
    
    # Check if container exists but stopped
    if docker ps -a | grep -q "$CONTAINER_NAME"; then
        log_info "Starting existing container..."
        docker start "$CONTAINER_NAME"
    else
        log_info "Creating new ROS2 container..."
        
        # Detect platform for display/GPU settings
        PLATFORM="${PLATFORM:-$(uname -s)}"
        DISPLAY_ARGS=""
        GPU_ARGS=""
        
        if [ "$PLATFORM" = "Darwin" ]; then
            # macOS - no direct X11/GPU support
            log_warn "macOS detected - Gazebo GUI will not be available"
        elif [ "$PLATFORM" = "Linux" ]; then
            # Linux - enable X11 forwarding if available
            if [ -n "$DISPLAY" ]; then
                DISPLAY_ARGS="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix"
                log_info "X11 forwarding enabled"
            fi
            
            # Enable GPU if available
            if command -v nvidia-smi &> /dev/null; then
                GPU_ARGS="--gpus all"
                log_info "NVIDIA GPU support enabled"
            fi
        fi
        
        # Run container with all necessary mounts
        docker run -d \
            --name "$CONTAINER_NAME" \
            --hostname ros2-humble \
            --network "$DOCKER_NETWORK" \
            --privileged \
            --restart unless-stopped \
            -p 8765:8765 \
            -p 11311:11311 \
            -p 9090:9090 \
            -v "$WORKSPACE_DIR:/workspace:rw" \
            -v /dev/shm:/dev/shm \
            $DISPLAY_ARGS \
            $GPU_ARGS \
            "$IMAGE_NAME" \
            bash -c "
                echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
                echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
                tail -f /dev/null
            "
    fi
    
    # Wait for container to be ready
    log_info "Waiting for container to be ready..."
    for i in {1..30}; do
        if docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && ros2 --version" &> /dev/null; then
            break
        fi
        sleep 1
    done
    
    # Install additional packages if needed
    install_packages
    
    log_success "Container $CONTAINER_NAME is running"
    show_status
}

# Install additional ROS2 packages
install_packages() {
    log_info "Checking for additional packages..."
    
    # Check if Nav2 is installed
    if ! docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list | grep -q nav2_bringup" 2>/dev/null; then
        log_info "Installing Nav2 packages..."
        docker exec "$CONTAINER_NAME" bash -c "
            apt-get update && \
            apt-get install -y \
                ros-humble-nav2-bringup \
                ros-humble-nav2-simple-commander \
                ros-humble-turtlebot3-gazebo \
                ros-humble-turtlebot3-navigation2 \
                python3-pip
        " || log_warn "Failed to install some packages (may require manual installation)"
    fi
}

# Stop container
stop() {
    if docker ps | grep -q "$CONTAINER_NAME"; then
        log_info "Stopping container $CONTAINER_NAME..."
        docker stop "$CONTAINER_NAME"
        log_success "Container stopped"
    else
        log_warn "Container $CONTAINER_NAME is not running"
    fi
}

# Show container status
show_status() {
    if docker ps | grep -q "$CONTAINER_NAME"; then
        echo ""
        echo "=========================================="
        echo "Container Status: $CONTAINER_NAME"
        echo "=========================================="
        docker ps --filter "name=$CONTAINER_NAME" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
        echo ""
        
        # Show ROS2 topics if available
        if docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && ros2 topic list" &> /dev/null; then
            echo "Active ROS2 Topics:"
            docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && ros2 topic list" 2>/dev/null | head -10
        fi
        echo ""
    else
        log_warn "Container $CONTAINER_NAME is not running"
    fi
}

# Open shell in container
shell() {
    if docker ps | grep -q "$CONTAINER_NAME"; then
        log_info "Opening shell in container..."
        docker exec -it "$CONTAINER_NAME" bash
    else
        log_error "Container $CONTAINER_NAME is not running. Start it first with: $0 start"
        exit 1
    fi
}

# Run tests inside container
run_tests() {
    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        log_info "Starting container first..."
        start
    fi
    
    log_info "Running tests inside container..."
    docker exec -it "$CONTAINER_NAME" bash -c "
        cd /workspace && \
        source /opt/ros/humble/setup.bash && \
        pip3 install -e '.[dev]' && \
        python3 -m pytest tests/ -v --tb=short
    "
}

# CI mode - headless, no GUI, optimized for GitHub Actions
ci_mode() {
    log_info "Running in CI mode..."
    
    # Use lighter base image for CI
    CI_IMAGE="osrf/ros:humble-ros-base"
    
    # Pull image
    if ! docker image inspect "$CI_IMAGE" &> /dev/null; then
        log_info "Pulling CI-optimized ROS2 image..."
        docker pull "$CI_IMAGE"
    fi
    
    # Remove existing container if exists
    if docker ps -a | grep -q "$CONTAINER_NAME"; then
        log_info "Removing existing container..."
        docker rm -f "$CONTAINER_NAME"
    fi
    
    # Run container in CI mode (no GUI, minimal packages)
    log_info "Starting CI container..."
    docker run -d \
        --name "$CONTAINER_NAME" \
        --network "$DOCKER_NETWORK" \
        --privileged \
        -p 8765:8765 \
        -p 11311:11311 \
        -v "$(pwd):/workspace:rw" \
        "$CI_IMAGE" \
        tail -f /dev/null
    
    # Install minimal packages for testing
    log_info "Installing minimal test dependencies..."
    docker exec "$CONTAINER_NAME" bash -c "
        apt-get update && \
        apt-get install -y \
            python3-pip \
            python3-pytest \
            python3-coverage \
            ros-humble-geometry-msgs \
            ros-humble-sensor-msgs \
            ros-humble-nav-msgs
    "
    
    log_success "CI container ready"
    show_status
}

# Main command handler
case "${1:-}" in
    start)
        start
        ;;
    stop)
        stop
        ;;
    restart)
        stop
        sleep 2
        start
        ;;
    status)
        show_status
        ;;
    setup)
        setup
        ;;
    test|tests)
        run_tests
        ;;
    shell)
        shell
        ;;
    ci)
        ci_mode
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status|setup|test|shell|ci}"
        echo ""
        echo "Commands:"
        echo "  start   - Start the ROS2 container"
        echo "  stop    - Stop the ROS2 container"
        echo "  restart - Restart the container"
        echo "  status  - Show container status"
        echo "  setup   - Initial setup (pull images)"
        echo "  test    - Run tests inside container"
        echo "  shell   - Open shell in container"
        echo "  ci      - CI mode (headless, minimal)"
        exit 1
        ;;
esac
