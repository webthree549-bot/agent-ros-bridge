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

log() { echo -e "${BLUE}[BUILD] $1${NC}"; }
success() { echo -e "${GREEN}[SUCCESS] $1${NC}"; }
warn() { echo -e "${YELLOW}[WARN] $1${NC}"; }
error() { echo -e "${RED}[ERROR] $1${NC}"; exit 1; }

log "============================================="
log "OpenClaw-ROS Bridge Build Script - v1.0.0"
log "============================================="

# Detect environment
detect_ros_environment() {
    # Check if we're inside a Docker container
    if [[ -f /.dockerenv ]]; then
        echo "docker"
        return
    fi
    
    # Check for local ROS installation
    if [[ -d "/opt/ros/noetic" ]]; then
        echo "local_ros1"
        return
    fi
    
    if [[ -d "/opt/ros/humble" ]]; then
        echo "local_ros2_humble"
        return
    fi
    
    if [[ -d "/opt/ros/jazzy" ]]; then
        echo "local_ros2_jazzy"
        return
    fi
    
    # Check if ROS Docker container is running
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -q "ros"; then
        echo "docker_running"
        return
    fi
    
    # Check mock mode
    if [[ "${MOCK_MODE:-false}" == "true" ]]; then
        echo "mock"
        return
    fi
    
    echo "none"
}

ROS_ENV=$(detect_ros_environment)

# Install Python dependencies (always needed)
log "Installing Python dependencies..."

# Find available pip command
PIP_CMD=""
if command -v pip3 &> /dev/null; then
    PIP_CMD="pip3"
elif command -v pip &> /dev/null; then
    PIP_CMD="pip"
else
    warn "pip not found, attempting to install..."
    apt-get update -qq && apt-get install -y -qq python3-pip || warn "Failed to install pip"
    PIP_CMD="pip3"
fi

if [[ -n "$PIP_CMD" ]]; then
    # Use --break-system-packages in Docker containers (PEP 668)
    if [[ -f /.dockerenv ]]; then
        $PIP_CMD install --break-system-packages -r "$PROJECT_ROOT/requirements.txt" || warn "Some pip packages may have failed, continuing..."
    else
        $PIP_CMD install -r "$PROJECT_ROOT/requirements.txt" || warn "Some pip packages may have failed, continuing..."
    fi
else
    warn "pip not available, skipping Python package installation"
fi

# Build based on detected environment
case "$ROS_ENV" in
    "docker")
        log "Detected Docker container environment"
        if [[ -f "/opt/ros/noetic/setup.bash" ]]; then
            log "Building for ROS1 Noetic (Docker)..."
            source /opt/ros/noetic/setup.bash
            mkdir -p catkin_ws/src
            cd catkin_ws
            catkin_make
        elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
            log "Building for ROS2 Humble (Docker)..."
            source /opt/ros/humble/setup.bash
            colcon build --symlink-install
        elif [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
            log "Building for ROS2 Jazzy (Docker)..."
            source /opt/ros/jazzy/setup.bash
            colcon build --symlink-install
            # Fix libexec directory for ROS2 launch
            if [[ -d "$PROJECT_ROOT/install/openclaw_ros_bridge/bin" ]]; then
                mkdir -p "$PROJECT_ROOT/install/openclaw_ros_bridge/lib/openclaw_ros_bridge"
                ln -sf "$PROJECT_ROOT/install/openclaw_ros_bridge/bin/"* "$PROJECT_ROOT/install/openclaw_ros_bridge/lib/openclaw_ros_bridge/"
                log "Fixed libexec symlinks for ROS2"
            fi
        else
            error "ROS not found in Docker container"
        fi
        ;;
        
    "local_ros1")
        log "Detected local ROS1 Noetic installation"
        source /opt/ros/noetic/setup.bash
        mkdir -p catkin_ws/src
        cd catkin_ws
        catkin_make
        ;;
        
    "local_ros2_humble")
        log "Detected local ROS2 Humble installation"
        source /opt/ros/humble/setup.bash
        colcon build --symlink-install
        ;;
        
    "local_ros2_jazzy")
        log "Detected local ROS2 Jazzy installation"
        source /opt/ros/jazzy/setup.bash
        colcon build --symlink-install
        # Fix libexec directory for ROS2 launch
        if [[ -d "$PROJECT_ROOT/install/openclaw_ros_bridge/bin" ]]; then
            mkdir -p "$PROJECT_ROOT/install/openclaw_ros_bridge/lib/openclaw_ros_bridge"
            ln -sf "$PROJECT_ROOT/install/openclaw_ros_bridge/bin/"* "$PROJECT_ROOT/install/openclaw_ros_bridge/lib/openclaw_ros_bridge/"
            log "Fixed libexec symlinks for ROS2"
        fi
        ;;
        
    "docker_running")
        log "Detected running ROS Docker container"
        # Find the ROS container
        ROS_CONTAINER=$(docker ps --format '{{.Names}}' | grep "ros" | head -1)
        log "Using container: $ROS_CONTAINER"
        
        # Build inside the container
        docker exec "$ROS_CONTAINER" bash -c "
            cd /app &&
            if [[ -f /opt/ros/noetic/setup.bash ]]; then
                source /opt/ros/noetic/setup.bash &&
                mkdir -p catkin_ws/src &&
                cd catkin_ws &&
                catkin_make
            elif [[ -f /opt/ros/humble/setup.bash ]]; then
                source /opt/ros/humble/setup.bash &&
                colcon build --symlink-install
            elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
                source /opt/ros/jazzy/setup.bash &&
                colcon build --symlink-install &&
                mkdir -p /app/install/openclaw_ros_bridge/lib/openclaw_ros_bridge &&
                ln -sf /app/install/openclaw_ros_bridge/bin/* /app/install/openclaw_ros_bridge/lib/openclaw_ros_bridge/
            fi
        " || error "Build in Docker container failed"
        ;;
        
    "mock")
        log "Mock mode enabled - skipping ROS build"
        log "Python packages installed successfully"
        ;;
        
    "none")
        warn "No ROS installation detected"
        warn "Options:"
        warn "  1. Install ROS locally (apt install ros-jazzy-desktop)"
        warn "  2. Start ROS Docker container (./scripts/docker_start.sh)"
        warn "  3. Enable mock mode (export MOCK_MODE=true)"
        
        # Still try to install Python packages for development
        log "Python packages installed for development mode"
        ;;
esac

# Create marker file for run scripts
if [[ "$ROS_ENV" != "none" ]]; then
    touch "$PROJECT_ROOT/.build_complete"
fi

success "Build complete!"

# Print next steps
log ""
log "Next steps:"
case "$ROS_ENV" in
    "docker"|"local_ros1")
        log "  source catkin_ws/devel/setup.bash"
        log "  ./scripts/run_demo.sh --greenhouse"
        ;;
    "local_ros2_humble"|"local_ros2_jazzy"|"docker_running")
        log "  source install/setup.bash"
        log "  ./scripts/run_demo.sh --greenhouse"
        ;;
    "mock")
        log "  export MOCK_MODE=true"
        log "  ./scripts/run_demo.sh --greenhouse"
        ;;
    "none")
        log "  # Option 1: Use Docker"
        log "  docker-compose -f docker/docker-compose.yml up -d"
        log "  # Option 2: Enable mock mode"
        log "  export MOCK_MODE=true"
        log "  ./scripts/run_demo.sh --greenhouse"
        ;;
esac
