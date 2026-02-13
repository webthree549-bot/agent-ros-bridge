#!/bin/bash
# start_openclaw_server.sh - Start the generic OpenClaw TCP Server
# 
# This starts the application-agnostic TCP server. To use with a specific
# application, either:
# 1. Load the application plugin after starting
# 2. Use the application's own server launcher (e.g., demo/greenhouse/greenhouse_server.py)

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PROJECT_ROOT"

log() { echo -e "${BLUE}[SERVER] $1${NC}"; }
success() { echo -e "${GREEN}[SUCCESS] $1${NC}"; }
warn() { echo -e "${YELLOW}[WARN] $1${NC}"; }
error() { echo -e "${RED}[ERROR] $1${NC}"; exit 1; }

MOCK_MODE="${MOCK_MODE:-false}"
PORT="${PORT:-9999}"

log "============================================="
log "OpenClaw TCP Server (Generic)"
log "============================================="
log "Port: $PORT"
log "Mock Mode: $MOCK_MODE"
log ""

# Detect environment
detect_ros_environment() {
    if [[ -f /.dockerenv ]]; then
        echo "docker"
        return
    fi
    
    if [[ -d "/opt/ros/humble" ]] || [[ -d "/opt/ros/jazzy" ]]; then
        echo "local_ros2"
        return
    fi
    
    if [[ -d "/opt/ros/noetic" ]]; then
        echo "local_ros1"
        return
    fi
    
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -q "ros"; then
        echo "docker_running"
        return
    fi
    
    if [[ "$MOCK_MODE" == "true" ]]; then
        echo "mock"
        return
    fi
    
    echo "none"
}

ROS_ENV=$(detect_ros_environment)

# Start server based on environment
start_server() {
    case "$ROS_ENV" in
        "docker")
            log "Running in Docker container..."
            if [[ -f "/opt/ros/jazzy/setup.bash" ]] || [[ -f "/opt/ros/humble/setup.bash" ]]; then
                source /opt/ros/*/setup.bash
            elif [[ -f "/opt/ros/noetic/setup.bash" ]]; then
                source /opt/ros/noetic/setup.bash
            fi
            
            export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"
            python3 "$PROJECT_ROOT/openclaw_ros_bridge/communication/openclaw_tcp_server.py"
            ;;
            
        "local_ros2")
            log "Running with local ROS2..."
            if [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
                source /opt/ros/jazzy/setup.bash
            elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
                source /opt/ros/humble/setup.bash
            fi
            
            export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"
            python3 "$PROJECT_ROOT/openclaw_ros_bridge/communication/openclaw_tcp_server.py"
            ;;
            
        "local_ros1")
            log "Running with local ROS1..."
            source /opt/ros/noetic/setup.bash
            
            export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"
            python3 "$PROJECT_ROOT/openclaw_ros_bridge/communication/openclaw_tcp_server.py"
            ;;
            
        "docker_running")
            ROS_CONTAINER=$(docker ps --format '{{.Names}}' | grep "ros" | head -1)
            log "Running in Docker container: $ROS_CONTAINER"
            
            docker exec -it "$ROS_CONTAINER" bash -c "
                cd /app &&
                export MOCK_MODE=$MOCK_MODE &&
                export PORT=$PORT &&
                source /opt/ros/jazzy/setup.bash 2>/dev/null || 
                source /opt/ros/humble/setup.bash 2>/dev/null ||
                source /opt/ros/noetic/setup.bash 2>/dev/null &&
                python3 openclaw_ros_bridge/communication/openclaw_tcp_server.py
            "
            ;;
            
        "mock")
            log "Running in mock mode (no ROS required)..."
            export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"
            python3 "$PROJECT_ROOT/openclaw_ros_bridge/communication/openclaw_tcp_server.py"
            ;;
            
        "none")
            error "No ROS installation detected.\nOptions:\n  1. Start Docker container: ./scripts/docker_start.sh\n  2. Use mock mode: MOCK_MODE=true $0\n  3. Install ROS locally"
            ;;
    esac
}

log "Environment: $ROS_ENV"
log "Starting generic TCP server..."
log ""
log "Note: This is the APPLICATION-AGNOSTIC server."
log "      Core commands only: ping, get_status, list_handlers"
log "      To use with an application (e.g., greenhouse), run:"
log "      ./demo/greenhouse/scripts/run_demo.sh"
log ""

start_server
