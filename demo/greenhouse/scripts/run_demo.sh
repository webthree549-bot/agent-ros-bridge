#!/bin/bash
# demo/greenhouse/scripts/run_demo.sh - Greenhouse demo launcher
# Usage: ./run_demo.sh [--mock]

set -eo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
cd "$PROJECT_ROOT"

log() { echo -e "${BLUE}[GREENHOUSE-DEMO] $1${NC}"; }
success() { echo -e "${GREEN}[SUCCESS] $1${NC}"; }
warn() { echo -e "${YELLOW}[WARN] $1${NC}"; }
error() { echo -e "${RED}[ERROR] $1${NC}"; exit 1; }

MOCK_MODE="${MOCK_MODE:-false}"

# Parse args
while [[ $# -gt 0 ]]; do
    case "$1" in
        --mock) MOCK_MODE="true"; shift ;;
        *) shift ;;
    esac
done

export MOCK_MODE="$MOCK_MODE"

log "============================================="
log "Greenhouse Demo - Application Example"
log "============================================="
log "Mock Mode: $MOCK_MODE"
log ""
log "This is a DEMO showing how to build on top"
log "of the generic OpenClaw ROS Bridge."
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

# Run demo
run_demo() {
    case "$ROS_ENV" in
        "docker"|"local_ros2")
            log "Running greenhouse demo with ROS2..."
            if [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
                source /opt/ros/jazzy/setup.bash
            elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
                source /opt/ros/humble/setup.bash
            fi
            
            # Run the greenhouse-specific server
            export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"
            python3 "$PROJECT_ROOT/demo/greenhouse/greenhouse_server.py"
            ;;
            
        "local_ros1")
            log "Running greenhouse demo with ROS1..."
            source /opt/ros/noetic/setup.bash
            export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"
            python3 "$PROJECT_ROOT/demo/greenhouse/greenhouse_server.py"
            ;;
            
        "docker_running")
            log "Running in Docker container..."
            ROS_CONTAINER=$(docker ps --format '{{.Names}}' | grep "ros" | head -1)
            log "Using container: $ROS_CONTAINER"
            
            docker exec "$ROS_CONTAINER" bash -c "
                cd /app &&
                export MOCK_MODE=$MOCK_MODE &&
                source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null &&
                python3 demo/greenhouse/greenhouse_server.py
            "
            ;;
            
        "mock")
            log "Running greenhouse demo in mock mode..."
            export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"
            python3 "$PROJECT_ROOT/demo/greenhouse/greenhouse_server.py"
            ;;
            
        "none")
            error "No ROS installation detected.\nOptions:\n  1. Start Docker container\n  2. Use mock mode: ./run_demo.sh --mock\n  3. Install ROS locally"
            ;;
    esac
}

# Run the demo
run_demo

success "Demo completed!"
