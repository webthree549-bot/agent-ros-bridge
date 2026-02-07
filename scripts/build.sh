#!/bin/bash
set -euo pipefail

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
error() { echo -e "${RED}[ERROR] $1${NC}"; exit 1; }

log "============================================="
log "OpenClaw-ROS Bridge Build Script - v1.0.0"
log "============================================="

# Install dependencies
log "Installing Python dependencies..."
pip3 install -r "$PROJECT_ROOT/requirements.txt"

# Build ROS
if [[ -f "$PROJECT_ROOT/catkin_ws/src/CMakeLists.txt" ]] || [[ -d "/opt/ros/noetic" ]]; then
    log "Building ROS1..."
    mkdir -p catkin_ws/src
    cd catkin_ws
    catkin_make
else
    log "Building ROS2..."
    colcon build --symlink-install
fi

success "Build complete!"