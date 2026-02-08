#!/bin/bash
# start_openclaw_server.sh - Start TCP server for OpenClaw connection
#
# Usage: ./start_openclaw_server.sh [port]
# Default port: 9999
#

set -euo pipefail

BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log() { echo -e "${BLUE}[SERVER] $1${NC}"; }
success() { echo -e "${GREEN}[SUCCESS] $1${NC}"; }
warn() { echo -e "${YELLOW}[WARN] $1${NC}"; }

PORT="${1:-9999}"

log "Starting OpenClaw TCP Server on port $PORT..."
log "This allows OpenClaw on macOS to connect to ROS Bridge in Docker"
log ""

# Check if running in Docker
if [[ -f /.dockerenv ]]; then
    log "Running inside Docker container"
    log "OpenClaw on macOS should connect to: localhost:$PORT"
else
    log "Running on host"
    log "Make sure Docker container maps port $PORT:$PORT"
fi

log ""
log "Starting server..."
log "Press Ctrl+C to stop"
log ""

# Start the TCP server
openclaw_tcp_server
