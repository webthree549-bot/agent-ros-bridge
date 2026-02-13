#!/bin/bash
set -eo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PROJECT_ROOT"

log() { echo -e "${BLUE}[RUN-DEMO] $1${NC}"; }
success() { echo -e "${GREEN}[SUCCESS] $1${NC}"; }
warn() { echo -e "${YELLOW}[WARN] $1${NC}"; }
error() { echo -e "${RED}[ERROR] $1${NC}"; exit 1; }

DEMO_TYPE=""
MOCK_MODE="${MOCK_MODE:-false}"

# Parse args
while [[ $# -gt 0 ]]; do
    case "$1" in
        --greenhouse) DEMO_TYPE="greenhouse"; shift ;;
        --arm) DEMO_TYPE="arm_manipulation"; shift ;;
        --mock) MOCK_MODE="true"; shift ;;
        --list) DEMO_TYPE="list"; shift ;;
        -h|--help) 
            echo "Usage: ./scripts/run_demo.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --greenhouse    Run greenhouse demo (agricultural robotics)"
            echo "  --arm           Run arm manipulation demo (industrial robotics)"
            echo "  --mock          Use mock mode (no ROS/hardware required)"
            echo "  --list          List available demos"
            echo "  -h, --help      Show this help"
            echo ""
            echo "Examples:"
            echo "  ./scripts/run_demo.sh --greenhouse --mock"
            echo "  ./scripts/run_demo.sh --list"
            exit 0
            ;;
        *) shift ;;
    esac
done

export MOCK_MODE="$MOCK_MODE"

log "============================================="
log "OpenClaw-ROS Bridge Demo Launcher"
log "============================================="

# List available demos
if [[ "$DEMO_TYPE" == "list" ]]; then
    log "Available demos:"
    log ""
    log "  --greenhouse    Agricultural robotics (greenhouse control)"
    log "                  Location: demo/greenhouse/"
    log ""
    if [[ -d "$PROJECT_ROOT/demo/arm_manipulation" ]]; then
        log "  --arm           Industrial arm manipulation"
        log "                  Location: demo/arm_manipulation/"
        log ""
    else
        log "  --arm           Industrial arm manipulation (not yet implemented)"
        log ""
    fi
    log "Use --mock flag to run without hardware/ROS"
    exit 0
fi

# Validate demo type
if [[ -z "$DEMO_TYPE" ]]; then
    error "No demo specified. Use --list to see available demos."
fi

# Check if demo exists
DEMO_DIR="$PROJECT_ROOT/demo/$DEMO_TYPE"
if [[ ! -d "$DEMO_DIR" ]]; then
    error "Demo not found: $DEMO_TYPE"
    log "Available demos:"
    ls -1 "$PROJECT_ROOT/demo/" 2>/dev/null | while read d; do
        log "  --$d"
    done
    exit 1
fi

# Run the demo
log "Demo: $DEMO_TYPE"
log "Mock Mode: $MOCK_MODE"
log ""

if [[ -f "$DEMO_DIR/scripts/run_demo.sh" ]]; then
    log "Delegating to demo script..."
    exec "$DEMO_DIR/scripts/run_demo.sh" "$@"
else
    error "Demo script not found: $DEMO_DIR/scripts/run_demo.sh"
fi
