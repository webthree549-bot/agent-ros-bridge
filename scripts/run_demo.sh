#!/bin/bash
set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PROJECT_ROOT"

log() { echo -e "${BLUE}[RUN-DEMO] $1${NC}"; }
success() { echo -e "${GREEN}[SUCCESS] $1${NC}"; }

DEMO_TYPE="greenhouse"
MOCK_MODE="false"

# Parse args
while [[ $# -gt 0 ]]; do
    case "$1" in
        --greenhouse) DEMO_TYPE="greenhouse"; shift ;;
        --arm) DEMO_TYPE="arm_manipulation"; shift ;;
        --mock) MOCK_MODE="true"; export MOCK_MODE="true"; shift ;;
        *) shift ;;
    esac
done

log "Starting $DEMO_TYPE Demo (Mock Mode: $MOCK_MODE)"

# Check ROS
if [[ -f "$PROJECT_ROOT/install/setup.bash" ]]; then
    source "$PROJECT_ROOT/install/setup.bash"
    ros2 launch openclaw_ros_bridge ${DEMO_TYPE}_demo.launch.py mock_mode:="$MOCK_MODE"
elif [[ -f "$PROJECT_ROOT/catkin_ws/devel/setup.bash" ]]; then
    source "$PROJECT_ROOT/catkin_ws/devel/setup.bash"
    roslaunch openclaw_ros_bridge ${DEMO_TYPE}_demo.launch mock_mode:="$MOCK_MODE"
else
    echo "No ROS setup found. Run ./scripts/build.sh first."
    exit 1
fi