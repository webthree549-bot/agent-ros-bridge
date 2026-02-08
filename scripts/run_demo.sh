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

DEMO_TYPE="greenhouse"
MOCK_MODE="${MOCK_MODE:-false}"

# Parse args
while [[ $# -gt 0 ]]; do
    case "$1" in
        --greenhouse) DEMO_TYPE="greenhouse"; shift ;;
        --arm) DEMO_TYPE="arm_manipulation"; shift ;;
        --mock) MOCK_MODE="true"; shift ;;
        *) shift ;;
    esac
done

export MOCK_MODE="$MOCK_MODE"

log "============================================="
log "OpenClaw-ROS Bridge Demo - v1.0.0"
log "============================================="
log "Demo: $DEMO_TYPE"
log "Mock Mode: $MOCK_MODE"
log ""

# Detect environment
detect_ros_environment() {
    # Check if we're inside a Docker container
    if [[ -f /.dockerenv ]]; then
        echo "docker"
        return
    fi
    
    # Check for local ROS2 installation
    if [[ -d "/opt/ros/humble" ]] || [[ -d "/opt/ros/jazzy" ]]; then
        echo "local_ros2"
        return
    fi
    
    # Check for local ROS1 installation
    if [[ -d "/opt/ros/noetic" ]]; then
        echo "local_ros1"
        return
    fi
    
    # Check if ROS Docker container is running
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -q "ros"; then
        echo "docker_running"
        return
    fi
    
    # Check mock mode
    if [[ "$MOCK_MODE" == "true" ]]; then
        echo "mock"
        return
    fi
    
    echo "none"
}

ROS_ENV=$(detect_ros_environment)

# Run demo based on environment
run_demo() {
    local demo_name="$1"
    
    case "$ROS_ENV" in
        "docker")
            log "Running in Docker container..."
            if [[ -f "/opt/ros/jazzy/setup.bash" ]] || [[ -f "/opt/ros/humble/setup.bash" ]]; then
                # ROS2
                source /opt/ros/*/setup.bash
                if [[ -f "/app/install/setup.bash" ]]; then
                    source /app/install/setup.bash
                fi
                ros2 launch openclaw_ros_bridge "${demo_name}_demo.launch.py" mock_mode:="$MOCK_MODE"
            elif [[ -f "/opt/ros/noetic/setup.bash" ]]; then
                # ROS1
                source /opt/ros/noetic/setup.bash
                roslaunch openclaw_ros_bridge "${demo_name}_demo.launch" mock_mode:="$MOCK_MODE"
            fi
            ;;
            
        "local_ros2")
            log "Running with local ROS2..."
            if [[ -f "$PROJECT_ROOT/install/setup.bash" ]]; then
                source "$PROJECT_ROOT/install/setup.bash"
            elif [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
                source /opt/ros/jazzy/setup.bash
            elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
                source /opt/ros/humble/setup.bash
            fi
            ros2 launch openclaw_ros_bridge "${demo_name}_demo.launch.py" mock_mode:="$MOCK_MODE"
            ;;
            
        "local_ros1")
            log "Running with local ROS1..."
            if [[ -f "$PROJECT_ROOT/catkin_ws/devel/setup.bash" ]]; then
                source "$PROJECT_ROOT/catkin_ws/devel/setup.bash"
            elif [[ -f "/opt/ros/noetic/setup.bash" ]]; then
                source /opt/ros/noetic/setup.bash
            fi
            roslaunch openclaw_ros_bridge "${demo_name}_demo.launch" mock_mode:="$MOCK_MODE"
            ;;
            
        "docker_running")
            log "Running in Docker container..."
            ROS_CONTAINER=$(docker ps --format '{{.Names}}' | grep "ros" | head -1)
            log "Using container: $ROS_CONTAINER"
            
            # Check if container has the bridge code
            if ! docker exec "$ROS_CONTAINER" test -d /app; then
                warn "Bridge code not found in container"
                warn "Mounting project directory..."
                docker exec -v "$PROJECT_ROOT:/app" "$ROS_CONTAINER" bash -c "
                    cd /app &&
                    export MOCK_MODE=$MOCK_MODE &&
                    if [[ -f /opt/ros/jazzy/setup.bash ]]; then
                        source /opt/ros/jazzy/setup.bash &&
                        ros2 launch openclaw_ros_bridge ${demo_name}_demo.launch.py mock_mode:=$MOCK_MODE
                    elif [[ -f /opt/ros/humble/setup.bash ]]; then
                        source /opt/ros/humble/setup.bash &&
                        ros2 launch openclaw_ros_bridge ${demo_name}_demo.launch.py mock_mode:=$MOCK_MODE
                    elif [[ -f /opt/ros/noetic/setup.bash ]]; then
                        source /opt/ros/noetic/setup.bash &&
                        roslaunch openclaw_ros_bridge ${demo_name}_demo.launch mock_mode:=$MOCK_MODE
                    fi
                "
            else
                docker exec "$ROS_CONTAINER" bash -c "
                    cd /app &&
                    export MOCK_MODE=$MOCK_MODE &&
                    if [[ -f /opt/ros/jazzy/setup.bash ]]; then
                        source /opt/ros/jazzy/setup.bash &&
                        ros2 launch openclaw_ros_bridge ${demo_name}_demo.launch.py mock_mode:=$MOCK_MODE
                    elif [[ -f /opt/ros/humble/setup.bash ]]; then
                        source /opt/ros/humble/setup.bash &&
                        ros2 launch openclaw_ros_bridge ${demo_name}_demo.launch.py mock_mode:=$MOCK_MODE
                    elif [[ -f /opt/ros/noetic/setup.bash ]]; then
                        source /opt/ros/noetic/setup.bash &&
                        roslaunch openclaw_ros_bridge ${demo_name}_demo.launch mock_mode:=$MOCK_MODE
                    fi
                "
            fi
            ;;
            
        "mock")
            log "Running in mock mode (no ROS required)..."
            # Run Python directly without ROS
            export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"
            python3 -c "
import sys
sys.path.insert(0, '$PROJECT_ROOT')
from demo.${demo_name}.${demo_name}_plugin import main
main()
" || error "Mock mode requires Python dependencies. Run: pip3 install -r requirements.txt"
            ;;
            
        "none")
            error "No ROS installation detected.\nOptions:\n  1. Start Docker container: docker run -it --rm ros:jazzy-ros-base\n  2. Use mock mode: ./scripts/run_demo.sh --$demo_name --mock\n  3. Install ROS locally"
            ;;
    esac
}

# Validate demo type
if [[ ! -f "launch/${DEMO_TYPE}_demo.launch.py" ]] && [[ ! -f "launch/${DEMO_TYPE}_demo.launch" ]]; then
    error "Unknown demo: $DEMO_TYPE"
    log "Available demos:"
    log "  --greenhouse         Agricultural robotics"
    log "  --arm                Industrial arm manipulation"
    exit 1
fi

# Run the demo
log "Starting demo..."
run_demo "$DEMO_TYPE"

success "Demo completed!"
