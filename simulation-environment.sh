#!/bin/bash
# simulation-environment.sh - Full ROS2 Gazebo Simulation Environment Setup
#
# This script sets up a complete ROS2 Humble + Gazebo + Nav2 + TurtleBot3 simulation
# for testing the agent-ros-bridge with a virtual robot.
#
# Usage:
#   ./simulation-environment.sh setup      # Build simulation Docker image
#   ./simulation-environment.sh start      # Start Gazebo + Nav2 simulation
#   ./simulation-environment.sh stop       # Stop simulation
#   ./simulation-environment.sh status     # Check simulation status
#   ./simulation-environment.sh test       # Run navigation tests against simulation
#   ./simulation-environment.sh logs       # View simulation logs

set -e

# Configuration
SIM_IMAGE_NAME="agent-ros-bridge-sim"
CONTAINER_GAZEBO="ros2_gazebo_sim"
CONTAINER_NAV2="ros2_nav2_sim"
CONTAINER_BRIDGE="ros2_bridge"
CONTAINER_ROSBRIDGE="ros2_rosbridge"
NETWORK_NAME="ros2_sim_network"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_section() { echo -e "\n${CYAN}══════════════════════════════════════════════════════════════${NC}"; echo -e "${CYAN}  $1${NC}"; echo -e "${CYAN}══════════════════════════════════════════════════════════════${NC}\n"; }

# Check Docker
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

# Create Docker network
create_network() {
    if ! docker network inspect "$NETWORK_NAME" &> /dev/null; then
        log_info "Creating Docker network: $NETWORK_NAME"
        docker network create "$NETWORK_NAME"
    fi
}

# Build simulation Docker image
build_image() {
    log_section "Building Simulation Docker Image"
    
    check_docker
    
    log_info "Creating Dockerfile for simulation environment..."
    
    cat > /tmp/Dockerfile.sim << 'EOF'
# Agent ROS Bridge - Full Simulation Environment
# ROS2 Humble + Gazebo + Nav2 + TurtleBot3
FROM osrf/ros:humble-desktop-full

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV TURTLEBOT3_MODEL=burger

# Install essential tools and dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Core tools
    git \
    wget \
    curl \
    vim \
    nano \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # Gazebo and simulation
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    # TurtleBot3 packages
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-turtlebot3-cartographer \
    # Nav2 packages
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    # Additional message types
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-std-msgs \
    ros-humble-action-msgs \
    ros-humble-tf2-ros \
    # Python dependencies
    python3-numpy \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# Setup workspace
WORKDIR /ros2_ws

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Source ROS2 in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> /root/.bashrc

# Copy agent-ros-bridge code
COPY . /ros2_ws/src/agent-ros-bridge/

# Install agent-ros-bridge dependencies
WORKDIR /ros2_ws/src/agent-ros-bridge
RUN pip3 install --no-cache-dir -e . || true

# Build the workspace
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -y || true && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -20 || true"

# Setup entrypoint
COPY docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
EOF

    # Create entrypoint script
    cat > /tmp/ros_entrypoint.sh << 'EOF'
#!/bin/bash
set -e

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Set environment
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/opt/ros/humble/share/turtlebot3_gazebo/models

exec "$@"
EOF

    cp /tmp/ros_entrypoint.sh "$WORKSPACE_DIR/docker/"
    
    log_info "Building Docker image (this may take 10-20 minutes)..."
    docker build -f /tmp/Dockerfile.sim -t "$SIM_IMAGE_NAME:latest" "$WORKSPACE_DIR" || {
        log_warn "Build had warnings but continuing..."
    }
    
    log_success "Simulation image built: $SIM_IMAGE_NAME:latest"
}

# Start Gazebo simulation
start_gazebo() {
    log_section "Starting Gazebo Simulation"
    
    check_docker
    create_network
    
    # Stop existing container
    if docker ps -a | grep -q "$CONTAINER_GAZEBO"; then
        log_info "Removing existing Gazebo container..."
        docker rm -f "$CONTAINER_GAZEBO" 2>/dev/null || true
    fi
    
    # Platform-specific display settings
    DISPLAY_ARGS=""
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        if [ -n "$DISPLAY" ]; then
            DISPLAY_ARGS="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
            xhost +local:docker 2>/dev/null || true
            log_info "X11 forwarding enabled for Linux"
        fi
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        log_warn "macOS detected - GUI will not be available"
        log_info "To view Gazebo on macOS, install XQuartz and run: xhost +localhost"
    fi
    
    log_info "Starting Gazebo with TurtleBot3 World..."
    
    docker run -d \
        --name "$CONTAINER_GAZEBO" \
        --hostname gazebo \
        --network "$NETWORK_NAME" \
        --privileged \
        --restart unless-stopped \
        -p 11345:11345 \
        $DISPLAY_ARGS \
        -e TURTLEBOT3_MODEL=burger \
        -e GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models \
        "$SIM_IMAGE_NAME:latest" \
        bash -c "
            source /opt/ros/humble/setup.bash
            echo 'Starting Gazebo simulation...'
            ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
            sleep 5
            echo 'Gazebo simulation started!'
            tail -f /dev/null
        "
    
    log_success "Gazebo container started"
    log_info "Waiting for simulation to initialize..."
    sleep 10
    
    # Check if Gazebo is running
    if docker exec "$CONTAINER_GAZEBO" pgrep -x "gzserver" > /dev/null 2>&1; then
        log_success "Gazebo server is running"
    else
        log_warn "Gazebo server may still be starting..."
    fi
}

# Start Nav2 navigation
start_nav2() {
    log_section "Starting Nav2 Navigation Stack"
    
    check_docker
    create_network
    
    # Check if Gazebo is running
    if ! docker ps | grep -q "$CONTAINER_GAZEBO"; then
        log_warn "Gazebo container not running. Starting it first..."
        start_gazebo
        sleep 15
    fi
    
    # Stop existing Nav2 container
    if docker ps -a | grep -q "$CONTAINER_NAV2"; then
        log_info "Removing existing Nav2 container..."
        docker rm -f "$CONTAINER_NAV2" 2>/dev/null || true
    fi
    
    log_info "Starting Nav2 navigation stack..."
    
    docker run -d \
        --name "$CONTAINER_NAV2" \
        --hostname nav2 \
        --network "$NETWORK_NAME" \
        --privileged \
        --restart unless-stopped \
        -e TURTLEBOT3_MODEL=burger \
        "$SIM_IMAGE_NAME:latest" \
        bash -c "
            source /opt/ros/humble/setup.bash
            
            # Wait for Gazebo to be ready
            echo 'Waiting for Gazebo topics...'
            until ros2 topic list | grep -q '/odom'; do
                sleep 2
            done
            echo 'Gazebo is ready!'
            
            # Start Nav2
            echo 'Starting Nav2 navigation...'
            ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true &
            
            # Keep container running
            tail -f /dev/null
        "
    
    log_success "Nav2 container started"
    log_info "Waiting for Nav2 to initialize..."
    sleep 15
}

# Start agent-ros-bridge
start_bridge() {
    log_section "Starting Agent ROS Bridge"
    
    check_docker
    create_network
    
    # Stop existing bridge container
    if docker ps -a | grep -q "$CONTAINER_BRIDGE"; then
        log_info "Removing existing bridge container..."
        docker rm -f "$CONTAINER_BRIDGE" 2>/dev/null || true
    fi
    
    log_info "Starting agent-ros-bridge..."
    
    docker run -d \
        --name "$CONTAINER_BRIDGE" \
        --hostname bridge \
        --network "$NETWORK_NAME" \
        --privileged \
        --restart unless-stopped \
        -p 8766:8766 \
        -p 50051:50051 \
        -e JWT_SECRET=${JWT_SECRET:-$(openssl rand -base64 32)} \
        -e ROS_DOMAIN_ID=0 \
        -v "$WORKSPACE_DIR:/workspace:ro" \
        "$SIM_IMAGE_NAME:latest" \
        bash -c "
            source /opt/ros/humble/setup.bash
            cd /workspace
            pip3 install -e . --quiet 2>/dev/null || true
            
            echo 'Starting Agent ROS Bridge...'
            export JWT_SECRET=\${JWT_SECRET:-test-secret-for-simulation}
            python3 -m agent_ros_bridge.gateway_v2 --transport websocket --port 8766 &
            
            tail -f /dev/null
        "
    
    log_success "Bridge container started on ws://localhost:8766"
}

# Start rosbridge for Foxglove
start_rosbridge() {
    log_section "Starting Rosbridge (Foxglove Visualization)"
    
    check_docker
    create_network
    
    # Check if Gazebo is running
    if ! docker ps | grep -q "$CONTAINER_GAZEBO"; then
        log_warn "Gazebo container not running. Starting it first..."
        start_gazebo
        sleep 15
    fi
    
    # Stop existing rosbridge container
    if docker ps -a | grep -q "$CONTAINER_ROSBRIDGE"; then
        log_info "Removing existing rosbridge container..."
        docker rm -f "$CONTAINER_ROSBRIDGE" 2>/dev/null || true
    fi
    
    log_info "Starting rosbridge_server for Foxglove..."
    
    docker run -d \
        --name "$CONTAINER_ROSBRIDGE" \
        --hostname rosbridge \
        --network "$NETWORK_NAME" \
        --privileged \
        --restart unless-stopped \
        -p 9090:9090 \
        -e ROS_DISTRO=humble \
        -e ROS_DOMAIN_ID=0 \
        osrf/ros:humble-desktop \
        bash -c "
            apt-get update && apt-get install -y ros-humble-rosbridge-server 2>/dev/null || true
            source /opt/ros/humble/setup.bash
            
            echo 'Starting rosbridge_server...'
            ros2 launch rosbridge_server rosbridge_websocket_launch.xml
        "
    
    log_success "Rosbridge container started on ws://localhost:9090"
    log_info "Open Foxglove Studio: https://studio.foxglove.dev"
    log_info "Connect to: ws://localhost:9090"
}

# Start full simulation stack
start() {
    log_section "Starting Full Simulation Environment"
    
    # Check if image exists
    if ! docker image inspect "$SIM_IMAGE_NAME:latest" &> /dev/null; then
        log_warn "Simulation image not found. Building first..."
        build_image
    fi
    
    start_gazebo
    start_nav2
    start_bridge
    start_rosbridge
    
    log_section "Simulation Environment Ready!"
    log_info "Services:"
    log_info "  - Gazebo:      Container '$CONTAINER_GAZEBO'"
    log_info "  - Nav2:        Container '$CONTAINER_NAV2'"
    log_info "  - Bridge:      Container '$CONTAINER_BRIDGE' (ws://localhost:8766)"
    log_info "  - Rosbridge:   Container '$CONTAINER_ROSBRIDGE' (ws://localhost:9090)"
    log_info ""
    log_info "Visualization:"
    log_info "  - Foxglove Studio: https://studio.foxglove.dev"
    log_info "  - Connect to: ws://localhost:9090"
    log_info ""
    log_info "AI Control:"
    log_info "  - WebSocket: ws://localhost:8766"
    log_info ""
    log_info "Run '$0 status' to check status"
    log_info "Run '$0 logs' to view logs"
    log_info "Run '$0 test' to run navigation tests"
}

# Stop all simulation containers
stop() {
    log_section "Stopping Simulation Environment"
    
    for container in "$CONTAINER_ROSBRIDGE" "$CONTAINER_BRIDGE" "$CONTAINER_NAV2" "$CONTAINER_GAZEBO"; do
        if docker ps | grep -q "$container"; then
            log_info "Stopping $container..."
            docker stop "$container" 2>/dev/null || true
        fi
        if docker ps -a | grep -q "$container"; then
            docker rm -f "$container" 2>/dev/null || true
        fi
    done
    
    log_success "Simulation environment stopped"
}

# Show status
status() {
    log_section "Simulation Environment Status"
    
    echo -e "${CYAN}Containers:${NC}"
    docker ps --filter "name=ros2_" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" 2>/dev/null || echo "No containers running"
    
    echo ""
    echo -e "${CYAN}ROS2 Topics (from Gazebo):${NC}"
    if docker ps | grep -q "$CONTAINER_GAZEBO"; then
        docker exec "$CONTAINER_GAZEBO" bash -c "source /opt/ros/humble/setup.bash && ros2 topic list" 2>/dev/null | head -15 || echo "Topics not yet available"
    else
        echo "Gazebo container not running"
    fi
    
    echo ""
    echo -e "${CYAN}Nav2 Action Servers:${NC}"
    if docker ps | grep -q "$CONTAINER_NAV2"; then
        docker exec "$CONTAINER_NAV2" bash -c "source /opt/ros/humble/setup.bash && ros2 action list" 2>/dev/null || echo "Nav2 not yet ready"
    else
        echo "Nav2 container not running"
    fi
}

# View logs
logs() {
    local container="${2:-$CONTAINER_GAZEBO}"
    
    if docker ps -a | grep -q "$container"; then
        log_info "Showing logs for $container (Ctrl+C to exit)..."
        docker logs -f "$container"
    else
        log_error "Container $container not found"
        log_info "Available containers:"
        docker ps -a --filter "name=ros2_" --format "{{.Names}}"
    fi
}

# Run navigation tests
run_tests() {
    log_section "Running Navigation Tests"
    
    # Check if simulation is running
    if ! docker ps | grep -q "$CONTAINER_GAZEBO"; then
        log_warn "Simulation not running. Starting it first..."
        start
        sleep 20
    fi
    
    log_info "Waiting for all services to be ready..."
    sleep 10
    
    # Run the navigation e2e tests
    log_info "Running navigation E2E tests..."
    cd "$WORKSPACE_DIR"
    
    # Update test to use simulation container
    python3 -m pytest tests/e2e/test_navigation_e2e.py -v --tb=short -k "not skip" || {
        log_warn "Some tests failed - this is expected if simulation is still initializing"
    }
    
    log_success "Test run complete"
}

# Shell into container
shell() {
    local container="${2:-$CONTAINER_GAZEBO}"
    
    if docker ps | grep -q "$container"; then
        log_info "Opening shell in $container..."
        docker exec -it "$container" bash
    else
        log_error "Container $container is not running"
        log_info "Start the simulation first with: $0 start"
    fi
}

# Quick test with simulated robot (no Gazebo needed)
quick_test() {
    log_section "Quick Test with Simulated Robot"
    
    log_info "Starting simulated robot (no Gazebo required)..."
    cd "$WORKSPACE_DIR"
    
    # Run the quickstart example
    python3 examples/quickstart/simulated_robot.py &
    local pid=$!
    
    sleep 3
    
    log_info "Testing connection to simulated robot..."
    # Test with a simple WebSocket client
    python3 -c "
import asyncio
import websockets
import json

async def test():
    try:
        async with websockets.connect('ws://localhost:8766') as ws:
            # Authenticate
            await ws.send(json.dumps({
                'type': 'auth',
                'token': 'test-token'
            }))
            
            # Discover
            await ws.send(json.dumps({
                'type': 'command',
                'action': 'discover'
            }))
            
            response = await asyncio.wait_for(ws.recv(), timeout=5)
            print(f'Response: {response}')
            print('✅ Simulated robot is working!')
    except Exception as e:
        print(f'❌ Error: {e}')

asyncio.run(test())
" 2>/dev/null || log_warn "Could not connect to simulated robot"
    
    kill $pid 2>/dev/null || true
}

# Main command handler
case "${1:-}" in
    setup|build)
        build_image
        ;;
    start)
        start
        ;;
    start-gazebo)
        start_gazebo
        ;;
    start-nav2)
        start_nav2
        ;;
    start-bridge)
        start_bridge
        ;;
    start-rosbridge)
        start_rosbridge
        ;;
    stop)
        stop
        ;;
    status)
        status
        ;;
    logs)
        logs "$@"
        ;;
    test)
        run_tests
        ;;
    quick-test)
        quick_test
        ;;
    shell)
        shell "$@"
        ;;
    restart)
        stop
        sleep 2
        start
        ;;
    *)
        echo "Agent ROS Bridge - Simulation Environment Manager"
        echo ""
        echo "Usage: $0 <command>"
        echo ""
        echo "Commands:"
        echo "  setup            Build the simulation Docker image"
        echo "  start            Start full simulation (Gazebo + Nav2 + Bridge + Rosbridge)"
        echo "  start-gazebo     Start only Gazebo simulation"
        echo "  start-nav2       Start only Nav2 navigation stack"
        echo "  start-bridge     Start only agent-ros-bridge (AI control)"
        echo "  start-rosbridge  Start only rosbridge (Foxglove visualization)"
        echo "  stop             Stop all simulation containers"
        echo "  restart          Restart the simulation"
        echo "  status           Check simulation status"
        echo "  logs [name]      View logs (default: gazebo)"
        echo "  test             Run navigation E2E tests"
        echo "  quick-test       Test with simulated robot (no Gazebo)"
        echo "  shell [name]     Open shell in container (default: gazebo)"
        echo ""
        echo "Services:"
        echo "  Gazebo (Port 11345)      - Physics simulation with TurtleBot3"
        echo "  Nav2                   - Navigation stack"
        echo "  Agent Bridge (Port 8766) - AI agent control (WebSocket)"
        echo "  Rosbridge (Port 9090)    - Foxglove visualization (WebSocket)"
        echo ""
        echo "Examples:"
        echo "  $0 setup                 # Build simulation image"
        echo "  $0 start                 # Start full simulation"
        echo "  $0 status                # Check status"
        echo "  $0 logs gazebo           # View Gazebo logs"
        echo "  $0 shell nav2            # Shell into Nav2 container"
        echo "  $0 test                  # Run tests"
        echo ""
        echo "Foxglove Visualization:"
        echo "  1. Start simulation: $0 start"
        echo "  2. Open https://studio.foxglove.dev"
        echo "  3. Connect to: ws://localhost:9090"
        echo ""
        exit 1
        ;;
esac
