#!/bin/bash
# Entrypoint script for optimized Agent ROS Bridge container

set -e

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Function to start the gateway
cmd_start() {
    echo "🚀 Starting Agent ROS Bridge v0.6.5..."
    echo ""
    echo "Services:"
    echo "  - HTTP Dashboard: http://localhost:8080/"
    echo "  - WebSocket: ws://localhost:8765/"
    echo "  - ROS Bridge: http://localhost:9090/"
    echo ""
    
    # Start the gateway
    cd /workspace
    exec python3 -m agent_ros_bridge.gateway_v2 --config config/global_config.yaml
}

# Function to run tests
cmd_test() {
    echo "🧪 Running TDD tests..."
    cd /workspace
    python3 -m pytest tests/unit/gateway_v2/transports/test_http_transport_tdd.py -v
}

# Function to check status
cmd_status() {
    echo "📊 Checking Agent ROS Bridge status..."
    curl -s http://localhost:8080/api/health 2>/dev/null || echo "Gateway not running"
}

# Main command handler
case "${1:-start}" in
    start)
        cmd_start
        ;;
    test)
        cmd_test
        ;;
    status)
        cmd_status
        ;;
    shell|bash|sh)
        exec /bin/bash
        ;;
    *)
        echo "Usage: $0 {start|test|status|shell}"
        exit 1
        ;;
esac
