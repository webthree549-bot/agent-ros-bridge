#!/bin/bash
# start_bridge.sh - Start Agent ROS Bridge with proper configuration
#
# Usage:
#   ./start_bridge.sh              # Start with defaults
#   WEBSOCKET_PORT=9000 ./start_bridge.sh  # Custom port
#   ./start_bridge.sh --daemon     # Run as daemon

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "=========================================="
echo "🚀 Agent ROS Bridge - Startup Script"
echo "=========================================="
echo ""

# Generate JWT secret if not set
if [ -z "$JWT_SECRET" ]; then
    export JWT_SECRET=$(openssl rand -base64 32)
    echo -e "${GREEN}✅ Generated JWT_SECRET${NC}"
    echo "   Secret: ${JWT_SECRET:0:20}..."
    echo ""
    echo -e "${YELLOW}⚠️  Save this secret! You'll need it to connect.${NC}"
    echo "   Export it later with: export JWT_SECRET=$JWT_SECRET"
    echo ""
else
    echo -e "${GREEN}✅ Using existing JWT_SECRET${NC}"
    echo "   Secret: ${JWT_SECRET:0:20}..."
    echo ""
fi

# Set defaults
export WEBSOCKET_PORT=${WEBSOCKET_PORT:-8765}
export MQTT_PORT=${MQTT_PORT:-1883}
export GRPC_PORT=${GRPC_PORT:-50051}
export LOG_LEVEL=${LOG_LEVEL:-INFO}
export DATABASE_URL=${DATABASE_URL:-sqlite:///agent_ros_bridge.db}

echo -e "${BLUE}📡 Configuration:${NC}"
echo "   WebSocket Port: $WEBSOCKET_PORT"
echo "   MQTT Port:      $MQTT_PORT"
echo "   gRPC Port:      $GRPC_PORT"
echo "   Log Level:      $LOG_LEVEL"
echo "   Database:       $DATABASE_URL"
echo ""

# Check if ports are already in use
check_port() {
    local port=$1
    local name=$2
    if lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1; then
        echo -e "${RED}❌ Port $port is already in use! ($name)${NC}"
        return 1
    fi
    return 0
}

echo -e "${BLUE}🔍 Checking ports...${NC}"
PORT_OK=true
check_port $WEBSOCKET_PORT "WebSocket" || PORT_OK=false
check_port $MQTT_PORT "MQTT" || PORT_OK=false
check_port $GRPC_PORT "gRPC" || PORT_OK=false

if [ "$PORT_OK" = false ]; then
    echo ""
    echo -e "${RED}❌ Some ports are already in use. Please stop other services or change ports.${NC}"
    exit 1
fi

echo -e "${GREEN}✅ All ports available${NC}"
echo ""

# Check for Docker ROS2 container
if docker ps | grep -q ros2_humble; then
    echo -e "${GREEN}✅ ROS2 Docker container detected${NC}"
    echo "   Container: ros2_humble"
    echo ""
fi

# Create config directory if needed
mkdir -p config

# Build command arguments
ARGS=""

# Add port arguments
ARGS="$ARGS --websocket-port $WEBSOCKET_PORT"
ARGS="$ARGS --mqtt-port $MQTT_PORT"
ARGS="$ARGS --grpc-port $GRPC_PORT"
ARGS="$ARGS --log-level $LOG_LEVEL"
ARGS="$ARGS --database-url $DATABASE_URL"

# Add daemon flag if requested
if [ "$1" = "--daemon" ] || [ "$1" = "-D" ]; then
    ARGS="$ARGS start --daemon"
    echo -e "${BLUE}👻 Running as daemon${NC}"
else
    ARGS="$ARGS start"
fi

echo -e "${GREEN}🚀 Starting Agent ROS Bridge...${NC}"
echo ""
echo -e "${BLUE}Connection URLs:${NC}"
echo "   WebSocket: ws://localhost:$WEBSOCKET_PORT?token=<JWT_TOKEN>"
echo "   MQTT:      mqtt://localhost:$MQTT_PORT"
echo "   gRPC:      grpc://localhost:$GRPC_PORT"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop${NC}"
echo "=========================================="
echo ""

# Start the bridge
exec agent-ros-bridge $ARGS
