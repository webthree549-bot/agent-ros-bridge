#!/bin/bash
# Integration Test Runner for Agent ROS Bridge
# Starts Docker ROS2 environment, runs tests, cleans up

set -e

echo "=========================================="
echo "🧪 Agent ROS Bridge Integration Tests"
echo "=========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
COMPOSE_FILE="docker-compose.yml"
TEST_TIMEOUT=60
BRIDGE_URL="ws://localhost:8767"

# Parse arguments
MOCK_MODE=false
SKIP_DOCKER=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --mock)
            MOCK_MODE=true
            shift
            ;;
        --skip-docker)
            SKIP_DOCKER=true
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --mock          Run tests against mock bridge (no Docker)"
            echo "  --skip-docker   Skip Docker cleanup/start (assume already running)"
            echo "  --help          Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Function to cleanup
cleanup() {
    echo ""
    echo "🧹 Cleaning up..."
    
    if [ "$MOCK_MODE" = true ]; then
        pkill -f "python demo/mock_bridge.py" || true
    elif [ "$SKIP_DOCKER" = false ]; then
        echo "Stopping Docker containers..."
        docker-compose -f $COMPOSE_FILE --profile ros2 down || true
    fi
    
    echo "✅ Cleanup complete"
}

# Set trap to cleanup on exit
trap cleanup EXIT

# Check if running in virtual environment
if [ -z "$VIRTUAL_ENV" ]; then
    echo -e "${YELLOW}⚠️  Not in virtual environment. Activating...${NC}"
    if [ -f "../agent-ros-bridge-test-venv/bin/activate" ]; then
        source ../agent-ros-bridge-test-venv/bin/activate
    elif [ -f "agent-ros-bridge-test-venv/bin/activate" ]; then
        source agent-ros-bridge-test-venv/bin/activate
    else
        echo -e "${YELLOW}⚠️  Could not find virtual environment${NC}"
    fi
fi

# Install test dependencies
echo "📦 Installing test dependencies..."
pip install pytest pytest-asyncio websockets paho-mqtt -q

# Start bridge
if [ "$MOCK_MODE" = true ]; then
    echo "🤖 Starting mock bridge..."
    python demo/mock_bridge.py > /tmp/mock_bridge.log 2>&1 &
    BRIDGE_PID=$!
    BRIDGE_URL="ws://localhost:8766"
    
    # Wait for bridge to start
    echo "⏳ Waiting for mock bridge to start..."
    for i in {1..30}; do
        if curl -s http://localhost:8766 > /dev/null 2>&1; then
            break
        fi
        sleep 1
    done
    
elif [ "$SKIP_DOCKER" = false ]; then
    echo "🐳 Starting Docker ROS2 bridge..."
    docker-compose -f $COMPOSE_FILE --profile ros2 up --build -d ros2-bridge
    
    # Wait for bridge to be ready
    echo "⏳ Waiting for bridge to be ready..."
    for i in {1..60}; do
        if curl -s http://localhost:8767 > /dev/null 2>&1; then
            echo "✅ Bridge is ready!"
            break
        fi
        echo -n "."
        sleep 1
    done
fi

# Export environment variables for tests
export BRIDGE_URL=$BRIDGE_URL
export MQTT_BROKER=localhost
export MQTT_PORT=1883

echo ""
echo "🧪 Running integration tests..."
echo "   Bridge URL: $BRIDGE_URL"
echo ""

# Run tests
pytest tests/integration/ -v --timeout=$TEST_TIMEOUT

TEST_RESULT=$?

echo ""
if [ $TEST_RESULT -eq 0 ]; then
    echo -e "${GREEN}✅ All tests passed!${NC}"
else
    echo -e "${RED}❌ Some tests failed${NC}"
fi

exit $TEST_RESULT
