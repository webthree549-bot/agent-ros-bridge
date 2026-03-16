#!/bin/bash
# Run E2E tests for Agent ROS Bridge

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  Agent ROS Bridge - E2E Test Runner       ${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^ros2_humble$"; then
    echo -e "${YELLOW}ROS2 container not running. Starting...${NC}"
    "${SCRIPT_DIR}/docker-manager.sh" start
    echo ""
fi

# Check Nav2 installation
echo -e "${BLUE}Checking Nav2 installation...${NC}"
if docker exec ros2_humble bash -c "ros2 pkg list | grep -q nav2" 2>/dev/null; then
    echo -e "${GREEN}✅ Nav2 is installed${NC}"
else
    echo -e "${YELLOW}⚠️  Nav2 is not installed. Some tests will be skipped.${NC}"
    echo "   To install Nav2, rebuild the image:"
    echo "   ./scripts/docker-manager.sh build"
fi
echo ""

# Run E2E tests
echo -e "${BLUE}Running E2E tests...${NC}"
echo ""

cd "${PROJECT_ROOT}"

# Run with verbose output
python3 -m pytest tests/e2e/ -v "$@"

echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}  E2E Tests Complete                        ${NC}"
echo -e "${GREEN}============================================${NC}"
