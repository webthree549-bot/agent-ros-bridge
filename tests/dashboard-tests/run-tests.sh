#!/bin/bash
# Quick test runner for dashboard tests

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  Agent ROS Bridge Dashboard Tests                      ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════╝${NC}"
echo ""

# Check if in right directory
if [ ! -f "package.json" ]; then
    echo -e "${YELLOW}Switching to test directory...${NC}"
    cd "$(dirname "$0")" || exit 1
fi

# Install dependencies if needed
if [ ! -d "node_modules" ]; then
    echo -e "${YELLOW}Installing dependencies...${NC}"
    npm install
    echo ""
fi

# Parse arguments
TEST_TYPE=${1:-all}

run_security() {
    echo -e "${GREEN}Running Security Audit...${NC}"
    echo "========================================"
    npm run test:security || true
    echo ""
}

run_e2e() {
    echo -e "${GREEN}Running E2E Tests...${NC}"
    echo "========================================"
    npm run test:e2e || true
    echo ""
}

run_load() {
    echo -e "${GREEN}Running Load Tests...${NC}"
    echo "========================================"
    npm run test:load:100 || true
    echo ""
}

# Run based on argument
case $TEST_TYPE in
    security)
        run_security
        ;;
    e2e)
        run_e2e
        ;;
    load)
        run_load
        ;;
    all|*)
        run_security
        run_e2e
        run_load
        ;;
esac

echo -e "${GREEN}Done!${NC}"
echo ""
echo "Run individual tests:"
echo "  ./run-tests.sh security"
echo "  ./run-tests.sh e2e"
echo "  ./run-tests.sh load"