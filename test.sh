#!/bin/bash
# Test runner for local development
# Supports TDD workflow with all test suites

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "🧪 Agent ROS Bridge Test Runner"
echo "================================"

# Function to check if Docker container is running
is_ros2_running() {
    docker ps --format "{{.Names}}" | grep -q "^ros2_humble$"
}

# Parse arguments
TEST_TYPE="${1:-unit}"

 case "$TEST_TYPE" in
    unit|u)
        echo -e "${GREEN}Running UNIT tests...${NC}"
        pytest tests/unit -v --tb=short "$@"
        ;;

    e2e|e)
        if is_ros2_running; then
            echo -e "${GREEN}Running E2E tests with ROS2...${NC}"
            pytest tests/e2e -v --tb=short "$@"
        else
            echo -e "${YELLOW}⚠️  ROS2 container not running. Starting...${NC}"
            docker-compose -f docker-compose.ros2.yml up -d
            echo -e "${YELLOW}Waiting for ROS2 to be ready...${NC}"
            sleep 10
            echo -e "${GREEN}Running E2E tests...${NC}"
            pytest tests/e2e -v --tb=short "$@"
        fi
        ;;

    integration|i)
        if is_ros2_running; then
            echo -e "${GREEN}Running INTEGRATION tests with ROS2...${NC}"
            pytest tests/integration -v --tb=short "$@"
        else
            echo -e "${YELLOW}⚠️  ROS2 container not running. Starting...${NC}"
            docker-compose -f docker-compose.ros2.yml up -d
            echo -e "${YELLOW}Waiting for ROS2 to be ready...${NC}"
            sleep 10
            echo -e "${GREEN}Running INTEGRATION tests...${NC}"
            pytest tests/integration -v --tb=short "$@"
        fi
        ;;

    all|a)
        echo -e "${GREEN}Running ALL tests...${NC}"
        echo -e "${YELLOW}Unit tests:${NC}"
        pytest tests/unit -v --tb=short

        if is_ros2_running; then
            echo -e "${YELLOW}E2E tests:${NC}"
            pytest tests/e2e -v --tb=short || true
            echo -e "${YELLOW}Integration tests:${NC}"
            pytest tests/integration -v --tb=short || true
        else
            echo -e "${YELLOW}⚠️  Skipping E2E/Integration tests (ROS2 container not running)${NC}"
            echo -e "${YELLOW}   Start with: docker-compose -f docker-compose.ros2.yml up -d${NC}"
        fi
        ;;

    ci)
        echo -e "${GREEN}Running CI checks...${NC}"
        echo -e "${YELLOW}1. Ruff linting...${NC}"
        ruff check .
        echo -e "${YELLOW}2. Black formatting...${NC}"
        black --check .
        echo -e "${YELLOW}3. Unit tests...${NC}"
        pytest tests/unit -v --tb=short
        echo -e "${GREEN}✅ CI checks passed!${NC}"
        ;;

    tdd)
        echo -e "${GREEN}TDD Mode: Watch for changes and re-run tests${NC}"
        echo -e "${YELLOW}Watching tests/unit... Press Ctrl+C to stop${NC}"
        pytest tests/unit -v --tb=short -f --lf "$@"
        ;;

    help|h|*)
        echo "Usage: $0 [command] [pytest_args]"
        echo ""
        echo "Commands:"
        echo "  unit (u)         - Run unit tests only (default)"
        echo "  e2e (e)          - Run E2E tests (starts ROS2 container if needed)"
        echo "  integration (i)  - Run integration tests (starts ROS2 container if needed)"
        echo "  all (a)          - Run all tests"
        echo "  ci               - Run CI checks (lint + unit tests)"
        echo "  tdd              - TDD mode: watch for changes and re-run"
        echo "  help (h)         - Show this help"
        echo ""
        echo "Examples:"
        echo "  $0                    # Run unit tests"
        echo "  $0 unit -k test_auth  # Run unit tests matching 'test_auth'"
        echo "  $0 e2e                # Run E2E tests"
        echo "  $0 all                # Run all tests"
        echo "  $0 tdd                # TDD watch mode"
        ;;
esac
