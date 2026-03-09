#!/bin/bash
#
# ROS Docker Test Runner
# 
# This script runs all tests using the ROS2 Docker container.
# No tests should be skipped - all must run in the container.
#

set -e

echo "ROS Docker Test Runner"
echo "======================"
echo ""

# Check if ROS2 container is running
if ! docker ps | grep -q ros2_humble; then
    echo "Starting ROS2 container..."
    docker start ros2_humble
    sleep 3
fi

# Verify container is running
if ! docker ps | grep -q ros2_humble; then
    echo "❌ Failed to start ROS2 container"
    exit 1
fi

echo "✅ ROS2 container is running"
echo ""

# Install test dependencies in container
echo "Installing test dependencies..."
docker exec ros2_humble bash -c "
    source /opt/ros/humble/setup.bash
    pip install pytest pytest-asyncio -q 2>/dev/null || true
"

# Copy workspace to container
echo "Copying workspace to container..."
docker cp /Users/webthree/.openclaw/workspace ros2_humble:/workspace

# Run tests in container
echo ""
echo "Running tests in ROS2 Docker container..."
echo "=========================================="
docker exec ros2_humble bash -c "
    source /opt/ros/humble/setup.bash
    cd /workspace
    
    # Set environment to prevent skipping
    export FORCE_ROS_TESTS=1
    export ROS_TESTING=1
    
    # Run all tests with verbose output
    python3 -m pytest tests/ \
        -v \
        --tb=short \
        --no-header \
        -p no:cacheprovider \
        2>&1
"

echo ""
echo "Test run complete!"
