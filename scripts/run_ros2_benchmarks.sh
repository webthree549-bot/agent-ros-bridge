#!/bin/bash
# Run ROS2-dependent benchmarks in the ros2_jazzy container

set -e

echo "=========================================="
echo "Running ROS2 Performance Benchmarks"
echo "=========================================="
echo ""

# Check if container is running
if ! docker ps | grep -q ros2_jazzy; then
    echo "❌ ros2_jazzy container is not running!"
    echo ""
    echo "Start it with:"
    echo "  ./scripts/docker/docker-manager.sh start"
    exit 1
fi

echo "✅ Container is running"
echo ""

# Copy current workspace to container
echo "📦 Copying workspace to container..."
docker cp . ros2_jazzy:/workspace/

# Install package in container
echo "📦 Installing package in container..."
docker exec ros2_jazzy bash -c "cd /workspace && pip install -e . -q"

# Run benchmarks
echo ""
echo "🚀 Running ROS2 benchmarks..."
echo "=========================================="
docker exec ros2_jazzy bash -c "cd /workspace && python3 -m pytest tests/performance/test_ros2_benchmarks.py -v --tb=short"

echo ""
echo "=========================================="
echo "✅ ROS2 Benchmarks Complete"
echo "=========================================="
