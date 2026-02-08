#!/bin/bash
# cleanup.sh - Reset workspace to clean state

cd /app 2>/dev/null || cd /Users/webthree/.openclaw/skills/openclaw-ros-bridge 2>/dev/null || exit 1

echo "Cleaning up build artifacts..."
rm -rf build/ install/ log/ catkin_ws/ .build_complete

echo "Removing Python cache..."
find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null
find . -type f -name "*.pyc" -delete 2>/dev/null
find . -type f -name "*.pyo" -delete 2>/dev/null

echo "Removing egg-info..."
rm -rf *.egg-info/ openclaw_ros_bridge.egg-info/ 2>/dev/null

echo "Done. Workspace is clean."
echo ""
echo "To rebuild:"
echo "  ./scripts/build.sh"
echo "  source install/setup.bash"
echo "  ./scripts/run_demo.sh --greenhouse --mock"
