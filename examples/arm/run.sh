#!/bin/bash
# Arm Robot Example - Manipulation Demo

set -e

echo "🦾 Starting Arm Robot Demo"
echo "==========================="
echo ""

if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 not found"
    exit 1
fi

if [ ! -f "arm_demo.py" ]; then
    echo "❌ Run this script from the arm/ directory"
    exit 1
fi

echo "Starting arm bridge on ws://localhost:8772"
echo ""
echo "Usage: ./run.sh [options]"
echo ""
echo "Options:"
echo "  --arm-type ur|xarm|franka    (default: ur)"
echo "  --ros-version ros1|ros2      (default: ros2)"
echo "  --demo pick_place|interactive|state"
echo ""
echo "Test commands:"
echo "  wscat -c ws://localhost:8772"
echo "  > {\"command\": {\"action\": \"arm.move_joints\", \"parameters\": {\"joints\": [0, -1.57, 0, -1.57, 0, 0]}}}"
echo ""
echo "Press Ctrl+C to stop"
echo ""

python3 arm_demo.py "$@"
