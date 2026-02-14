#!/bin/bash
# ROS Actions Example - Long-running Tasks

set -e

echo "🎯 Starting ROS Actions Demo"
echo "============================="
echo ""

if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 not found"
    exit 1
fi

if [ ! -f "actions_demo.py" ]; then
    echo "❌ Run this script from the actions/ directory"
    exit 1
fi

ACTION="${1:-navigate}"

echo "Starting actions server on ws://localhost:8773"
echo ""
echo "Demo action: $ACTION"
echo ""
echo "Test commands:"
echo "  wscat -c ws://localhost:8773"
echo "  > {\"command\": {\"action\": \"actions.navigate\", \"parameters\": {\"x\": 5.0, \"y\": 3.0}}}"
echo ""
echo "Press Ctrl+C to stop"
echo ""

python3 actions_demo.py --action "$ACTION"
