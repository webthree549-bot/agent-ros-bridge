#!/bin/bash
# Quickstart Example - Mock Bridge

set -e

echo "🚀 Starting Agent ROS Bridge - Quickstart"
echo "=========================================="
echo ""

# Check Python
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 not found"
    exit 1
fi

# Check if running from correct directory
if [ ! -f "mock_bridge.py" ]; then
    echo "❌ Run this script from the quickstart/ directory"
    exit 1
fi

echo "Starting mock bridge on ws://localhost:8765"
echo ""
echo "Test commands:"
echo "  wscat -c ws://localhost:8765"
echo "  > {\"command\": {\"action\": \"list_robots\"}}"
echo ""
echo "Press Ctrl+C to stop"
echo ""

python3 mock_bridge.py "$@"
