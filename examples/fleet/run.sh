#!/bin/bash
# Fleet Example - Multi-Robot Orchestration

set -e

echo "🚁 Starting Fleet Orchestration Demo"
echo "======================================"
echo ""

if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 not found"
    exit 1
fi

if [ ! -f "fleet_demo.py" ]; then
    echo "❌ Run this script from the fleet/ directory"
    exit 1
fi

echo "Starting fleet orchestrator on ws://localhost:8771"
echo ""
echo "Simulating 4 robots:"
echo "  - tb4_001 (TurtleBot4-Alpha)"
echo "  - tb4_002 (TurtleBot4-Beta)"
echo "  - ur5_001 (UR5-Arm-Station1)"
echo "  - drone_001 (Drone-Alpha)"
echo ""
echo "Test commands:"
echo "  wscat -c ws://localhost:8771"
echo "  > {\"command\": {\"action\": \"fleet.status\"}}"
echo "  > {\"command\": {\"action\": \"fleet.submit_task\", \"parameters\": {\"type\": \"navigate\", \"target_location\": \"zone_a\"}}}"
echo ""
echo "Press Ctrl+C to stop"
echo ""

python3 fleet_demo.py "$@"
