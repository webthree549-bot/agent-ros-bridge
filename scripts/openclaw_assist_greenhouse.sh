#!/bin/bash
# DEPRECATED: This script has moved to demo/greenhouse/scripts/
# 
# This file is kept for backward compatibility.
# The greenhouse demo is now in demo/greenhouse/ as an application example.

echo "================================================"
echo "  ⚠️  DEPRECATED"
echo "================================================"
echo ""
echo "This script functionality has moved to:"
echo "  demo/greenhouse/scripts/gh_control.sh"
echo ""
echo "The greenhouse demo is now an APPLICATION built"
echo "on top of the generic OpenClaw ROS Bridge."
echo ""
echo "To control the greenhouse:"
echo "  ./demo/greenhouse/scripts/gh_control.sh status"
echo "  ./demo/greenhouse/scripts/gh_control.sh fan on"
echo "  ./demo/greenhouse/scripts/gh_control.sh valve open"
echo ""

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

# Check if gh_control.sh exists in new location
if [[ -f "$PROJECT_ROOT/demo/greenhouse/scripts/gh_control.sh" ]]; then
    exec "$PROJECT_ROOT/demo/greenhouse/scripts/gh_control.sh" "$@"
else
    echo "Error: New script location not found."
    echo "Please check that demo/greenhouse/scripts/ exists."
    exit 1
fi
