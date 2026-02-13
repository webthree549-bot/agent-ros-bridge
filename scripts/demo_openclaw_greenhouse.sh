#!/bin/bash
# DEPRECATED: This script has moved to demo/greenhouse/scripts/demo_openclaw.sh
# 
# This file is kept for backward compatibility but delegates to the new location.
# The greenhouse demo is now in demo/greenhouse/ as an application example.

echo "================================================"
echo "  ⚠️  DEPRECATED"
echo "================================================"
echo ""
echo "This script has moved to:"
echo "  demo/greenhouse/scripts/demo_openclaw.sh"
echo ""
echo "The greenhouse demo is now an APPLICATION built"
echo "on top of the generic OpenClaw ROS Bridge."
echo ""
echo "Redirecting to new location..."
echo ""

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
exec "$PROJECT_ROOT/demo/greenhouse/scripts/demo_openclaw.sh" "$@"
