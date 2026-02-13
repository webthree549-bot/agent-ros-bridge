#!/bin/bash
# DEPRECATED: This script has moved to demo/greenhouse/scripts/gh_control.sh
# 
# This file is kept for backward compatibility but delegates to the new location.

echo "Note: This script has moved to demo/greenhouse/scripts/gh_control.sh" >&2

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
exec "$PROJECT_ROOT/demo/greenhouse/scripts/gh_control.sh" "$@"
