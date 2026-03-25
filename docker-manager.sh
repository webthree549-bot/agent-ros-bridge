#!/bin/bash
# DEPRECATED: This script has been moved to scripts/docker/docker-manager.sh
# Please use: ./scripts/docker/docker-manager.sh instead
#
# This wrapper remains for backward compatibility but will be removed in v0.7.0

echo "⚠️  DEPRECATION WARNING"
echo "======================"
echo ""
echo "This script location is deprecated."
echo "Please use the new location:"
echo ""
echo "  ./scripts/docker/docker-manager.sh $@"
echo ""
echo "Redirecting now..."
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/scripts/docker/docker-manager.sh" "$@"
