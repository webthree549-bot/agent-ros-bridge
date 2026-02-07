#!/bin/bash
set -euo pipefail

BLUE='\033[0;34m'
GREEN='\033[0;32m'
NC='\033[0m'

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PROJECT_ROOT"

log() { echo -e "${BLUE}[TEST] $1${NC}"; }
success() { echo -e "${GREEN}[SUCCESS] $1${NC}"; }

log "Running tests (MOCK MODE)..."
export MOCK_MODE="true"
pytest test/ -v --cov=openclaw_ros_bridge --cov-report=term-missing
success "Tests completed!"