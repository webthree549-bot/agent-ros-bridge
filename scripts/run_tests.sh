#!/bin/bash
set -euo pipefail

BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PROJECT_ROOT"

log() { echo -e "${BLUE}[TEST] $1${NC}"; }
success() { echo -e "${GREEN}[SUCCESS] $1${NC}"; }
warn() { echo -e "${YELLOW}[WARN] $1${NC}"; }

# Parse arguments
FAST_MODE="false"
WITH_ROS="false"
WITH_INTEGRATION="false"
PARALLEL="auto"

while [[ $# -gt 0 ]]; do
    case $1 in
        --fast)
            FAST_MODE="true"
            shift
            ;;
        --with-ros)
            WITH_ROS="true"
            shift
            ;;
        --with-integration)
            WITH_INTEGRATION="true"
            shift
            ;;
        --parallel)
            PARALLEL="$2"
            shift 2
            ;;
        --no-parallel)
            PARALLEL="0"
            shift
            ;;
        *)
            shift
            ;;
    esac
done

log "Running tests with configuration:"
echo "  Fast mode: $FAST_MODE"
echo "  With ROS: $WITH_ROS"
echo "  With Integration: $WITH_INTEGRATION"
echo "  Parallel: $PARALLEL"
echo ""

# Build pytest arguments
PYTEST_ARGS=("-v")

if [[ "$FAST_MODE" == "true" ]]; then
    PYTEST_ARGS+=("--fast")
    warn "Running in fast mode (skipping slow tests)"
fi

if [[ "$WITH_ROS" == "true" ]]; then
    PYTEST_ARGS+=("--with-ros")
    log "Including ROS-dependent tests"
fi

if [[ "$WITH_INTEGRATION" == "true" ]]; then
    PYTEST_ARGS+=("--with-integration")
    log "Including integration tests"
fi

# Check for pytest-xdist
if [[ "$PARALLEL" != "0" ]] && python -c "import xdist" 2>/dev/null; then
    PYTEST_ARGS+=("-n" "$PARALLEL")
    log "Running tests in parallel (xdist)"
else
    warn "pytest-xdist not available, running tests sequentially"
fi

# Set environment
export MOCK_MODE="true"
export JWT_SECRET="test-secret-key-for-ci-only"

# Run tests
log "Starting test run..."
if pytest tests/ "${PYTEST_ARGS[@]}" --tb=short; then
    success "All tests passed!"
else
    warn "Some tests failed"
    exit 1
fi
