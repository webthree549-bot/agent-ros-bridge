#!/bin/bash
# build-and-test.sh - Complete build and test workflow for Agent ROS Bridge
# Usage: ./build-and-test.sh [test|build|install|clean|all]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log() { echo -e "${BLUE}[BUILD]${NC} $1"; }
success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Configuration - ALL outside source directory
VENV_DIR="$HOME/agent-ros-bridge-test-venv"
BUILD_DIR="$HOME/agent-ros-bridge-build-artifacts"

create_venv() {
    log "Creating virtual environment at $VENV_DIR..."
    if [ -d "$VENV_DIR" ]; then
        warn "Virtual environment already exists, reusing..."
    else
        python3 -m venv "$VENV_DIR"
        success "Virtual environment created"
    fi
}

activate_venv() {
    source "$VENV_DIR/bin/activate"
    log "Activated virtual environment"
}

install_deps() {
    log "Installing dependencies..."
    pip install --upgrade pip setuptools wheel
    pip install build twine pytest pytest-asyncio websockets
    success "Dependencies installed"
}

build_package() {
    log "Building package..."
    mkdir -p "$BUILD_DIR"
    python -m build --outdir "$BUILD_DIR/dist"
    success "Package built to $BUILD_DIR/dist/"
    ls -lh "$BUILD_DIR/dist/"
}

test_source() {
    log "Running tests from source..."
    pip install -e ".[dev]"
    pytest tests/ -v --tb=short -x
    success "Tests passed"
}

test_installed() {
    log "Testing installed package..."
    pip install "$BUILD_DIR/dist/"*.whl
    python -c "from agent_ros_bridge import Bridge; print('✓ Import successful')"
    agent-ros-bridge --version
    success "Installed package works"
}

test_cli() {
    log "Testing CLI commands..."
    agent-ros-bridge --help > /dev/null
    agent-ros-bridge --version
    success "CLI working"
}

full_test() {
    log "Starting full build and test..."
    echo ""
    
    create_venv
    activate_venv
    install_deps
    test_source
    build_package
    test_installed
    test_cli
    
    echo ""
    success "ALL TESTS PASSED!"
    echo ""
    log "Build artifacts: $BUILD_DIR/dist/"
    log "Virtual environment: $VENV_DIR"
}

clean_all() {
    log "Cleaning all artifacts..."
    echo ""
    
    # Clean source directory
    log "Cleaning source directory..."
    make clean 2>/dev/null || true
    find . -type f -name ".DS_Store" -delete 2>/dev/null || true
    success "Source directory cleaned"
    
    # Remove virtual environment
    if [ -d "$VENV_DIR" ]; then
        log "Removing virtual environment..."
        rm -rf "$VENV_DIR"
        success "Removed: $VENV_DIR"
    fi
    
    # Remove build artifacts
    if [ -d "$BUILD_DIR" ]; then
        log "Removing build artifacts..."
        rm -rf "$BUILD_DIR"
        success "Removed: $BUILD_DIR"
    fi
    
    # Clean pip cache
    pip cache purge 2>/dev/null || true
    
    # Verify clean
    echo ""
    log "Verifying clean state..."
    if [ -d "$VENV_DIR" ] || [ -d "$BUILD_DIR" ]; then
        error "Some artifacts remain!"
        exit 1
    fi
    
    success "ALL ARTIFACTS CLEANED! Ready for fresh start."
    echo ""
    echo "To start fresh:"
    echo "  ./build-and-test.sh all"
}

print_usage() {
    echo "Agent ROS Bridge - Build and Test Script"
    echo ""
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  test       - Run tests from source code"
    echo "  build      - Build package (wheel + sdist)"
    echo "  install    - Install and test built package"
    echo "  clean      - Remove ALL artifacts for fresh start"
    echo "  all        - Full workflow: test → build → install → verify"
    echo ""
    echo "Artifacts are stored OUTSIDE source directory:"
    echo "  - Virtual env: $VENV_DIR"
    echo "  - Build files: $BUILD_DIR"
}

case "${1:-all}" in
    test)
        create_venv
        activate_venv
        install_deps
        test_source
        ;;
    build)
        create_venv
        activate_venv
        install_deps
        build_package
        ;;
    install)
        activate_venv
        test_installed
        test_cli
        ;;
    clean)
        clean_all
        ;;
    all)
        full_test
        ;;
    help|--help|-h)
        print_usage
        ;;
    *)
        error "Unknown command: $1"
        print_usage
        exit 1
        ;;
esac
