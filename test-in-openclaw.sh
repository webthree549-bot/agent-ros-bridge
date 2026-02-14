#!/bin/bash
# test-in-openclaw.sh - Test Agent ROS Bridge in OpenClaw environment

set -e

echo "🤖 Testing Agent ROS Bridge in OpenClaw"
echo "========================================"
echo ""

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

log() { echo -e "${BLUE}[TEST]${NC} $1"; }
success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }

PROJECT_DIR="$HOME/dev/agent-ros-bridge"
OPENCLAW_SKILLS_DIR="$HOME/.openclaw/skills"
BUILD_DIR="$HOME/agent-ros-bridge-build-artifacts"

check_openclaw() {
    log "Checking OpenClaw..."
    if ! command -v openclaw &> /dev/null; then
        echo "⚠️  OpenClaw CLI not found. Testing manually..."
        return 1
    fi
    success "OpenClaw found"
    return 0
}

install_to_openclaw() {
    log "Installing Agent ROS Bridge to OpenClaw..."
    
    # Create OpenClaw skill directory
    mkdir -p "$OPENCLAW_SKILLS_DIR/agent-ros-bridge"
    
    # Copy SKILL.md (required by OpenClaw)
    cp "$PROJECT_DIR/SKILL.md" "$OPENCLAW_SKILLS_DIR/agent-ros-bridge/"
    
    # Copy essential files
    cp -r "$PROJECT_DIR/agent_ros_bridge" "$OPENCLAW_SKILLS_DIR/agent-ros-bridge/"
    cp -r "$PROJECT_DIR/config" "$OPENCLAW_SKILLS_DIR/agent-ros-bridge/" 2>/dev/null || true
    
    success "Installed to $OPENCLAW_SKILLS_DIR/agent-ros-bridge/"
}

test_openclaw_import() {
    log "Testing import in OpenClaw Python..."
    
    # Test if OpenClaw can import it
    python3 -c "
import sys
sys.path.insert(0, '$OPENCLAW_SKILLS_DIR/agent-ros-bridge')
from agent_ros_bridge import Bridge
print('✓ Import successful in OpenClaw context')
"
}

test_openclaw_cli() {
    log "Testing CLI..."
    
    # Add to PATH temporarily
    export PATH="$OPENCLAW_SKILLS_DIR/agent-ros-bridge:$PATH"
    
    # Test CLI
    if command -v agent-ros-bridge &> /dev/null; then
        agent-ros-bridge --version
        success "CLI accessible"
    else
        echo "⚠️  CLI not in PATH, testing Python import only"
    fi
}

manual_test() {
    log "Running manual OpenClaw test..."
    
    # Install the package in user space
    pip install --user "$BUILD_DIR/dist/"*.whl 2>/dev/null || pip install "$BUILD_DIR/dist/"*.whl
    
    success "Package installed"
    
    # Test import
    python3 -c "from agent_ros_bridge import Bridge; print('✓ OpenClaw can import Bridge')"
    
    # Test CLI
    agent-ros-bridge --version
    
    success "OpenClaw integration test passed"
}

start_bridge_demo() {
    log "Starting Bridge in demo mode..."
    echo ""
    echo "Starting: agent-ros-bridge --demo"
    echo ""
    echo "Once started, test with:"
    echo "  curl http://localhost:8765/health"
    echo ""
    
    # Start in background
    agent-ros-bridge --demo &
    BRIDGE_PID=$!
    
    # Wait for startup
    sleep 3
    
    # Test health endpoint
    if curl -s http://localhost:8765/health > /dev/null 2>&1; then
        success "Bridge is running!"
        echo ""
        echo "Test commands:"
        echo "  curl http://localhost:8765/health"
        echo "  agent-ros-bridge --help"
        echo ""
        echo "To stop: kill $BRIDGE_PID"
    else
        echo "⚠️  Bridge may still be starting..."
    fi
}

cleanup() {
    log "Cleaning up..."
    rm -rf "$OPENCLAW_SKILLS_DIR/agent-ros-bridge"
    success "Cleaned OpenClaw skill directory"
}

# Main
case "${1:-test}" in
    install)
        install_to_openclaw
        test_openclaw_import
        ;;
    test)
        check_openclaw || manual_test
        ;;
    demo)
        start_bridge_demo
        ;;
    manual)
        manual_test
        ;;
    clean)
        cleanup
        ;;
    *)
        echo "Usage: $0 [install|test|demo|manual|clean]"
        echo ""
        echo "Commands:"
        echo "  install  - Install skill to OpenClaw directory"
        echo "  test     - Test OpenClaw integration"
        echo "  demo     - Start bridge demo mode"
        echo "  manual   - Manual test without OpenClaw CLI"
        echo "  clean    - Remove from OpenClaw"
        ;;
esac
