#!/bin/bash
# install.sh - Agent ROS Bridge Installation Script

set -e

echo "🤖 Installing Agent ROS Bridge..."
echo ""

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log() { echo -e "${BLUE}[INSTALL]${NC} $1"; }
success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; }

check_python() {
    log "Checking Python version..."
    
    if ! command -v python3 &> /dev/null; then
        error "Python 3 is not installed"
        exit 1
    fi
    
    PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
    log "Found Python $PYTHON_VERSION"
    
    REQUIRED_VERSION="3.8"
    if [ "$(printf '%s\n' "$REQUIRED_VERSION" "$PYTHON_VERSION" | sort -V | head -n1)" != "$REQUIRED_VERSION" ]; then
        error "Python 3.8 or higher is required"
        exit 1
    fi
    
    success "Python version OK"
}

check_docker() {
    log "Checking Docker..."
    
    if ! command -v docker &> /dev/null; then
        warn "Docker is not installed"
    else
        if docker info &> /dev/null; then
            success "Docker is running"
        else
            warn "Docker is installed but not running"
        fi
    fi
}

install_package() {
    log "Installing Python package..."
    
    pip3 install --upgrade pip
    
    if pip3 install agent-ros-bridge 2>/dev/null; then
        success "Installed from PyPI"
    else
        warn "PyPI install failed, installing from source..."
        pip3 install -e ".[all]"
        success "Installed from source"
    fi
}

setup_config() {
    log "Setting up configuration..."
    
    CONFIG_DIR="$HOME/.config/agent-ros-bridge"
    mkdir -p "$CONFIG_DIR"
    
    if [ ! -f "$CONFIG_DIR/bridge.yaml" ]; then
        cat > "$CONFIG_DIR/bridge.yaml" << 'EOF'
name: "agent_ros_bridge"
log_level: INFO

transports:
  websocket:
    enabled: true
    host: 0.0.0.0
    port: 8765
  
  grpc:
    enabled: true
    host: 0.0.0.0
    port: 50051

plugins:
  - name: greenhouse
    enabled: true

discovery:
  enabled: true
  methods: ["mdns", "ros2"]
EOF
        success "Created default configuration"
    fi
    
    mkdir -p "$HOME/.local/share/agent-ros-bridge/logs"
}

create_wrappers() {
    log "Creating command wrappers..."
    
    BIN_DIR="$HOME/.local/bin"
    mkdir -p "$BIN_DIR"
    
    cat > "$BIN_DIR/agent-ros-bridge-helper" << 'EOF'
#!/bin/bash
case "$1" in
    start)
        echo "🤖 Starting Agent ROS Bridge..."
        agent-ros-bridge --config "$HOME/.config/agent-ros-bridge/bridge.yaml"
        ;;
    demo)
        echo "🌱 Starting demo mode..."
        agent-ros-bridge --demo
        ;;
    docker)
        echo "🐳 Starting with Docker..."
        docker-compose up -d
        ;;
    status)
        curl -s http://localhost:8765/health 2>/dev/null || echo "Bridge not running"
        ;;
    stop)
        echo "🛑 Stopping bridge..."
        pkill -f "agent-ros-bridge" || true
        docker-compose down 2>/dev/null || true
        ;;
    help|--help|-h)
        echo "Agent ROS Bridge Commands:"
        echo "  start    - Start the bridge"
        echo "  demo     - Start demo mode"
        echo "  docker   - Start with Docker"
        echo "  status   - Check status"
        echo "  stop     - Stop the bridge"
        echo "  help     - Show this help"
        ;;
    *)
        echo "Unknown command: $1"
        exit 1
        ;;
esac
EOF
    
    chmod +x "$BIN_DIR/agent-ros-bridge-helper"
    
    if [[ ":$PATH:" != *":$BIN_DIR:"* ]]; then
        echo "export PATH=\"$BIN_DIR:\$PATH\"" >> "$HOME/.bashrc"
        warn "Added $BIN_DIR to PATH. Restart your shell or run:"
        echo "  export PATH=\"$BIN_DIR:\$PATH\""
    fi
    
    success "Created command wrapper"
}

print_usage() {
    echo ""
    echo "🎉 Installation Complete!"
    echo ""
    echo "Quick Start:"
    echo "  agent-ros-bridge --demo    # Start demo mode"
    echo "  agent-ros-bridge --help    # Show all options"
    echo ""
    echo "Documentation:"
    echo "  https://agent-ros-bridge.readthedocs.io"
    echo ""
}

main() {
    echo "========================================"
    echo "  Agent ROS Bridge Installer"
    echo "========================================"
    echo ""
    
    check_python
    check_docker
    install_package
    setup_config
    create_wrappers
    print_usage
    
    success "Installation complete!"
}

main "$@"
