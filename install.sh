#!/bin/bash
# install.sh - ClawHub Skill Installation Script
# This script is called by ClawHub when installing the skill

set -e

echo "🤖 Installing OpenClaw ROS Bridge Skill..."
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log() { echo -e "${BLUE}[INSTALL]${NC} $1"; }
success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Check Python version
check_python() {
    log "Checking Python version..."
    
    if ! command -v python3 &> /dev/null; then
        error "Python 3 is not installed"
        echo "Please install Python 3.8 or higher: https://python.org"
        exit 1
    fi
    
    PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
    log "Found Python $PYTHON_VERSION"
    
    # Check minimum version
    REQUIRED_VERSION="3.8"
    if [ "$(printf '%s\n' "$REQUIRED_VERSION" "$PYTHON_VERSION" | sort -V | head -n1)" != "$REQUIRED_VERSION" ]; then
        error "Python 3.8 or higher is required (found $PYTHON_VERSION)"
        exit 1
    fi
    
    success "Python version OK"
}

# Check Docker
check_docker() {
    log "Checking Docker..."
    
    if ! command -v docker &> /dev/null; then
        warn "Docker is not installed"
        echo "For full functionality, install Docker: https://docker.com"
        echo "You can still use the skill in mock mode without Docker."
    else
        if docker info &> /dev/null; then
            success "Docker is running"
        else
            warn "Docker is installed but not running"
        fi
    fi
}

# Install Python package
install_package() {
    log "Installing Python package..."
    
    pip3 install --upgrade pip
    
    # Try to install from PyPI first
    if pip3 install openclaw-ros-bridge 2>/dev/null; then
        success "Installed from PyPI"
    else
        warn "PyPI install failed, installing from source..."
        pip3 install -e ".[all]"
        success "Installed from source"
    fi
}

# Setup configuration
setup_config() {
    log "Setting up configuration..."
    
    CONFIG_DIR="$HOME/.config/openclaw-ros-bridge"
    mkdir -p "$CONFIG_DIR"
    
    if [ ! -f "$CONFIG_DIR/gateway.yaml" ]; then
        cat > "$CONFIG_DIR/gateway.yaml" << 'EOF'
name: "openclaw_gateway"
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
        success "Created default configuration at $CONFIG_DIR/gateway.yaml"
    fi
    
    # Create logs directory
    mkdir -p "$HOME/.local/share/openclaw-ros-bridge/logs"
}

# Create wrapper scripts
create_wrappers() {
    log "Creating command wrappers..."
    
    BIN_DIR="$HOME/.local/bin"
    mkdir -p "$BIN_DIR"
    
    # Create skill wrapper
    cat > "$BIN_DIR/openclaw-skill-ros-bridge" << 'EOF'
#!/bin/bash
# Wrapper for OpenClaw ROS Bridge skill

case "$1" in
    start)
        echo "🤖 Starting OpenClaw ROS Bridge Gateway..."
        openclaw-gateway --config "$HOME/.config/openclaw-ros-bridge/gateway.yaml"
        ;;
    demo)
        echo "🌱 Starting demo mode..."
        openclaw-gateway --demo
        ;;
    docker)
        echo "🐳 Starting with Docker..."
        docker-compose up -d
        ;;
    status)
        echo "📊 Checking status..."
        curl -s http://localhost:8765/health 2>/dev/null || echo "Gateway not running"
        ;;
    stop)
        echo "🛑 Stopping gateway..."
        pkill -f "openclaw-gateway" || true
        docker-compose down 2>/dev/null || true
        ;;
    config)
        echo "⚙️  Opening configuration..."
        ${EDITOR:-nano} "$HOME/.config/openclaw-ros-bridge/gateway.yaml"
        ;;
    logs)
        echo "📜 Showing logs..."
        tail -f "$HOME/.local/share/openclaw-ros-bridge/logs/gateway.log"
        ;;
    help|--help|-h)
        echo "OpenClaw ROS Bridge Skill Commands:"
        echo ""
        echo "  start    - Start the gateway"
        echo "  demo     - Start demo mode with greenhouse"
        echo "  docker   - Start with Docker Compose"
        echo "  status   - Check gateway status"
        echo "  stop     - Stop the gateway"
        echo "  config   - Edit configuration"
        echo "  logs     - View logs"
        echo "  help     - Show this help"
        echo ""
        echo "Quick Start:"
        echo "  openclaw-skill-ros-bridge demo"
        ;;
    *)
        echo "Unknown command: $1"
        echo "Run 'openclaw-skill-ros-bridge help' for usage"
        exit 1
        ;;
esac
EOF
    
    chmod +x "$BIN_DIR/openclaw-skill-ros-bridge"
    
    # Check if bin dir is in PATH
    if [[ ":$PATH:" != *":$BIN_DIR:"* ]]; then
        echo "export PATH=\"$BIN_DIR:\$PATH\"" >> "$HOME/.bashrc"
        echo "export PATH=\"$BIN_DIR:\$PATH\"" >> "$HOME/.zshrc" 2>/dev/null || true
        warn "Added $BIN_DIR to PATH. Please restart your shell or run:"
        echo "  export PATH=\"$BIN_DIR:\$PATH\""
    fi
    
    success "Created command wrapper"
}

# Print usage info
print_usage() {
    echo ""
    echo "🎉 Installation Complete!"
    echo ""
    echo "Quick Start:"
    echo "  openclaw-skill-ros-bridge demo    # Start demo mode"
    echo "  openclaw-skill-ros-bridge start   # Start gateway"
    echo "  openclaw-skill-ros-bridge status  # Check status"
    echo "  openclaw-skill-ros-bridge help    # Show all commands"
    echo ""
    echo "Configuration:"
    echo "  Edit: $HOME/.config/openclaw-ros-bridge/gateway.yaml"
    echo "  Logs:  $HOME/.local/share/openclaw-ros-bridge/logs/"
    echo ""
    echo "Documentation:"
    echo "  https://openclaw-ros-bridge.readthedocs.io"
    echo "  https://github.com/webthree549-bot/openclaw-ros-bridge"
    echo ""
}

# Main installation
main() {
    echo "========================================"
    echo "  OpenClaw ROS Bridge Skill Installer"
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
