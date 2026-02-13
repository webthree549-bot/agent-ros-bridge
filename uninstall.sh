#!/bin/bash
# uninstall.sh - Agent ROS Bridge Uninstallation Script

set -e

echo "🤖 Uninstalling Agent ROS Bridge..."
echo ""

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log() { echo -e "${BLUE}[UNINSTALL]${NC} $1"; }
success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }

stop_services() {
    log "Stopping services..."
    pkill -f "agent-ros-bridge" 2>/dev/null || true
    docker-compose down 2>/dev/null || true
    success "Services stopped"
}

uninstall_package() {
    log "Uninstalling Python package..."
    pip3 uninstall -y agent-ros-bridge 2>/dev/null || true
    success "Package uninstalled"
}

remove_config() {
    log "Removing configuration..."
    CONFIG_DIR="$HOME/.config/agent-ros-bridge"
    DATA_DIR="$HOME/.local/share/agent-ros-bridge"
    
    if [ -d "$CONFIG_DIR" ]; then
        read -p "Remove configuration directory? ($CONFIG_DIR) [y/N] " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$CONFIG_DIR"
            success "Configuration removed"
        fi
    fi
    
    if [ -d "$DATA_DIR" ]; then
        read -p "Remove data directory? ($DATA_DIR) [y/N] " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$DATA_DIR"
            success "Data removed"
        fi
    fi
}

remove_wrappers() {
    log "Removing wrapper scripts..."
    BIN_DIR="$HOME/.local/bin"
    rm -f "$BIN_DIR/agent-ros-bridge-helper"
    success "Wrappers removed"
}

main() {
    echo "=========================================="
    echo "  Agent ROS Bridge Uninstaller"
    echo "=========================================="
    echo ""
    
    read -p "Are you sure you want to uninstall? [y/N] " -n 1 -r
    echo
    
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Uninstallation cancelled."
        exit 0
    fi
    
    stop_services
    uninstall_package
    remove_wrappers
    remove_config
    
    echo ""
    success "Uninstallation complete!"
}

main "$@"
