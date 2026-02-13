#!/bin/bash
# uninstall.sh - ClawHub Skill Uninstallation Script

set -e

echo "🤖 Uninstalling OpenClaw ROS Bridge Skill..."
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log() { echo -e "${BLUE}[UNINSTALL]${NC} $1"; }
success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }

# Stop running services
stop_services() {
    log "Stopping services..."
    
    # Stop gateway if running
    pkill -f "openclaw-gateway" 2>/dev/null || true
    
    # Stop Docker containers
    if command -v docker-compose &> /dev/null; then
        docker-compose down 2>/dev/null || true
    fi
    
    success "Services stopped"
}

# Uninstall Python package
uninstall_package() {
    log "Uninstalling Python package..."
    
    pip3 uninstall -y openclaw-ros-bridge 2>/dev/null || true
    
    success "Package uninstalled"
}

# Remove configuration
remove_config() {
    log "Removing configuration..."
    
    CONFIG_DIR="$HOME/.config/openclaw-ros-bridge"
    DATA_DIR="$HOME/.local/share/openclaw-ros-bridge"
    
    if [ -d "$CONFIG_DIR" ]; then
        read -p "Remove configuration directory? ($CONFIG_DIR) [y/N] " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$CONFIG_DIR"
            success "Configuration removed"
        else
            warn "Configuration preserved at $CONFIG_DIR"
        fi
    fi
    
    if [ -d "$DATA_DIR" ]; then
        read -p "Remove data directory? ($DATA_DIR) [y/N] " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$DATA_DIR"
            success "Data removed"
        else
            warn "Data preserved at $DATA_DIR"
        fi
    fi
}

# Remove wrapper scripts
remove_wrappers() {
    log "Removing wrapper scripts..."
    
    BIN_DIR="$HOME/.local/bin"
    WRAPPER="$BIN_DIR/openclaw-skill-ros-bridge"
    
    if [ -f "$WRAPPER" ]; then
        rm -f "$WRAPPER"
        success "Wrapper removed"
    fi
}

# Main uninstallation
main() {
    echo "=========================================="
    echo "  OpenClaw ROS Bridge Skill Uninstaller"
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
    echo ""
    echo "Note: If you installed from source, you may need to manually"
    echo "      remove the repository directory."
    echo ""
}

main "$@"
