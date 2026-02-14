#!/bin/bash
# install-openclaw-skill.sh - Manual installation for OpenClaw

set -e

echo "🤖 Installing Agent ROS Bridge for OpenClaw"
echo "============================================"
echo ""

# Configuration
PROJECT_DIR="${PROJECT_DIR:-$HOME/dev/agent-ros-bridge}"
OPENCLAW_DIR="${OPENCLAW_DIR:-$HOME/.openclaw}"
SKILL_NAME="agent-ros-bridge"

# Check if we're in the right place
if [ ! -f "$PROJECT_DIR/SKILL.md" ]; then
    echo "❌ Error: SKILL.md not found in $PROJECT_DIR"
    echo "Please set PROJECT_DIR to your agent-ros-bridge directory"
    exit 1
fi

# Create OpenClaw skill directory
echo "📁 Creating skill directory..."
mkdir -p "$OPENCLAW_DIR/skills/$SKILL_NAME"

# Copy essential files
echo "📦 Copying skill files..."
cp "$PROJECT_DIR/SKILL.md" "$OPENCLAW_DIR/skills/$SKILL_NAME/"
cp -r "$PROJECT_DIR/agent_ros_bridge" "$OPENCLAW_DIR/skills/$SKILL_NAME/"
cp -r "$PROJECT_DIR/config" "$OPENCLAW_DIR/skills/$SKILL_NAME/" 2>/dev/null || true

# Create __init__.py for OpenClaw discovery
if [ ! -f "$OPENCLAW_DIR/skills/$SKILL_NAME/__init__.py" ]; then
    cat > "$OPENCLAW_DIR/skills/$SKILL_NAME/__init__.py" << 'EOF'
"""Agent ROS Bridge - OpenClaw Skill"""
from agent_ros_bridge import Bridge

__all__ = ['Bridge']
EOF
fi

echo ""
echo "✅ Skill installed to: $OPENCLAW_DIR/skills/$SKILL_NAME/"
echo ""
echo "📋 Installation complete!"
echo ""
echo "To use in OpenClaw:"
echo "  from agent_ros_bridge import Bridge"
echo ""
echo "Or install the package:"
echo "  pip install $PROJECT_DIR"
echo ""
