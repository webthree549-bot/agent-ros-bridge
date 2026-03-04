#!/bin/bash
# Package Agent ROS Bridge skill for ClawHub

set -e

SKILL_NAME="agent-ros-bridge"
VERSION="0.5.0"
OUTPUT_DIR="dist"

echo "📦 Packaging $SKILL_NAME v$VERSION for ClawHub..."

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Create temporary directory for packaging
TMP_DIR=$(mktemp -d)
trap "rm -rf $TMP_DIR" EXIT

# Copy skill files
cp -r skills/agent-ros-bridge/* "$TMP_DIR/"

# Create skill manifest
cat > "$TMP_DIR/skill.json" << EOF
{
  "name": "$SKILL_NAME",
  "version": "$VERSION",
  "description": "Control ROS1/ROS2 robots via natural language through Agent ROS Bridge",
  "author": "Agent ROS Bridge Team",
  "license": "MIT",
  "homepage": "https://github.com/webthree549-bot/agent-ros-bridge",
  "repository": "https://github.com/webthree549-bot/agent-ros-bridge",
  "keywords": ["ros", "ros2", "robotics", "ai", "agents"],
  "requirements": {
    "openclaw": ">=0.5.0"
  },
  "entry_points": {
    "tools": [
      "ros2_publish",
      "ros2_subscribe",
      "ros2_action",
      "bridge_list_robots",
      "fleet_submit_task"
    ]
  }
}
EOF

# Create the .skill package
SKILL_FILE="$OUTPUT_DIR/${SKILL_NAME}-${VERSION}.skill"
cd "$TMP_DIR"
zip -r "$OLDPWD/$SKILL_FILE" .
cd "$OLDPWD"

echo "✅ Packaged: $SKILL_FILE"
echo ""
echo "📤 To publish to ClawHub:"
echo "   npx clawhub publish $SKILL_FILE"
echo ""
echo "📋 Or upload manually at https://clawhub.ai/upload"
