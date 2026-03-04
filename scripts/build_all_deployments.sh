#!/bin/bash
# Build all deployment artifacts for Agent ROS Bridge

set -e

VERSION="0.5.0"
DIST_DIR="dist"

echo "🔨 Building Agent ROS Bridge v$VERSION deployment artifacts..."
echo ""

# Create dist directory
mkdir -p "$DIST_DIR"

# =============================================================================
# 1. PyPI Package
# =============================================================================
echo "📦 Building PyPI package..."
python -m pip install --upgrade build twine
python -m build
mv dist/*.whl "$DIST_DIR/" 2>/dev/null || true
mv dist/*.tar.gz "$DIST_DIR/" 2>/dev/null || true
echo "✅ PyPI package built"
echo ""

# =============================================================================
# 2. Docker Images
# =============================================================================
echo "🐳 Building Docker images..."

# Build multi-arch images
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --target production \
  -t agentrosbridge/agent-ros-bridge:$VERSION \
  -t agentrosbridge/agent-ros-bridge:latest \
  --push \
  .

echo "✅ Docker images built and pushed"
echo ""

# =============================================================================
# 3. Helm Chart
# =============================================================================
echo "☸️  Packaging Helm chart..."
cd helm/agent-ros-bridge
helm package . -d ../../$DIST_DIR/
cd ../..
echo "✅ Helm chart packaged"
echo ""

# =============================================================================
# 4. ClawHub Skill
# =============================================================================
echo "🎯 Packaging ClawHub skill..."
./scripts/package_clawhub.sh
echo "✅ ClawHub skill packaged"
echo ""

# =============================================================================
# 5. APT Package (Debian/Ubuntu)
# =============================================================================
echo "📥 Building APT package..."

# Create package structure
mkdir -p packaging/deb/agent-ros-bridge/usr/share/agent-ros-bridge
cp -r agent_ros_bridge packaging/deb/agent-ros-bridge/usr/share/agent-ros-bridge/
cp -r skills packaging/deb/agent-ros-bridge/usr/share/agent-ros-bridge/
cp pyproject.toml packaging/deb/agent-ros-bridge/usr/share/agent-ros-bridge/
cp requirements.txt packaging/deb/agent-ros-bridge/usr/share/agent-ros-bridge/

# Create systemd service
mkdir -p packaging/deb/agent-ros-bridge/lib/systemd/system
cat > packaging/deb/agent-ros-bridge/lib/systemd/system/agent-ros-bridge.service << 'EOF'
[Unit]
Description=Agent ROS Bridge
After=network.target

[Service]
Type=simple
User=bridge
Group=bridge
WorkingDirectory=/var/lib/agent-ros-bridge
Environment=PYTHONPATH=/usr/share/agent-ros-bridge
Environment=JWT_SECRET_FILE=/etc/agent-ros-bridge/jwt-secret
ExecStart=/usr/bin/python3 -m agent_ros_bridge.cli start
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# Build .deb package
cd packaging/deb
dpkg-deb --build agent-ros-bridge
mv agent-ros-bridge.deb ../../$DIST_DIR/agent-ros-bridge_${VERSION}_amd64.deb
cd ../..

echo "✅ APT package built"
echo ""

# =============================================================================
# Summary
# =============================================================================
echo "🎉 All deployment artifacts built successfully!"
echo ""
echo "📁 Artifacts in $DIST_DIR/:"
ls -lh "$DIST_DIR/"
echo ""
echo "📤 Deployment instructions:"
echo ""
echo "1. PyPI (Python developers):"
echo "   twine upload $DIST_DIR/*.whl"
echo ""
echo "2. Docker Hub (container users):"
echo "   Already pushed to agentrosbridge/agent-ros-bridge:$VERSION"
echo ""
echo "3. Helm (Kubernetes):"
echo "   helm push $DIST_DIR/agent-ros-bridge-$VERSION.tgz oci://ghcr.io/webthree549-bot/charts"
echo ""
echo "4. ClawHub (OpenClaw users):"
echo "   npx clawhub publish $DIST_DIR/agent-ros-bridge-$VERSION.skill"
echo ""
echo "5. APT (Ubuntu/Debian):"
echo "   Upload $DIST_DIR/agent-ros-bridge_${VERSION}_amd64.deb to APT repository"
