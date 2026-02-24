#!/bin/bash
# Native ROS Installation Script for Agent ROS Bridge
# Supports Ubuntu 20.04/22.04/24.04 with ROS1 Noetic or ROS2 Humble/Jazzy

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "=========================================="
echo "🤖 Agent ROS Bridge - Native Installer"
echo "=========================================="

# Detect OS
if [ -f /etc/os-release ]; then
    . /etc/os-release
    OS=$NAME
    VERSION=$VERSION_ID
else
    echo -e "${RED}❌ Cannot detect OS${NC}"
    exit 1
fi

echo "Detected: $OS $VERSION"

# Check Ubuntu version
if [[ "$VERSION" != "20.04" && "$VERSION" != "22.04" && "$VERSION" != "24.04" ]]; then
    echo -e "${YELLOW}⚠️  Unsupported Ubuntu version. Tested on 20.04, 22.04, 24.04${NC}"
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check if ROS is installed
echo ""
echo "🔍 Checking for ROS installation..."

ROS_VERSION=""
if [ -f /opt/ros/noetic/setup.bash ]; then
    ROS_VERSION="noetic"
    ROS_TYPE="ros1"
    echo -e "${GREEN}✅ Found ROS1 Noetic${NC}"
elif [ -f /opt/ros/humble/setup.bash ]; then
    ROS_VERSION="humble"
    ROS_TYPE="ros2"
    echo -e "${GREEN}✅ Found ROS2 Humble${NC}"
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    ROS_VERSION="jazzy"
    ROS_TYPE="ros2"
    echo -e "${GREEN}✅ Found ROS2 Jazzy${NC}"
else
    echo -e "${YELLOW}⚠️  ROS not found in /opt/ros/${NC}"
    echo "Please install ROS first:"
    echo "  - ROS1 Noetic: http://wiki.ros.org/noetic/Installation/Ubuntu"
    echo "  - ROS2 Humble: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

# Check Python
PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
echo "Python version: $PYTHON_VERSION"

# Install system dependencies
echo ""
echo "📦 Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-venv \
    python3-dev \
    build-essential \
    git \
    curl \
    jq

# Install ROS Python packages
if [ "$ROS_TYPE" = "ros2" ]; then
    echo "Installing ROS2 Python packages..."
    sudo apt-get install -y python3-rclpy || true
else
    echo "Installing ROS1 Python packages..."
    sudo apt-get install -y python3-rospy || true
fi

# Create virtual environment
echo ""
echo "🐍 Creating Python virtual environment..."
VENV_PATH="${HOME}/agent-ros-bridge-venv"

if [ -d "$VENV_PATH" ]; then
    echo -e "${YELLOW}⚠️  Virtual environment already exists at $VENV_PATH${NC}"
    read -p "Remove and recreate? (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$VENV_PATH"
        python3 -m venv "$VENV_PATH"
    fi
else
    python3 -m venv "$VENV_PATH"
fi

source "$VENV_PATH/bin/activate"

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install agent-ros-bridge
echo ""
echo "📥 Installing Agent ROS Bridge..."

read -p "Install from PyPI or source? (pypi/source) [pypi]: " -r
INSTALL_METHOD=${REPLY:-pypi}

if [ "$INSTALL_METHOD" = "source" ]; then
    echo "Installing from source..."
    
    # Clone if not already in repo
    if [ ! -f "pyproject.toml" ]; then
        git clone https://github.com/webthree549-bot/agent-ros-bridge.git
        cd agent-ros-bridge
    fi
    
    pip install -e ".[dev]"
else
    echo "Installing from PyPI..."
    pip install agent-ros-bridge
fi

# Create convenience scripts
echo ""
echo "📝 Creating convenience scripts..."

# Start script
cat > "${HOME}/start-agent-bridge.sh" << 'EOF'
#!/bin/bash
source ${HOME}/agent-ros-bridge-venv/bin/activate

# Auto-detect ROS
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Run bridge
agent-ros-bridge "$@"
EOF

chmod +x "${HOME}/start-agent-bridge.sh"

# Dashboard script
cat > "${HOME}/start-agent-dashboard.sh" << 'EOF'
#!/bin/bash
source ${HOME}/agent-ros-bridge-venv/bin/activate
# Dashboard is accessible once the bridge starts: http://localhost:8080
agent-ros-bridge "$@"
EOF

chmod +x "${HOME}/start-agent-dashboard.sh"

# Create systemd service file (optional)
echo ""
read -p "Create systemd service? (requires sudo) (y/N) " -n 1 -r
echo

if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo tee /etc/systemd/system/agent-ros-bridge.service > /dev/null << EOF
[Unit]
Description=Agent ROS Bridge
After=network.target

[Service]
Type=simple
User=$USER
Environment="PATH=${VENV_PATH}/bin:/usr/local/bin:/usr/bin:/bin"
Environment="ROS_DISTRO=${ROS_VERSION}"
ExecStart=${VENV_PATH}/bin/python ${HOME}/start-agent-bridge.sh
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reload
    echo -e "${GREEN}✅ Systemd service created${NC}"
    echo "Start with: sudo systemctl start agent-ros-bridge"
    echo "Enable on boot: sudo systemctl enable agent-ros-bridge"
fi

# Print summary
echo ""
echo "=========================================="
echo -e "${GREEN}✅ Installation Complete!${NC}"
echo "=========================================="
echo ""
echo "ROS Version: $ROS_VERSION"
echo "Virtual Environment: $VENV_PATH"
echo ""
echo "Quick Start:"
echo "  1. Activate venv: source ${VENV_PATH}/bin/activate"
echo "  2. Source ROS: source /opt/ros/${ROS_VERSION}/setup.bash"
echo "  3. Run bridge: agent-ros-bridge"
echo "  4. Dashboard starts automatically on http://localhost:8080"
echo ""
echo "Convenience scripts:"
echo "  ${HOME}/start-agent-bridge.sh"
echo "  ${HOME}/start-agent-dashboard.sh"
echo ""
echo "Documentation:"
echo "  docs/NATIVE_ROS.md"
echo ""
echo "Test your installation:"
echo "  pytest tests/integration/ -v"
echo ""
echo "=========================================="
