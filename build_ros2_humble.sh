#!/bin/bash
#
# ROS2 Humble Build Script for macOS (Apple Silicon)
# Target: /Volumes/2nd-HD/ros2_humble
#

set -e

# Configuration
ROS2_VERSION="humble"
INSTALL_DIR="/Volumes/2nd-HD/ros2_humble"
WORKSPACE_DIR="$INSTALL_DIR/ros2_ws"
SRC_DIR="$WORKSPACE_DIR/src"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}ROS2 Humble Build Script for macOS${NC}"
echo "======================================"
echo "Install directory: $INSTALL_DIR"
echo "Workspace: $WORKSPACE_DIR"
echo ""

# Check if running on Apple Silicon
if [[ $(uname -m) != "arm64" ]]; then
    echo -e "${RED}Error: This script is for Apple Silicon (arm64) only${NC}"
    exit 1
fi

# Check macOS version
MACOS_VERSION=$(sw_vers -productVersion | cut -d. -f1)
if [[ $MACOS_VERSION -lt 12 ]]; then
    echo -e "${RED}Error: macOS 12 (Monterey) or later required${NC}"
    exit 1
fi

echo -e "${YELLOW}Step 1: Installing dependencies via Homebrew...${NC}"

# Install Homebrew if not present
if ! command -v brew &> /dev/null; then
    echo "Installing Homebrew..."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
fi

# Install required packages
brew install cmake git wget python@3.10

# Install ROS2 dependencies (tinyxml deprecated, skip it)
brew install asio tinyxml2 eigen pcre poco openssl
brew install qt@5 freetype assimp bullet
brew install ffmpeg opencv

# Note: tinyxml is deprecated, ROS2 Humble uses tinyxml2

echo -e "${GREEN}✓ Dependencies installed${NC}"

echo -e "${YELLOW}Step 2: Setting up Python environment...${NC}"

# Use Python 3.10 (recommended for ROS2 Humble)
PYTHON_EXE=$(brew --prefix python@3.10)/bin/python3.10

# Create virtual environment
if [ ! -d "$INSTALL_DIR/venv" ]; then
    $PYTHON_EXE -m venv "$INSTALL_DIR/venv"
fi

source "$INSTALL_DIR/venv/bin/activate"

# Upgrade pip and install base packages
pip install --upgrade pip setuptools wheel
pip install colcon-common-extensions colcon-mixin
pip install rosdep vcstool

echo -e "${GREEN}✓ Python environment ready${NC}"

echo -e "${YELLOW}Step 3: Creating workspace...${NC}"

# Create directories
mkdir -p "$SRC_DIR"
cd "$WORKSPACE_DIR"

echo -e "${GREEN}✓ Workspace created${NC}"

echo -e "${YELLOW}Step 4: Downloading ROS2 source code...${NC}"

# Download ROS2 repos file
wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos -O ros2.repos

# Import repositories
vcs import "$SRC_DIR" < ros2.repos

echo -e "${GREEN}✓ Source code downloaded${NC}"

echo -e "${YELLOW}Step 5: Installing Python dependencies...${NC}"

# Install Python dependencies from ROS packages
pip install -U \
    argcomplete \
    catkin_pkg \
    coverage \
    cryptography \
    empy \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    importlib-metadata \
    lark \
    lxml \
    mock \
    mypy \
    netifaces \
    nose \
    numpy \
    pep8 \
    pydocstyle \
    pydot \
    pyyaml \
    rcutils \
    rosdep \
    rosidl-parser \
    setuptools \
    vcstool \
    wheel

echo -e "${GREEN}✓ Python dependencies installed${NC}"

echo -e "${YELLOW}Step 6: Building ROS2...${NC}"
echo "This will take 1-3 hours depending on your system..."
echo ""

# Build ROS2
colcon build \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_OSX_ARCHITECTURES=arm64 \
        -DBUILD_TESTING=OFF \
        -DINSTALL_EXAMPLES=OFF \
    --packages-up-to \
        ros_base \
        examples_rclcpp_minimal_publisher \
        examples_rclcpp_minimal_subscriber

echo -e "${GREEN}✓ Build complete${NC}"

echo -e "${YELLOW}Step 7: Creating setup script...${NC}"

# Create setup script
cat > "$INSTALL_DIR/setup.bash" << 'EOF'
#!/bin/bash
# ROS2 Humble setup for macOS

INSTALL_DIR="/Volumes/2nd-HD/ros2_humble"

# Source virtual environment
source "$INSTALL_DIR/venv/bin/activate"

# Source ROS2
source "$INSTALL_DIR/ros2_ws/install/setup.bash"

# Set environment variables
export ROS_DISTRO=humble
export ROS_PYTHON_VERSION=3

echo "ROS2 Humble environment loaded"
echo "Run 'ros2 --help' to get started"
EOF

chmod +x "$INSTALL_DIR/setup.bash"

echo -e "${GREEN}✓ Setup script created${NC}"

echo ""
echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}ROS2 Humble build complete!${NC}"
echo -e "${GREEN}======================================${NC}"
echo ""
echo "To use ROS2, run:"
echo "  source /Volumes/2nd-HD/ros2_humble/setup.bash"
echo ""
echo "Test installation:"
echo "  ros2 --help"
echo "  ros2 run demo_nodes_cpp talker"
