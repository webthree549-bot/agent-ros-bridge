#!/bin/bash
# Build script for RoboStack (Conda-based ROS on macOS/Windows/Linux)
# https://robostack.github.io/

set -e

echo "Building Agent ROS Bridge for RoboStack..."

# Setup conda environment
conda create -n ros_env python=3.10 -y
source activate ros_env

# Install RoboStack ROS2
conda install -c conda-forge -c robostack ros-humble-desktop -y

# Install additional ROS2 packages
conda install -c conda-forge -c robostack \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-moveit -y || true

# Install pip dependencies
pip install \
  mcp>=1.0.0 \
  websockets>=10.0 \
  grpcio>=1.50.0 \
  aiohttp>=3.8.0 \
  pyyaml>=6.0

# Install Agent ROS Bridge
cd "$(dirname "$0")/../.."
pip install -e ".[all]"

echo "Build complete!"
echo ""
echo "To activate:"
echo "  conda activate ros_env"
echo "  source /opt/ros/humble/setup.bash"
echo ""
echo "To run:"
echo "  python -m agent_ros_bridge.mcp"
