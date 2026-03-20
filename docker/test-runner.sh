#!/bin/bash
# Test runner script for ROS2 Docker container
# This script ensures ROS2 environment is available to pytest

# Source ROS2 setup
source /opt/ros/jazzy/setup.bash

# Run pytest with all arguments passed through
cd /workspace
exec python3 -m pytest "$@"
