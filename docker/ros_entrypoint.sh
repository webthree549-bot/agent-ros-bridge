#!/bin/bash
set -e

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Set environment
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/opt/ros/humble/share/turtlebot3_gazebo/models

exec "$@"
