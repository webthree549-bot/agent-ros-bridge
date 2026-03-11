#!/bin/bash
# ROS2 TurtleBot3 + Nav2 + Foxglove Setup Script

source /opt/ros/humble/setup.bash

# Install required packages
echo "Installing packages..."
apt-get update
apt-get install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3-navigation2 \
  ros-humble-turtlebot3-gazebo \
  ros-humble-slam-toolbox \
  ros-humble-foxglove-bridge

# Set environment
export TURTLEBOT3_MODEL=burger

# Launch Gazebo with TurtleBot3 House
echo "Starting Gazebo..."
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py gui:=false &
sleep 10

# Start SLAM Toolbox
echo "Starting SLAM Toolbox..."
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True &
sleep 5

# Start Foxglove Bridge
echo "Starting Foxglove Bridge on port 8765..."
ros2 run foxglove_bridge foxglove_bridge &

echo "Setup complete! Foxglove Studio available at:"
echo "  Web: https://studio.foxglove.dev/"
echo "  Desktop: Download from https://foxglove.dev/download"
echo ""
echo "Connect to: ws://localhost:8765"

# Keep container running
tail -f /dev/null
