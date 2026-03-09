#!/bin/bash
# Next Steps Script for Agent ROS Bridge

echo "Agent ROS Bridge - Next Steps"
echo "=============================="
echo ""

# Step 1: Install TurtleBot3 Gazebo (if not already installed)
echo "Step 1: Installing TurtleBot3 Gazebo..."
docker exec ros2_humble bash -c "apt-get update && apt-get install -y ros-humble-turtlebot3-gazebo ros-humble-nav2-bringup"

echo ""
echo "Step 2: Start Gazebo with TurtleBot3..."
echo "Run in Terminal 1:"
echo "  docker exec -it ros2_humble bash -c 'source /opt/ros/humble/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py'"
echo ""

echo "Step 3: Start Navigation (after Gazebo is running)..."
echo "Run in Terminal 2:"
echo "  docker exec -it ros2_humble bash -c 'source /opt/ros/humble/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch nav2_bringup navigation_launch.py'"
echo ""

echo "Step 4: Run E2E Tests..."
echo "Run in Terminal 3:"
echo "  cd /Users/webthree/.openclaw/workspace && python3 -m pytest tests/e2e/ -v"
echo ""

echo "Note: Gazebo requires X11 forwarding for GUI."
echo "Install XQuartz: brew install xquartz"
echo "Start XQuartz: open -a XQuartz"
echo "Enable X11: xhost +localhost"
