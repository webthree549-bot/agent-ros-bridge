#!/bin/bash
# Start Gazebo with TurtleBot3 in Docker
# Run this script in a terminal with X11 support

echo "Starting TurtleBot3 Gazebo Simulation..."
echo "========================================"

# Check if container is running
if ! docker ps | grep -q ros2_humble; then
    echo "Starting ROS2 container..."
    docker start ros2_humble
    sleep 3
fi

# For macOS, you need XQuartz for GUI applications
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo ""
    echo "Note: On macOS, Gazebo requires X11 forwarding:"
    echo "  1. Install XQuartz: brew install xquartz"
    echo "  2. Start XQuartz: open -a XQuartz"
    echo "  3. Enable X11: xhost +localhost"
    echo ""
    
    # Check if XQuartz is running
    if ! pgrep -x "XQuartz" > /dev/null; then
        echo "⚠️  XQuartz is not running. Starting it now..."
        open -a XQuartz
        sleep 3
        xhost +localhost
    fi
    
    # Run Gazebo with X11 forwarding
    docker exec -it \
        -e DISPLAY=host.docker.internal:0 \
        -e TURTLEBOT3_MODEL=burger \
        ros2_humble \
        bash -c "source /opt/ros/humble/setup.bash && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
else
    # Linux
    docker exec -it \
        -e DISPLAY=$DISPLAY \
        -e TURTLEBOT3_MODEL=burger \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        ros2_humble \
        bash -c "source /opt/ros/humble/setup.bash && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
fi
