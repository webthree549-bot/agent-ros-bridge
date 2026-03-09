#!/bin/bash
# Start Gazebo with TurtleBot3 in Docker (Headless Mode)
# This version runs without GUI for testing

echo "Starting TurtleBot3 Gazebo Simulation (Headless)..."
echo "===================================================="

# Check if container is running
if ! docker ps | grep -q ros2_humble; then
    echo "Starting ROS2 container..."
    docker start ros2_humble
    sleep 3
fi

# Run Gazebo in headless mode (no GUI)
# Use gzserver only, no gzclient
docker exec -it \
    -e TURTLEBOT3_MODEL=burger \
    ros2_humble \
    bash -c "
        source /opt/ros/humble/setup.bash
        echo 'Starting Gazebo server...'
        ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false &
        sleep 10
        echo ''
        echo 'Gazebo topics:'
        ros2 topic list
        echo ''
        echo 'Robot state:'
        ros2 topic echo /odom --once 2>/dev/null || echo 'Waiting for robot...'
        bash
    "
