#!/bin/bash
#
# TurtleBot3 Bridge Demo - Direct ROS2 Commands
#
# This script demonstrates how to interact with the TurtleBot3
# simulation using ROS2 commands directly in the Docker container.
# This shows what agent-ros-bridge does under the hood.

set -e

echo "=========================================="
echo "🐢 TurtleBot3 Bridge Demo"
echo "=========================================="
echo ""

# Check if container is running
if ! docker ps | grep -q ros2_humble; then
    echo "❌ Docker container 'ros2_humble' is not running!"
    echo "   Start it with: docker start ros2_humble"
    exit 1
fi

echo "✅ Docker container is running"
echo ""

# Function to run ROS2 commands in container
ros2_cmd() {
    docker exec ros2_humble bash -c "source /opt/ros/humble/setup.bash && $1"
}

echo "📡 Step 1: List available topics"
echo "----------------------------------------"
ros2_cmd "ros2 topic list -t"
echo ""

echo "📊 Step 2: Monitor odometry (position) for 3 seconds"
echo "----------------------------------------"
timeout 3 ros2_cmd "ros2 topic echo /odom --field pose.pose.position" || true
echo ""

echo "📊 Step 3: Check laser scan (single reading)"
echo "----------------------------------------"
ros2_cmd "ros2 topic echo /scan --once | head -20"
echo ""

echo "📊 Step 4: Check IMU data (single reading)"
echo "----------------------------------------"
ros2_cmd "ros2 topic echo /imu --once | head -15"
echo ""

echo "🚀 Step 5: Move the robot forward"
echo "----------------------------------------"
echo "Publishing velocity command: linear.x = 0.2 m/s"
ros2_cmd "ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --once"
echo ""

echo "⏳ Moving for 2 seconds..."
sleep 2

echo "🛑 Step 6: Stop the robot"
echo "----------------------------------------"
ros2_cmd "ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --once"
echo ""

echo "🔄 Step 7: Rotate the robot"
echo "----------------------------------------"
echo "Publishing velocity command: angular.z = 0.5 rad/s"
ros2_cmd "ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}' --once"
echo ""

echo "⏳ Rotating for 2 seconds..."
sleep 2

echo "🛑 Step 8: Stop the robot"
echo "----------------------------------------"
ros2_cmd "ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --once"
echo ""

echo "📊 Step 9: Final position check"
echo "----------------------------------------"
ros2_cmd "ros2 topic echo /odom --field pose.pose.position --once"
echo ""

echo "=========================================="
echo "✅ Demo complete!"
echo "=========================================="
echo ""
echo "Summary of what agent-ros-bridge does:"
echo "  • Connects to ROS2 topics (like /odom, /scan, /cmd_vel)"
echo "  • Subscribes to sensor data (position, laser, IMU)"
echo "  • Publishes commands (velocity, actions)"
echo "  • Provides a unified API for ROS1 and ROS2"
echo "  • Handles message type conversion automatically"
