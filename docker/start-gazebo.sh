#!/bin/bash
# start-gazebo.sh - Start Gazebo with proper initialization sequence

source /opt/ros/humble/setup.bash

echo "Starting Gazebo server with ROS plugins..."

# Create a simple world file
cat > /tmp/ros_world.world << 'EOF'
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <physics type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>
    
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
  </world>
</sdf>
EOF

# Start Gazebo with the factory plugin loaded as system plugin
gzserver /tmp/ros_world.world --verbose -s libgazebo_ros_factory.so -s libgazebo_ros_init.so &
GAZEBO_PID=$!

# Wait for Gazebo to be ready
echo "Waiting for Gazebo to initialize (25s)..."
sleep 25

# Check if Gazebo is running
if ! kill -0 $GAZEBO_PID 2>/dev/null; then
    echo "ERROR: Gazebo server failed to start"
    exit 1
fi

echo "Gazebo server is running (PID: $GAZEBO_PID)"

# Start robot state publisher with direct URDF
echo "Starting robot state publisher..."
URDF_FILE="/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf"
ros2 run robot_state_publisher robot_state_publisher "$URDF_FILE" &

sleep 5

# Spawn the robot
echo "Spawning TurtleBot3..."
ros2 run gazebo_ros spawn_entity.py \
    -entity turtlebot3 \
    -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
    -x 0.0 -y 0.0 -z 0.1 || echo "Spawn may have failed, continuing..."

echo "Gazebo simulation ready!"
echo "Active topics:"
ros2 topic list

# Keep container running
tail -f /dev/null
