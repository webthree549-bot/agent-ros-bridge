#!/bin/bash
source /opt/ros/humble/setup.bash

# Publish a bright magenta sphere marker at robot position
ros2 topic pub /marker visualization_msgs/Marker "{
  header: {frame_id: 'base_footprint'},
  ns: 'robot',
  id: 0,
  type: 2,
  action: 0,
  pose: {
    position: {x: 0.0, y: 0.0, z: 0.1},
    orientation: {w: 1.0}
  },
  scale: {x: 0.3, y: 0.3, z: 0.3},
  color: {r: 1.0, g: 0.0, b: 1.0, a: 1.0}
}" -r 10
