# ROS1 Guide

Guide for using Agent ROS Bridge with ROS1 (Noetic).

## Prerequisites

- ROS1 Noetic installed
- `roscore` running
- Agent ROS Bridge with ROS1 connector enabled

## Setup

```bash
# Source ROS1
source /opt/ros/noetic/setup.bash

# Start roscore
roscore

# In another terminal, start Agent ROS Bridge with ROS1
export JWT_SECRET=$(openssl rand -base64 32)
agent-ros-bridge --config config/ros1.yaml
```

## ROS1-Specific Commands

Agent ROS Bridge uses `ros1_` prefix for ROS1 commands:

### Topics

```python
# Publish to a ROS1 topic
{
    "command": {
        "action": "ros1_publish",
        "parameters": {
            "topic": "/cmd_vel",
            "message_type": "geometry_msgs/Twist",
            "message": {
                "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
            }
        }
    }
}

# Subscribe to a topic
{
    "command": {
        "action": "ros1_subscribe_once",
        "parameters": {
            "topic": "/odom"
        }
    }
}
```

### Services

```python
# Call a ROS1 service
{
    "command": {
        "action": "ros1_service_call",
        "parameters": {
            "service": "/spawn",
            "request": {
                "x": 5.0,
                "y": 5.0,
                "theta": 0.0,
                "name": "turtle2"
            }
        }
    }
}
```

## Common ROS1 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/odom` | `nav_msgs/Odometry` | Odometry data |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR data |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree |
| `/joint_states` | `sensor_msgs/JointState` | Joint positions |

## Differences from ROS2

- Uses `rospy` instead of `rclpy`
- Requires `roscore` to be running
- Different message serialization
- No lifecycle nodes
