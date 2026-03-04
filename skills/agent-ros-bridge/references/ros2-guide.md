# ROS2 Guide

Guide for using Agent ROS Bridge with ROS2 (Jazzy/Humble).

## Prerequisites

- ROS2 Jazzy or Humble installed
- Agent ROS Bridge with ROS2 connector enabled

## Setup

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash  # or humble

# Start Agent ROS Bridge with ROS2
export JWT_SECRET=$(openssl rand -base64 32)
agent-ros-bridge --config config/ros2.yaml
```

## ROS2-Specific Commands

Agent ROS Bridge uses `ros2_` prefix for ROS2 commands:

### Topics

```python
# Publish to a ROS2 topic
{
    "command": {
        "action": "ros2_publish",
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
        "action": "ros2_subscribe_once",
        "parameters": {
            "topic": "/odom",
            "timeout": 5.0
        }
    }
}
```

### Services

```python
# Call a ROS2 service
{
    "command": {
        "action": "ros2_service_call",
        "parameters": {
            "service": "/spawn_entity",
            "service_type": "gazebo_msgs/SpawnEntity",
            "request": {
                "name": "robot",
                "xml": "...",
                "robot_namespace": "",
                "initial_pose": {...}
            }
        }
    }
}
```

### Actions

```python
# Send an action goal
{
    "command": {
        "action": "ros2_action_goal",
        "parameters": {
            "action_name": "/navigate_to_pose",
            "action_type": "nav2_msgs/NavigateToPose",
            "goal": {
                "pose": {
                    "header": {"frame_id": "map"},
                    "pose": {
                        "position": {"x": 1.0, "y": 2.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                    }
                }
            }
        }
    }
}
```

## Common ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/odom` | `nav_msgs/Odometry` | Odometry data |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR data |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree |
| `/tf_static` | `tf2_msgs/TFMessage` | Static transforms |
| `/joint_states` | `sensor_msgs/JointState` | Joint positions |
| `/camera/image_raw` | `sensor_msgs/Image` | Camera feed |
| `/battery_state` | `sensor_msgs/BatteryState` | Battery info |

## Building ROS2 Packages

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone packages
git clone <package-repo>

# Build
cd ~/ros2_ws
colcon build --symlink-install

# Source
source install/setup.bash
```

## Differences from ROS1

- Uses `rclpy` instead of `rospy`
- No `roscore` needed (discovery via DDS)
- Lifecycle nodes supported
- Actions are first-class
- Different build system (colcon vs catkin)
