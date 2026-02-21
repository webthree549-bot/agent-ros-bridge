# ROS2 Actions Demo

Demonstrates ROS Actions for long-running tasks using Agent ROS Bridge.

## Running

```bash
export JWT_SECRET=$(openssl rand -base64 32)
docker-compose up
```

## Features

- Navigation actions (navigate_to_pose)
- Manipulation actions (follow_joint_trajectory)
- Action feedback and results
- Cancel/preempt operations

## WebSocket API

```json
{
  "command": {
    "action": "actions.navigate",
    "parameters": {"x": 5.0, "y": 3.0, "theta": 1.57}
  }
}
```
