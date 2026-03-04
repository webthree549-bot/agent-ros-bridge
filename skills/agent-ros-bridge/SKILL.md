---
name: agent-ros-bridge
description: Control ROS1/ROS2 robots via Agent ROS Bridge. Use when users want to control robots, navigate, check sensors, manage robot fleets, or perform any robot-related tasks. Supports both ROS1 (Noetic) and ROS2 (Jazzy/Humble) through WebSocket, MQTT, or gRPC connections.
---

# Agent ROS Bridge

Control robots through Agent ROS Bridge's universal gateway.

## Quick Start

Start Agent ROS Bridge:
```bash
# Set JWT secret
export JWT_SECRET=$(openssl rand -base64 32)

# Start the bridge
agent-ros-bridge --websocket-port 8765
```

Connect via WebSocket:
```python
import asyncio
import websockets
import json

async def control_robot():
    uri = "ws://localhost:8765?token=<JWT_TOKEN>"
    async with websockets.connect(uri) as ws:
        # List robots
        await ws.send(json.dumps({
            "command": {"action": "list_robots"}
        }))
        response = await ws.recv()
        print(json.loads(response))

asyncio.run(control_robot())
```

## Common Tasks

### Movement
- **Move forward**: Publish to `/cmd_vel`
- **Rotate**: Publish angular velocity
- **Navigate**: Use Nav2 action server

### Sensors
- **Camera**: Subscribe to `/camera/image_raw`
- **LiDAR**: Subscribe to `/scan`
- **Odometry**: Subscribe to `/odom`

### Fleet
- **List robots**: `list_robots` command
- **Submit task**: `submit_task` command
- **Get metrics**: `fleet_get_metrics` command

### Safety
- **Emergency stop**: `safety_trigger_estop` command
- **Release e-stop**: `safety_release_estop` command

## ROS1 vs ROS2

Agent ROS Bridge supports both:
- **ROS1**: Use `ros1_` prefixed commands
- **ROS2**: Use `ros2_` prefixed commands

See references/ros1-guide.md and references/ros2-guide.md for details.
