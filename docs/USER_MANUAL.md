# Agent ROS Bridge - User Manual

## Table of Contents

1. [Introduction](#introduction)
2. [Installation](#installation)
3. [Quick Start](#quick-start)
4. [Configuration](#configuration)
5. [Using the Bridge](#using-the-bridge)
6. [Working with Robots](#working-with-robots)
7. [Fleet Management](#fleet-management)
8. [Monitoring](#monitoring)
9. [Troubleshooting](#troubleshooting)
10. [Advanced Topics](#advanced-topics)

---

## Introduction

Agent ROS Bridge is a universal gateway that connects AI agents to ROS-based robots. It supports:

- **Multiple robot types**: Mobile robots, arms, IoT sensors
- **Multiple ROS versions**: ROS1 Noetic, ROS2 Humble/Jazzy
- **Multiple protocols**: WebSocket, MQTT, gRPC
- **Mixed fleets**: ROS1 and ROS2 robots together
- **Production features**: Authentication, monitoring, metrics

### Use Cases

- **Warehouse Automation**: Coordinate fleets of AMRs
- **Manufacturing**: Control robotic arms and conveyors
- **Research**: Interface AI systems with physical robots
- **IoT Robotics**: Integrate sensors and actuators

---

## Installation

### Requirements

- Python 3.8+
- Ubuntu 20.04/22.04/24.04 (for native ROS)
- Or Docker (any platform)

### Method 1: PyPI (Easiest)

```bash
pip install agent-ros-bridge
```

### Method 2: Native Installation (Recommended for Production)

On Ubuntu with ROS installed:

```bash
# One-line installer
curl -sSL https://raw.githubusercontent.com/webthree549-bot/agent-ros-bridge/main/scripts/install-native.sh | bash

# Or manual
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
cd agent-ros-bridge
pip install -e ".[dev]"
```

### Method 3: Docker

```bash
docker-compose --profile ros2 up ros2-bridge
```

---

## Quick Start

### 1. Start the Bridge

**Option A: Mock Mode (No ROS required)**
```bash
python demo/mock_bridge.py
```

**Option B: With ROS**
```bash
source /opt/ros/humble/setup.bash
python run_bridge.py
```

**Option C: Dual ROS1 + ROS2**
```bash
source /opt/ros/noetic/setup.bash
source /opt/ros/humble/setup.bash
python run_bridge_dual_ros.py
```

### 2. Connect and Control

**Using WebSocket Client:**
```bash
wscat -c ws://localhost:8765
> {"command": {"action": "list_robots"}}
```

**Using Web Dashboard:**
```bash
python dashboard/server.py
# Open http://localhost:8080
```

**Using Python:**
```python
import asyncio
import websockets
import json

async def control_robot():
    async with websockets.connect('ws://localhost:8765') as ws:
        # List robots
        await ws.send(json.dumps({
            "command": {"action": "list_robots"}
        }))
        response = await ws.recv()
        print(json.loads(response))
        
        # Send movement command
        await ws.send(json.dumps({
            "command": {
                "action": "move",
                "parameters": {"direction": "forward", "distance": 1.0}
            }
        }))

asyncio.run(control_robot())
```

---

## Configuration

### Configuration File

Create `config/bridge.yaml`:

```yaml
bridge:
  name: "my_robot_fleet"
  
  transports:
    websocket:
      port: 8765
      host: "0.0.0.0"
      auth:
        enabled: false
        jwt_secret: null
    
    mqtt:
      enabled: true
      host: "localhost"
      port: 1883
  
  connectors:
    ros:
      auto_detect: true
      endpoints:
        # Example: Local ROS2
        - id: "turtlebot_01"
          ros_type: "ros2"
          ros_distro: "humble"
          host: "localhost"
          domain_id: 0
        
        # Example: Remote ROS1 arm
        - id: "ur5_arm"
          ros_type: "ros1"
          ros_distro: "noetic"
          host: "192.168.1.100"
  
  metrics:
    enabled: true
    port: 9090
```

### Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `BRIDGE_CONFIG` | Path to config file | `config/bridge.yaml` |
| `ROS_DOMAIN_ID` | ROS2 domain ID | `0` |
| `JWT_SECRET` | JWT signing secret | None |
| `MQTT_BROKER` | MQTT broker host | `localhost` |

---

## Using the Bridge

### WebSocket Protocol

**Connection:**
```
ws://hostname:port[?token=JWT_TOKEN]
```

**Message Format:**
```json
{
  "header": {
    "message_id": "uuid",
    "timestamp": "2025-01-01T00:00:00Z"
  },
  "command": {
    "action": "action_name",
    "parameters": {...}
  }
}
```

**Response Format:**
```json
{
  "header": {
    "message_id": "uuid",
    "correlation_id": "original_message_id"
  },
  "telemetry": {
    "topic": "response_topic",
    "data": {...}
  }
}
```

### Common Commands

**List Robots:**
```json
{"command": {"action": "list_robots"}}
```

**Get Robot State:**
```json
{"command": {"action": "get_robot_state", "parameters": {"robot_id": "tb4_001"}}}
```

**Move Robot:**
```json
{
  "command": {
    "action": "move",
    "parameters": {
      "direction": "forward",
      "distance": 1.0,
      "speed": 0.5
    }
  }
}
```

**Control Arm Joints:**
```json
{
  "command": {
    "action": "arm.move_joints",
    "parameters": {
      "joints": [0, -1.57, 0, -1.57, 0, 0]
    }
  }
}
```

**Navigate to Pose (ROS Action):**
```json
{
  "command": {
    "action": "navigate_to_pose",
    "parameters": {
      "x": 5.0,
      "y": 3.0,
      "theta": 1.57
    }
  }
}
```

---

## Working with Robots

### Mobile Robots (AMR)

**Supported Platforms:**
- TurtleBot4 (ROS2)
- TurtleBot3 (ROS1/ROS2)
- Custom AMRs

**Typical Workflow:**
1. Connect to robot's ROS master
2. List available topics
3. Send navigation goals
4. Monitor odometry and sensor data

### Robot Arms

**Supported Arms:**
- Universal Robots (UR5, UR10, UR3e, UR5e, UR10e)
- UFACTORY xArm (xArm6, xArm7)
- Franka Emika Panda

**Control Modes:**
- **Joint Control**: Move individual joints
- **Cartesian Control**: Move end-effector to position
- **Trajectory**: Follow multi-waypoint paths

**Example Pick and Place:**
```python
from agent_ros_bridge.plugins.arm_robot import ArmRobotPlugin

arm = ArmRobotPlugin(arm_type="ur", ros_version="ros2")

# 1. Move to home
await arm.handle_command("arm.move_joints", {
    "joints": [0, -1.57, 0, -1.57, 0, 0]
})

# 2. Open gripper
await arm.handle_command("arm.gripper", {"position": 0.0})

# 3. Move to pick position
await arm.handle_command("arm.move_joints", {
    "joints": [0.5, -1.0, 0.5, -1.5, -0.5, 0]
})

# 4. Close gripper
await arm.handle_command("arm.gripper", {"position": 0.8})

# 5. Move to place position
await arm.handle_command("arm.move_joints", {
    "joints": [-0.5, -1.0, 0.5, -1.5, -0.5, 0]
})

# 6. Open gripper
await arm.handle_command("arm.gripper", {"position": 0.0})
```

---

## Fleet Management

### Starting the Fleet Orchestrator

```bash
python demo/fleet_demo.py
```

### Submitting Tasks

```json
{
  "command": {
    "action": "fleet.submit_task",
    "parameters": {
      "type": "navigate",
      "target": "warehouse_zone_a",
      "priority": 5,
      "payload": 3.0
    }
  }
}
```

### Task Types

| Type | Description | Required Capabilities |
|------|-------------|---------------------|
| `navigate` | Move to location | `can_navigate` |
| `transport` | Carry payload | `can_navigate`, lift capacity |
| `manipulate` | Arm operation | `can_manipulate` |
| `charge` | Go to charging station | `can_navigate` |

### Querying Fleet Status

```json
{"command": {"action": "fleet.status"}}
```

Response:
```json
{
  "robots": [
    {
      "id": "tb4_001",
      "name": "TurtleBot4-Alpha",
      "status": "BUSY",
      "location": "zone_a",
      "battery": 85,
      "current_task": "task_123"
    }
  ],
  "tasks": [
    {
      "id": "task_123",
      "type": "navigate",
      "status": "EXECUTING",
      "assigned_robot": "tb4_001"
    }
  ],
  "metrics": {
    "total_robots": 4,
    "active": 2,
    "idle": 2,
    "utilization": 50.0
  }
}
```

---

## Monitoring

### Prometheus Metrics

**Start Metrics Server:**
```python
from agent_ros_bridge.metrics import MetricsServer

server = MetricsServer(port=9090)
await server.start()
```

**View Metrics:**
```bash
curl http://localhost:9090/metrics
```

**Key Metrics:**
- `agent_ros_bridge_robots_online` - Connected robots
- `agent_ros_bridge_tasks_completed_total` - Task throughput
- `agent_ros_bridge_messages_sent_total` - Message rate
- `agent_ros_bridge_task_duration_seconds` - Performance

### Grafana Dashboard

1. Install Grafana
2. Add Prometheus data source: `http://localhost:9090`
3. Import dashboard: `dashboards/grafana-dashboard.json`

### Web Dashboard

**Start Dashboard:**
```bash
python dashboard/server.py --port 8080
```

**Features:**
- Real-time robot status
- Telemetry visualization
- Manual robot control
- Task queue monitoring

---

## Troubleshooting

### Connection Issues

**Problem:** Cannot connect to bridge
```
Solution: Check firewall rules
sudo ufw allow 8765/tcp
```

**Problem:** ROS not detected
```
Solution: Source ROS setup
source /opt/ros/humble/setup.bash
```

### Authentication Issues

**Problem:** JWT token rejected
```
Solution: Generate new token
python scripts/generate_token.py --secret "your-secret"
```

### Performance Issues

**Problem:** High latency
```
Solution: Check network, reduce message rate,
or use MQTT for high-frequency telemetry
```

**Problem:** High CPU usage
```
Solution: Reduce number of concurrent connections,
or run on dedicated machine
```

### Debug Mode

Enable debug logging:
```python
logging.basicConfig(level=logging.DEBUG)
```

---

## Advanced Topics

### Custom Plugins

Create a custom robot plugin:

```python
from agent_ros_bridge import Plugin

class MyRobotPlugin(Plugin):
    name = "my_robot"
    version = "1.0.0"
    
    async def initialize(self, gateway):
        # Setup code
        pass
    
    async def handle_message(self, message, identity):
        # Handle commands
        if message.command.action == "my_custom_action":
            # Execute action
            return Message(telemetry=...)
```

### ROS Actions

For long-running tasks:

```python
from agent_ros_bridge.actions import create_action_client

client = create_action_client(
    "navigate_to_pose",
    "nav2_msgs/action/NavigateToPose"
)

# Send goal with feedback
result = await client.send_goal(
    {"pose": {"x": 5.0, "y": 3.0}},
    timeout_sec=60.0
)
```

### Security Hardening

1. Enable JWT authentication
2. Use TLS for WebSocket
3. Restrict network access
4. Regular security updates

See `docs/SECURITY.md` for details.

---

## Support

- **GitHub Issues**: https://github.com/webthree549-bot/agent-ros-bridge/issues
- **Documentation**: https://github.com/webthree549-bot/agent-ros-bridge/tree/main/docs
- **API Reference**: See `docs/API_REFERENCE.md`

---

## License

MIT License - See LICENSE file
