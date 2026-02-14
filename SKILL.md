---
name: agent-ros-bridge
version: 2.0.0
description: Universal ROS1/ROS2 bridge for AI agents to control robots and embodied intelligence systems.
author: Agent ROS Bridge Team
homepage: https://github.com/webthree549-bot/agent-ros-bridge
repository: https://github.com/webthree549-bot/agent-ros-bridge.git
license: MIT
metadata:
  {
    "openclaw":
      {
        "emoji": "🤖",
        "requires": { "bins": ["python3"] },
        "suggests": { "bins": ["docker"] },
        "install":
          [
            {
              "id": "python3",
              "kind": "manual",
              "label": "Python 3.8+",
              "instruction": "Install Python 3.8 or higher from https://python.org",
            },
            {
              "id": "docker",
              "kind": "manual",
              "label": "Docker Desktop (optional)",
              "instruction": "For containerized ROS. Install from https://www.docker.com/products/docker-desktop",
            },
          ],
        "category": "robotics",
        "tags": ["ros", "ros2", "robotics", "iot", "automation", "bridge", "embodied-intelligence", "arm", "navigation"],
        "commands":
          {
            "bridge": 
              {
                "description": "Start the robot bridge",
                "args": [{"name": "config", "required": false, "description": "Path to config file"}],
              },
            "demo":
              {
                "description": "Run demo modes",
                "subcommands":
                  {
                    "mock": "Run mock robot (no ROS required)",
                    "fleet": "Run fleet orchestration demo",
                    "arm": "Run arm robot demo",
                    "actions": "Run ROS actions demo",
                    "mqtt": "Run MQTT IoT demo",
                    "metrics": "Run Prometheus metrics demo",
                  },
              },
            "dashboard":
              {
                "description": "Start web dashboard",
                "args": [{"name": "port", "required": false, "description": "Dashboard port (default: 8080)"}],
              },
          },
      },
  }
---

# 🤖 Agent ROS Bridge

**Universal ROS1/ROS2 bridge for AI agents to control robots and embodied intelligence systems.**

[![CI](https://github.com/webthree549-bot/agent-ros-bridge/actions/workflows/ci.yml/badge.svg)](https://github.com/webthree549-bot/agent-ros-bridge/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/agent-ros-bridge.svg)](https://pypi.org/project/agent-ros-bridge/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

---

## 🚀 OpenClaw Quick Start

### Installation

```bash
# Via ClawHub (recommended)
openclaw skills add agent-ros-bridge

# Or via PyPI
pip install agent-ros-bridge
```

### Start the Bridge

**Option 1: Mock Mode (No ROS required - perfect for testing)**
```bash
openclaw run agent-ros-bridge demo mock
# Or directly:
python demo/mock_bridge.py
```

**Option 2: With ROS (requires ROS installed)**
```bash
source /opt/ros/humble/setup.bash  # or noetic
openclaw run agent-ros-bridge bridge
# Or directly:
python run_bridge.py
```

**Option 3: Web Dashboard**
```bash
openclaw run agent-ros-bridge dashboard
# Or directly:
python dashboard/server.py
# Open http://localhost:8080
```

### Control Robots

Once the bridge is running, control robots via WebSocket:

```python
# In OpenClaw or any Python environment
import asyncio
import websockets
import json

async def control_robot():
    async with websockets.connect('ws://localhost:8765') as ws:
        # List available robots
        await ws.send(json.dumps({
            "command": {"action": "list_robots"}
        }))
        robots = json.loads(await ws.recv())
        print("Robots:", robots)
        
        # Send movement command
        await ws.send(json.dumps({
            "command": {
                "action": "move",
                "parameters": {"direction": "forward", "distance": 1.0}
            }
        }))
        response = await ws.recv()
        print("Response:", response)

asyncio.run(control_robot())
```

---

## ✨ What It Does

Agent ROS Bridge enables OpenClaw agents to control real robots through a unified interface:

| Feature | Description |
|---------|-------------|
| **🤖 Multi-Robot** | Control fleets of robots (AMRs, arms, IoT) |
| **🌐 Multi-Protocol** | WebSocket, MQTT, gRPC support |
| **🔄 Multi-ROS** | ROS1 Noetic + ROS2 Humble/Jazzy simultaneously |
| **🎯 Fleet Orchestration** | Task allocation and coordination |
| **🦾 Arm Control** | UR, xArm, Franka manipulation |
| **📊 Monitoring** | Prometheus metrics + Grafana dashboards |
| **🔐 Security** | JWT authentication |
| **📱 Dashboard** | Web-based robot control |

---

## 📚 Available Demos

All demos work in mock mode (no real hardware required):

```bash
# Mock robot (best for getting started)
openclaw run agent-ros-bridge demo mock

# Fleet of 4 robots with task allocation
openclaw run agent-ros-bridge demo fleet

# Arm robot pick-and-place
openclaw run agent-ros-bridge demo arm -- --arm-type ur --ros-version ros2

# ROS navigation actions
openclaw run agent-ros-bridge demo actions -- --action navigate

# IoT sensor integration
openclaw run agent-ros-bridge demo mqtt

# Prometheus metrics
openclaw run agent-ros-bridge demo metrics
```

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    OpenClaw Agent                           │
└──────────────────┬──────────────────────────────────────────┘
                   │ WebSocket / MQTT / gRPC
                   ▼
┌─────────────────────────────────────────────────────────────┐
│              Agent ROS Bridge Gateway                       │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │ WebSocket   │  │ MQTT        │  │ gRPC                │ │
│  │ Port 8765   │  │ Port 1883   │  │ Port 50051          │ │
│  └─────────────┘  └─────────────┘  └─────────────────────┘ │
│  ┌─────────────────────────────────────────────────────────┐│
│  │  Fleet Orchestrator                                     ││
│  │  • Task allocation                                      ││
│  │  • Load balancing                                       ││
│  └─────────────────────────────────────────────────────────┘│
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │   ROS2      │    │   ROS1      │    │    Arm      │     │
│  │  Connector  │    │  Connector  │    │   Plugin    │     │
│  └─────────────┘    └─────────────┘    └─────────────┘     │
└──────────────────┬──────────────────────────────────────────┘
                   │
        ┌─────────┴─────────┐
        ▼                   ▼
┌──────────────┐    ┌──────────────┐
│  Mobile      │    │  Arm         │
│  Robots      │    │  Robots      │
│  (AMR)       │    │  (UR/xArm)   │
└──────────────┘    └──────────────┘
```

---

## 🔌 WebSocket API

Connect to `ws://localhost:8765` and send commands:

### Common Commands

**List Robots:**
```json
{"command": {"action": "list_robots"}}
```

**Move Robot:**
```json
{"command": {"action": "move", "parameters": {"direction": "forward", "distance": 1.0}}}
```

**Control Arm:**
```json
{"command": {"action": "arm.move_joints", "parameters": {"joints": [0, -1.57, 0, -1.57, 0, 0]}}}
```

**Navigate (ROS Action):**
```json
{"command": {"action": "navigate_to_pose", "parameters": {"x": 5.0, "y": 3.0}}}
```

**Fleet Status:**
```json
{"command": {"action": "fleet.status"}}
```

---

## 📖 Documentation

| Document | Description |
|----------|-------------|
| [User Manual](docs/USER_MANUAL.md) | Complete guide (23,000+ words) |
| [API Reference](docs/API_REFERENCE.md) | Full API documentation |
| [Native ROS](docs/NATIVE_ROS.md) | Ubuntu/ROS installation |
| [Multi-ROS](docs/MULTI_ROS.md) | Fleet management guide |

---

## 🎯 Use Cases

### Warehouse Automation
```python
# Coordinate fleet of AMRs
orchestrator = FleetOrchestrator()
await orchestrator.submit_task(Task(
    type="transport",
    target_location="zone_a",
    payload_kg=10.0
))
```

### Manufacturing
```python
# Control robotic arm
arm = ArmRobotPlugin(arm_type="ur", ros_version="ros2")
await arm.handle_command("arm.move_joints", {
    "joints": [0, -1.57, 0, -1.57, 0, 0]
})
```

### Research
```python
# Interface AI with physical robots
client = create_action_client("navigate_to_pose", "nav2_msgs/NavigateToPose")
result = await client.send_goal({"pose": {"x": 5, "y": 3}})
```

---

## 🔗 Links

- **Documentation**: https://github.com/webthree549-bot/agent-ros-bridge/tree/main/docs
- **PyPI**: https://pypi.org/project/agent-ros-bridge/
- **GitHub**: https://github.com/webthree549-bot/agent-ros-bridge
- **Issues**: https://github.com/webthree549-bot/agent-ros-bridge/issues

## License

MIT License - See [LICENSE](LICENSE)
