---
name: agent-ros-bridge
version: 2.0.0
description: Agent ROS Bridge - Universal ROS1/ROS2 bridge for AI agents to control robots and embodied intelligence systems.
author: Agent ROS Bridge Team
homepage: https://github.com/webthree549-bot/agent-ros-bridge
repository: https://github.com/webthree549-bot/agent-ros-bridge.git
license: MIT
metadata:
  {
    "openclaw":
      {
        "emoji": "🤖",
        "requires": { "bins": ["docker", "python3"] },
        "install":
          [
            {
              "id": "docker",
              "kind": "manual",
              "label": "Docker Desktop",
              "instruction": "Install Docker Desktop from https://www.docker.com/products/docker-desktop",
            },
            {
              "id": "python3",
              "kind": "manual",
              "label": "Python 3.8+",
              "instruction": "Install Python 3.8 or higher from https://python.org",
            },
          ],
        "category": "robotics",
        "tags": ["ros", "ros2", "robotics", "iot", "automation", "bridge", "embodied-intelligence"],
      },
  }
---

# 🤖 Agent ROS Bridge

**Universal ROS1/ROS2 bridge for AI agents to control robots and embodied intelligence systems.**

[![CI](https://github.com/webthree549-bot/agent-ros-bridge/actions/workflows/ci.yml/badge.svg)](https://github.com/webthree549-bot/agent-ros-bridge/actions/workflows/ci.yml)
[![Release](https://github.com/webthree549-bot/agent-ros-bridge/actions/workflows/release.yml/badge.svg)](https://github.com/webthree549-bot/agent-ros-bridge/actions/workflows/release.yml)
[![PyPI](https://img.shields.io/pypi/v/agent-ros-bridge.svg)](https://pypi.org/project/agent-ros-bridge/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

---

## What It Does

This skill provides the **infrastructure** for AI agents to communicate with ROS-based systems:

- **Generic ROS communication** - Send/receive messages to any ROS topic
- **Hardware abstraction** - Unified interface for sensors and actuators
- **Plugin architecture** - Build applications on top of the bridge
- **Version agnostic** - Works with ROS1 (Noetic) and ROS2 (Humble/Jazzy)
- **Simulation ready** - Mock mode for testing without hardware

## Quick Install

```bash
# Via ClawHub
openclaw skills add agent-ros-bridge

# Or manually
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
./install.sh
```

## Architecture

The bridge is **application-agnostic**. It provides the communication infrastructure, and applications (like greenhouse control, arm manipulation) are built as plugins on top.

```
┌─────────────────────────────────────────────────────────────┐
│                    AI Agent                                 │
└──────────────────┬──────────────────────────────────────────┘
                   │ WebSocket/gRPC/MQTT
                   ▼
┌─────────────────────────────────────────────────────────────┐
│              Agent ROS Bridge (Generic)                     │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  TCP Server - Core Communication Layer             │   │
│  │  • ping, get_status, list_handlers (built-in)      │   │
│  │  • Plugin-registered handlers (application)        │   │
│  └─────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  Transport Layer                                     │   │
│  │  • WebSocket (8765) • gRPC (50051) • MQTT • TCP    │   │
│  └─────────────────────────────────────────────────────┘   │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │   ROS2      │    │   ROS1      │    │    HAL      │     │
│  │Communicator │    │Communicator │    │(Hardware    │     │
│  └─────────────┘    └─────────────┘    │ Abstraction)│     │
│                                         └─────────────┘     │
└──────────────────┬──────────────────────────────────────────┘
                   │
        ┌─────────┴─────────┐
        ▼                   ▼
┌──────────────┐    ┌──────────────┐
│  Greenhouse  │    │  Arm Robot   │
│    Demo      │    │   (Future)   │
└──────────────┘    └──────────────┘
     (Applications built on the bridge)
```

## Quick Start

### 1. Start Bridge Container

```bash
# Start Docker container with ROS2 Jazzy
./scripts/docker_start.sh

# Or start with specific ROS version
./scripts/docker_start.sh --jazzy    # ROS2 Jazzy (default)
./scripts/docker_start.sh --humble   # ROS2 Humble
./scripts/docker_start.sh --noetic   # ROS1 Noetic
```

### 2. Run Generic TCP Server

```bash
# Start the application-agnostic bridge
./scripts/start_bridge.sh

# Test connection
python3 -c "
import socket, json
s = socket.socket()
s.connect(('localhost', 9999))
s.send(json.dumps({'action': 'ping'}).encode() + b'\n')
print(json.loads(s.recv(4096).decode()))
"
```

### 3. Try a Demo Application

```bash
# Run the greenhouse demo (application built on the bridge)
./demo/greenhouse/scripts/demo.sh

# Or run greenhouse demo in mock mode (no ROS/hardware required)
export MOCK_MODE=true
./demo/greenhouse/scripts/run_demo.sh --mock
```

## Core Commands

The generic bridge provides these built-in commands:

| Command | Description |
|---------|-------------|
| `ping` | Health check |
| `get_status` | Get bridge status and registered handlers |
| `list_handlers` | List available command handlers |

Applications register additional handlers. See `demo/greenhouse/` for an example.

## Available Commands

### Container Management

```bash
# Start Docker container with bridge
./scripts/docker_start.sh              # ROS2 Jazzy (default)
./scripts/docker_start.sh --jazzy      # ROS2 Jazzy
./scripts/docker_start.sh --humble     # ROS2 Humble
./scripts/docker_start.sh --noetic     # ROS1 Noetic

# Stop container
./scripts/docker_start.sh --stop

# Remove container
./scripts/docker_start.sh --rm

# Run ROS commands inside container
ros2-jazzy-bridge ros2 topic list
ros2-jazzy-bridge ros2 node list
ros2-jazzy-bridge ros2 doctor

# Check container status
docker ps | grep ros
```

### Demos (Application Examples)

```bash
# Greenhouse demo (agricultural robotics)
./demo/greenhouse/scripts/demo.sh
./demo/greenhouse/scripts/run_demo.sh --mock

# Control greenhouse manually
./demo/greenhouse/scripts/control.sh status
./demo/greenhouse/scripts/control.sh fan on
./demo/greenhouse/scripts/control.sh valve open
```

### Development

```bash
cd <path-to-agent-ros-bridge>

# Build the project
./scripts/build.sh

# Run tests
./scripts/run_tests.sh

# Check version detection
python3 -c "
from agent_ros_bridge import version_manager
print(f'ROS: {version_manager.ROS_TYPE} {version_manager.ROS_DISTRO}')
print(f'Bridge: {version_manager.VERSION}')
"
```

## Configuration

### Environment Variables

```bash
# ROS Version (auto-detected if not set)
export ROS_DISTRO=jazzy          # humble, jazzy, noetic
export ROS_TYPE=ros2             # ros1, ros2

# Operation Mode
export MOCK_MODE=true            # true = simulation, false = real hardware
export HAL_HARDWARE=auto         # auto, dht22, bme280, robotiq_2f_85, etc.
```

### Config Files

All configs are in `config/`:
- `ros2_config.yaml` - ROS2 Humble/Jazzy settings
- `bridge_config.yaml` - TCP ports, timeouts
- `hal_config.yaml` - Hardware abstraction settings
- `fault_config.yaml` - Recovery policies

## Building Applications

To build your own application on the bridge:

### 1. Create a Plugin

```python
# my_app/my_plugin.py
from agent_ros_bridge.communication.tcp_server import bridge_server
from agent_ros_bridge.hal import sensor_hal, actuator_hal

class MyRobotPlugin:
    def register(self, server):
        # Register custom commands
        server.register_handler("move_arm", self.handle_move_arm)
        server.register_handler("gripper", self.handle_gripper)
    
    def handle_move_arm(self, cmd):
        x, y, z = cmd.get('x'), cmd.get('y'), cmd.get('z')
        # Your control logic here
        return {"status": "ok", "position": [x, y, z]}
    
    def handle_gripper(self, cmd):
        state = cmd.get('state')  # 'open' or 'close'
        # Your control logic here
        return {"status": "ok", "state": state}
```

### 2. Launch with Plugin

```python
# my_app/my_server.py
from agent_ros_bridge.communication.tcp_server import bridge_server
from my_app.my_plugin import MyRobotPlugin

# Initialize plugin
plugin = MyRobotPlugin()
plugin.register(bridge_server)

# Start server
bridge_server.start()
```

### 3. Use from AI Agent

```python
# Core commands (always available)
{"action": "ping"}
{"action": "get_status"}  # Shows registered handlers
{"action": "list_handlers"}

# Your custom commands
{"action": "move_arm", "x": 0.5, "y": 0.2, "z": 0.1}
{"action": "gripper", "state": "close"}
```

See `demo/greenhouse/` for a complete working example.

## Common Workflows

### Workflow 1: Test Without Hardware (Mock Mode)

```bash
export MOCK_MODE=true
./scripts/start_bridge.sh

# Or run greenhouse demo in mock mode
./demo/greenhouse/scripts/run_demo.sh --mock
```

### Workflow 2: Read Sensor Data

```python
from agent_ros_bridge import sensor_hal, ros2_comm

# Initialize
sensor_hal.init_hardware()

# Read temperature/humidity
data = sensor_hal.read("env")
print(f"Temp: {data['temperature']}°C, Humidity: {data['humidity']}%")
```

### Workflow 3: Control Actuators

```python
from agent_ros_bridge import actuator_hal

# Initialize
actuator_hal.init_hardware()

# Turn on device
actuator_hal.write({"motor": True})

# Emergency stop
actuator_hal.safe_state()
```

## Docker Deployment

```bash
cd <path-to-agent-ros-bridge>

# Build images
./scripts/docker_build.sh

# Start all services
docker-compose -f docker/docker-compose.yml up -d

# View logs
docker-compose -f docker/docker-compose.yml logs -f

# Stop
docker-compose -f docker/docker-compose.yml down
```

## Troubleshooting

### Container Won't Start

```bash
# Check Docker is running
docker info

# Restart container
docker restart agent-ros-bridge

# Check logs
docker logs agent-ros-bridge
```

### ROS Topics Not Showing

```bash
# Source ROS environment
source /opt/ros/jazzy/setup.bash

# Check nodes are running
ros2 node list

# Check topic types
ros2 topic type /greenhouse/sensors
```

### Permission Issues

```bash
# Fix script permissions
chmod +x scripts/*.sh
chmod +x demo/greenhouse/scripts/*.sh

# Fix Docker permissions (macOS)
sudo dseditgroup -o edit -a $USER -t user docker
```

## Project Structure

```
agent-ros-bridge/
├── agent_ros_bridge/               # Core bridge (application-agnostic)
│   ├── communication/              # TCP server, ROS communicators
│   │   └── tcp_server.py           # Generic TCP server
│   ├── hal/                        # Hardware abstraction layer
│   ├── ros1/                       # ROS1 support
│   ├── ros2/                       # ROS2 support
│   └── ...
├── demo/                           # Application examples
│   └── greenhouse/                 # Greenhouse demo (application)
│       ├── greenhouse_plugin.py    # Plugin implementation
│       ├── greenhouse_server.py    # Server launcher
│       ├── scripts/                # Demo scripts
│       └── README.md               # Demo documentation
├── scripts/                        # Core bridge scripts
│   ├── docker_start.sh             # Container management
│   ├── start_bridge.sh             # Generic server launcher
│   └── ...
├── config/                         # Configuration files
├── docs/                           # Documentation
└── README.md                       # Project readme
```

## Documentation

| Document | Description |
|----------|-------------|
| `README.md` | Project overview and quick start |
| `docs/ARCHITECTURE.md` | Detailed architecture and design patterns |
| `docs/VERSION_AGNOSTIC.md` | Version-agnostic design principles |
| `demo/greenhouse/README.md` | Greenhouse demo documentation |

## Tips

1. **The bridge is application-agnostic** - Greenhouse is just a demo
2. **Use mock mode for testing** - No hardware required
3. **Build applications as plugins** - See `demo/greenhouse/` for example
4. **Check version detection first** - Run version_manager check
5. **Use Docker for isolation** - Avoid system ROS conflicts
6. **Read the logs** - Most issues are in `logs/` directory

---

## Links

- **Documentation**: https://agent-ros-bridge.readthedocs.io
- **GitHub**: https://github.com/webthree549-bot/agent-ros-bridge
- **PyPI**: https://pypi.org/project/agent-ros-bridge/
- **Docker Hub**: https://hub.docker.com/r/agent-ros-bridge/agent-ros-bridge
- **Issues**: https://github.com/webthree549-bot/agent-ros-bridge/issues

## License

MIT License - See [LICENSE](https://github.com/webthree549-bot/agent-ros-bridge/blob/main/LICENSE)

## Contributing

Contributions welcome! See [CONTRIBUTING.md](https://github.com/webthree549-bot/agent-ros-bridge/blob/main/CONTRIBUTING.md)

---

**Made with ❤️ by the Agent ROS Bridge Team**
