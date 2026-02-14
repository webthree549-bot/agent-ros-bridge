# Agent ROS Bridge v2.0.0

**Universal ROS1/ROS2 bridge for AI agents to control robots and embodied intelligence systems.**

[![CI](https://github.com/webthree549-bot/agent-ros-bridge/actions/workflows/ci.yml/badge.svg)](https://github.com/webthree549-bot/agent-ros-bridge/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/agent-ros-bridge.svg)](https://pypi.org/project/agent-ros-bridge/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

---

## Quick Start

### Option 1: Mock Mode (No ROS2 Required)
Test the WebSocket API without a real robot:
```bash
pip install agent-ros-bridge
python run_bridge.py
# WebSocket available at ws://localhost:8765
```

### Option 2: With Real ROS2 (Docker)
```bash
docker-compose up ros2-bridge
```

### Option 3: Development
```bash
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
cd agent-ros-bridge
pip install -e ".[dev]"
python run_bridge.py
```

## Features

- **Multi-Protocol**: WebSocket, gRPC, MQTT, TCP
- **Multi-Robot**: Manage fleets of robots
- **Plugin System**: Build custom applications
- **ROS Support**: ROS1 Noetic, ROS2 Humble/Jazzy
- **Cloud-Native**: Docker, Kubernetes ready

## Documentation

- **[User Manual](docs/USER_MANUAL.md)** - Complete usage guide
- **[API Reference](docs/API_REFERENCE.md)** - API documentation
- **[Architecture](docs/ARCHITECTURE.md)** - System design

## Installation

### Via PyPI
```bash
pip install agent-ros-bridge
```

### Via ClawHub
```bash
openclaw skills add agent-ros-bridge
```

### From Source
```bash
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
cd agent-ros-bridge
pip install -e ".[dev]"
```

## Usage

### Python API
```python
from agent_ros_bridge import Bridge
from agent_ros_bridge.transports.websocket import WebSocketTransport

bridge = Bridge()
bridge.transport_manager.register(WebSocketTransport({"port": 8765}))
await bridge.start()
```

### CLI
```bash
agent-ros-bridge --demo
agent-ros-bridge --config ./config.yaml
```

## Development

```bash
make install-dev
make test
make dev-server
```

## License

[MIT License](LICENSE)

---

Made with ❤️ by the Agent ROS Bridge Team
