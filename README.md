# Agent ROS Bridge v2.0.0

<p align="center">
  <img src="https://img.shields.io/badge/ROS-1%2F2-orange.svg" alt="ROS">
  <img src="https://img.shields.io/badge/Python-3.8%2B-blue.svg" alt="Python">
  <img src="https://img.shields.io/badge/License-MIT-green.svg" alt="License">
</p>

<p align="center">
  <a href="https://github.com/webthree549-bot/agent-ros-bridge/actions/workflows/ci.yml">
    <img src="https://github.com/webthree549-bot/agent-ros-bridge/actions/workflows/ci.yml/badge.svg" alt="CI">
  </a>
  <a href="https://pypi.org/project/agent-ros-bridge/">
    <img src="https://img.shields.io/pypi/v/agent-ros-bridge.svg" alt="PyPI">
  </a>
  <a href="https://codecov.io/gh/webthree549-bot/agent-ros-bridge">
    <img src="https://codecov.io/gh/webthree549-bot/agent-ros-bridge/branch/main/graph/badge.svg" alt="Coverage">
  </a>
</p>

---

**Agent ROS Bridge** - Universal ROS1/ROS2 bridge for AI agents to control robots and embodied intelligence systems.

## 🚀 Quick Start

```bash
# Install
pip install agent-ros-bridge

# Run demo
agent-ros-bridge --demo

# Or with Docker
docker run -p 8765:8765 agent-ros-bridge/agent-ros-bridge:latest
```

## ✨ Features

- **Multi-Protocol**: WebSocket, gRPC, MQTT, TCP
- **Multi-Robot**: Manage fleets of robots
- **Plugin System**: Build custom applications
- **ROS Support**: ROS1 Noetic, ROS2 Humble/Jazzy
- **Cloud-Native**: Docker, Kubernetes ready

## 📚 Documentation

- **[User Manual](docs/USER_MANUAL.md)** - Complete usage guide
- **[API Reference](docs/API_REFERENCE.md)** - API documentation
- **[Architecture](docs/ARCHITECTURE.md)** - System design
- **[Migration Guide](docs/MIGRATION.md)** - v1 to v2 migration

## 🛠️ Installation

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

## 💻 Basic Usage

### Python API

```python
from agent_ros_bridge import Bridge
from agent_ros_bridge.transports.websocket import WebSocketTransport

# Create bridge
bridge = Bridge()

# Add transport
bridge.transport_manager.register(
    WebSocketTransport({"port": 8765})
)

# Start
await bridge.start()
```

### WebSocket Client

```python
import asyncio
import websockets
import json

async def client():
    async with websockets.connect("ws://localhost:8765") as ws:
        await ws.send(json.dumps({
            "command": {"action": "ping"}
        }))
        response = await ws.recv()
        print(response)

asyncio.run(client())
```

## 🏗️ Architecture

```
┌──────────────┐      WebSocket/gRPC/MQTT      ┌──────────────────┐
│   AI Agent   │  ◄──────────────────────────►  │  Agent ROS       │
│  (Any AI)    │                               │     Bridge       │
└──────────────┘                               └────────┬─────────┘
                                                        │
                              ┌────────────────────────┼────────────────────────┐
                              │                        │                        │
                              ▼                        ▼                        ▼
                       ┌────────────┐          ┌────────────┐          ┌────────────┐
                       │   Robot    │          │   Robot    │          │   Robot    │
                       │  (ROS2)    │          │  (ROS1)    │          │   (MQTT)   │
                       └────────────┘          └────────────┘          └────────────┘
```

## 🧪 Development

```bash
# Setup
make install-dev

# Run tests
make test

# Run demo
make dev-server

# Build docs
make docs
```

## 📦 Docker

```bash
# Pull image
docker pull agent-ros-bridge/agent-ros-bridge:latest

# Run
docker run -p 8765:8765 -p 50051:50051 agent-ros-bridge/agent-ros-bridge:latest

# Or use docker-compose
docker-compose up -d
```

## 🤝 Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## 📄 License

[MIT License](LICENSE)

## 🔗 Links

- **Documentation**: https://agent-ros-bridge.readthedocs.io
- **PyPI**: https://pypi.org/project/agent-ros-bridge/
- **Docker Hub**: https://hub.docker.com/r/agent-ros-bridge/agent-ros-bridge
- **Issues**: https://github.com/webthree549-bot/agent-ros-bridge/issues

---

<p align="center">
  Made with ❤️ by the Agent ROS Bridge Team
</p>
