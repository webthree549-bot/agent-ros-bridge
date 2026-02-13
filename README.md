# OpenClaw ROS Bridge v2.0.0

<p align="center">
  <img src="https://img.shields.io/badge/ROS-1%2F2-orange.svg" alt="ROS">
  <img src="https://img.shields.io/badge/Python-3.8%2B-blue.svg" alt="Python">
  <img src="https://img.shields.io/badge/License-MIT-green.svg" alt="License">
</p>

<p align="center">
  <a href="https://github.com/webthree549-bot/openclaw-ros-bridge/actions/workflows/ci.yml">
    <img src="https://github.com/webthree549-bot/openclaw-ros-bridge/actions/workflows/ci.yml/badge.svg" alt="CI">
  </a>
  <a href="https://pypi.org/project/openclaw-ros-bridge/">
    <img src="https://img.shields.io/pypi/v/openclaw-ros-bridge.svg" alt="PyPI">
  </a>
  <a href="https://codecov.io/gh/webthree549-bot/openclaw-ros-bridge">
    <img src="https://codecov.io/gh/webthree549-bot/openclaw-ros-bridge/branch/main/graph/badge.svg" alt="Coverage">
  </a>
</p>

---

**Universal Robot Gateway** - Multi-protocol, multi-robot, cloud-native connectivity platform for AI agents to control ROS-based robots.

## 🚀 Quick Start

```bash
# Install
pip install openclaw-ros-bridge

# Run demo
openclaw-gateway --demo

# Or with Docker
docker run -p 8765:8765 ghcr.io/webthree549-bot/openclaw-ros-bridge:latest
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
pip install openclaw-ros-bridge
```

### Via ClawHub

```bash
openclaw skills add openclaw-ros-bridge
```

### From Source

```bash
git clone https://github.com/webthree549-bot/openclaw-ros-bridge.git
cd openclaw-ros-bridge
pip install -e ".[dev]"
```

## 💻 Basic Usage

### Python API

```python
from openclaw_ros_bridge.gateway_v2 import OpenClawGateway
from openclaw_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

# Create gateway
gateway = OpenClawGateway()

# Add transport
gateway.transport_manager.register(
    WebSocketTransport({"port": 8765})
)

# Start
await gateway.start()
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
│  OpenClaw    │  ◄──────────────────────────►  │     Gateway      │
│  AI Agent    │                               │  (This Project)  │
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
docker pull ghcr.io/webthree549-bot/openclaw-ros-bridge:latest

# Run
docker run -p 8765:8765 -p 50051:50051 openclaw/ros-bridge:latest

# Or use docker-compose
docker-compose up -d
```

## 🤝 Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## 📄 License

[MIT License](LICENSE)

## 🔗 Links

- **Documentation**: https://openclaw-ros-bridge.readthedocs.io
- **PyPI**: https://pypi.org/project/openclaw-ros-bridge/
- **Docker Hub**: https://hub.docker.com/r/openclaw/ros-bridge
- **Issues**: https://github.com/webthree549-bot/openclaw-ros-bridge/issues

---

<p align="center">
  Made with ❤️ by the OpenClaw ROS Team
</p>
