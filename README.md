# Agent ROS Bridge v2.0.0

**Universal ROS1/ROS2 bridge for AI agents to control robots and embodied intelligence systems.**

[![CI](https://github.com/webthree549-bot/agent-ros-bridge/actions/workflows/ci.yml/badge.svg)](https://github.com/webthree549-bot/agent-ros-bridge/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/agent-ros-bridge.svg)](https://pypi.org/project/agent-ros-bridge/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

---

## Quick Start

### Production (requires ROS1 or ROS2)
```bash
pip install agent-ros-bridge
python run_bridge.py  # Auto-detects ROS1 Noetic or ROS2 Humble/Jazzy
```

### Demo/Testing (no ROS required)
```bash
pip install agent-ros-bridge
python demo/mock_bridge.py  # Simulated robot
```

### Docker (production in container)
```bash
docker-compose --profile ros2 up ros2-bridge  # ROS2 + bridge
docker-compose --profile ros1 up ros1-bridge  # ROS1 + bridge
```

### Web Dashboard
```bash
# Start bridge (in another terminal)
python demo/mock_bridge.py

# Start dashboard
python dashboard/server.py
# Open http://localhost:8080 in browser
```

## Project Structure

```
agent-ros-bridge/
├── run_bridge.py          # Production bridge (auto-detects ROS1/ROS2)
├── demo/
│   ├── mock_bridge.py     # Demo mode (simulated robot)
│   ├── mock_bridge_auth.py # With JWT authentication
│   └── mqtt_demo.py       # IoT sensor demo
├── dashboard/
│   ├── server.py          # Web dashboard server
│   └── static/
│       └── index.html     # Dashboard UI
├── docker/
│   ├── Dockerfile.ros1    # ROS1 Noetic container
│   └── Dockerfile.ros2    # ROS2 Jazzy container
├── docker-compose.yml     # Docker orchestration
└── agent_ros_bridge/      # Core package
    └── gateway_v2/
        ├── transports/
        │   ├── websocket.py      # WebSocket server
        │   └── mqtt_transport.py # MQTT client
        └── connectors/
            ├── ros1_connector.py  # ROS1 support
            └── ros2_connector.py  # ROS2 support
```

## Features

- **Multi-Protocol**: WebSocket, gRPC, MQTT, TCP
- **Multi-Robot**: Manage fleets of robots
- **Multi-ROS**: Connect to multiple ROS1/ROS2 endpoints simultaneously
- **Remote ROS**: Connect to robots over the network
- **Plugin System**: Build custom applications
- **ROS Support**: ROS1 Noetic, ROS2 Humble/Jazzy/Iron
- **Cloud-Native**: Docker, Kubernetes ready

## Documentation

- **[User Manual](docs/USER_MANUAL.md)** - Complete usage guide
- **[API Reference](docs/API_REFERENCE.md)** - API documentation
- **[Architecture](docs/ARCHITECTURE.md)** - System design
- **[Multi-ROS Setup](docs/MULTI_ROS.md)** - Fleet management guide

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
