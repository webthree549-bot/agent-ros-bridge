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
  <a href="https://github.com/webthree549-bot/openclaw-ros-bridge/actions/workflows/release.yml">
    <img src="https://github.com/webthree549-bot/openclaw-ros-bridge/actions/workflows/release.yml/badge.svg" alt="Release">
  </a>
  <a href="https://codecov.io/gh/webthree549-bot/openclaw-ros-bridge">
    <img src="https://codecov.io/gh/webthree549-bot/openclaw-ros-bridge/branch/main/graph/badge.svg" alt="Coverage">
  </a>
  <a href="https://pypi.org/project/openclaw-ros-bridge/">
    <img src="https://img.shields.io/pypi/v/openclaw-ros-bridge.svg" alt="PyPI">
  </a>
  <a href="https://hub.docker.com/r/openclaw/ros-bridge">
    <img src="https://img.shields.io/docker/pulls/openclaw/ros-bridge.svg" alt="Docker Pulls">
  </a>
</p>

<p align="center">
  <a href="https://openclaw-ros-bridge.readthedocs.io">
    <img src="https://img.shields.io/badge/docs-mkdocs-blue.svg" alt="Documentation">
  </a>
  <a href="https://securityscorecards.dev/viewer/?uri=github.com/webthree549-bot/openclaw-ros-bridge">
    <img src="https://api.securityscorecards.dev/projects/github.com/webthree549-bot/openclaw-ros-bridge/badge" alt="OpenSSF Scorecard">
  </a>
  <a href="https://github.com/webthree549-bot/openclaw-ros-bridge/blob/main/LICENSE">
    <img src="https://img.shields.io/github/license/webthree549-bot/openclaw-ros-bridge.svg" alt="License">
  </a>
</p>

---

**Universal Robot Gateway** - Multi-protocol, multi-robot, cloud-native connectivity platform for AI agents to control ROS-based robots and embodied intelligence systems.

## 🌟 Key Features

### Multi-Protocol Support
- **WebSocket** - Browser dashboards, real-time web apps
- **gRPC** - High-performance microservices, cloud deployments  
- **TCP Socket** - Legacy compatibility, simple integrations
- **MQTT** - IoT, mobile robots, low-bandwidth networks
- **QUIC** - Mobile, lossy networks, fast reconnect

### Multi-Robot Management
- **Fleet Management** - Control multiple robots as a unified fleet
- **Auto-Discovery** - Find robots via mDNS, ROS, MQTT
- **Broadcast Commands** - Send commands to robot groups
- **Smart Routing** - Route messages based on content and priority

### Universal Connectors
- **ROS1/ROS2** - Native support for all ROS distributions
- **Industrial** - Modbus, EtherCAT, Profinet
- **Drones** - MAVLink, DJI SDK
- **Simulation** - Gazebo, Isaac Sim, MuJoCo

### Production-Ready
- **Comprehensive Testing** - Unit, integration, performance tests
- **Security** - TLS, JWT auth, API keys, audit logging
- **Observability** - Prometheus metrics, OpenTelemetry tracing
- **CI/CD** - Automated testing, Docker builds, PyPI releases

## 🚀 Quick Start

### Installation

```bash
# From PyPI
pip install openclaw-ros-bridge

# With all extras
pip install "openclaw-ros-bridge[all]"

# From source
git clone https://github.com/webthree549-bot/openclaw-ros-bridge.git
cd openclaw-ros-bridge
pip install -e ".[dev]"
```

### Run Demo

```bash
# Start demo with WebSocket transport + greenhouse plugin
openclaw-gateway --demo

# Or with custom config
openclaw-gateway --config ./config.yaml
```

### Docker

```bash
# Pull and run
docker run -p 8765:8765 ghcr.io/webthree549-bot/openclaw-ros-bridge:latest

# Or with docker-compose
docker-compose up -d
```

## 💻 Usage

### Python API

```python
import asyncio
from openclaw_ros_bridge.gateway_v2 import OpenClawGateway
from openclaw_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
from openclaw_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector

async def main():
    # Create gateway
    gateway = OpenClawGateway()
    
    # Add transports
    gateway.transport_manager.register(
        WebSocketTransport({"port": 8765})
    )
    
    # Add connectors
    gateway.connector_registry.register(ROS2Connector())
    
    # Create robot fleet
    warehouse = gateway.create_fleet("warehouse")
    
    # Connect to robot
    robot = await gateway.connect_robot(
        "ros2://192.168.1.100",
        fleet_name="warehouse"
    )
    
    # Start gateway
    async with gateway.run():
        # Broadcast command to all robots in fleet
        await warehouse.broadcast({
            "action": "move_to",
            "parameters": {"x": 1.0, "y": 2.0}
        })
        
        # Subscribe to telemetry
        async for telemetry in robot.subscribe("/sensors"):
            print(f"Received: {telemetry.data}")

asyncio.run(main())
```

### CLI

```bash
# Start gateway
openclaw-gateway --websocket-port 8765 --grpc-port 50051

# List available commands
curl ws://localhost:8765
> {"action": "list_handlers"}

# Connect to ROS2 and discover robots
> {"action": "discover"}

# Send command to robot
> {"action": "robot.execute", "parameters": {"robot_id": "robot1", "action": "move_to", "x": 1.0, "y": 2.0}}
```

## 📁 Project Structure

```
openclaw-ros-bridge/
├── openclaw_ros_bridge/        # Main package
│   ├── gateway_v2/            # New gateway architecture
│   │   ├── core.py            # Core abstractions
│   │   ├── config.py          # Configuration system
│   │   ├── transports/        # Transport implementations
│   │   ├── connectors/        # Robot connectors
│   │   └── plugins/           # Application plugins
│   └── ...                    # Legacy v1 code
├── tests/                     # Test suite
├── docs/                      # Documentation
├── docker/                    # Docker configurations
├── scripts/                   # Utility scripts
├── .github/                   # GitHub Actions & templates
├── pyproject.toml             # Python package config
├── Makefile                   # Development commands
└── README.md                  # This file
```

## 🧪 Development

```bash
# Setup
make install-dev

# Run tests
make test

# Run with coverage
make test-cov

# Lint code
make lint

# Format code
make format

# Build documentation
make docs

# Run CI checks locally
make ci
```

See [CONTRIBUTING.md](CONTRIBUTING.md) for detailed contribution guidelines.

## 📚 Documentation

- [Full Documentation](https://openclaw-ros-bridge.readthedocs.io)
- [API Reference](https://openclaw-ros-bridge.readthedocs.io/api)
- [Architecture](docs/ARCHITECTURE_V2.md)
- [Deployment Guide](docs/deployment.md)

## 🐳 Docker Images

| Tag | Description |
|-----|-------------|
| `latest` | Latest stable release |
| `latest-humble` | ROS2 Humble based |
| `latest-jazzy` | ROS2 Jazzy based |
| `v2.0.0` | Specific version |
| `v2.0.0-humble` | Version + ROS distro |

All images support `linux/amd64` and `linux/arm64`.

## 🔒 Security

We take security seriously. Please report vulnerabilities privately:

- [GitHub Security Advisory](https://github.com/webthree549-bot/openclaw-ros-bridge/security/advisories/new)
- Email: security@openclaw-ros.org

See [SECURITY.md](SECURITY.md) for details.

## 📊 Stats

[![Stargazers over time](https://starchart.cc/webthree549-bot/openclaw-ros-bridge.svg)](https://starchart.cc/webthree549-bot/openclaw-ros-bridge)

## 🤝 Contributing

We welcome contributions! Please see:

- [Contributing Guide](CONTRIBUTING.md)
- [Code of Conduct](CODE_OF_CONDUCT.md)
- [Development Guide](docs/development.md)

## 📄 License

This project is licensed under the [MIT License](LICENSE).

## 🙏 Acknowledgments

- [ROS](https://www.ros.org/) - Robot Operating System
- [OpenClaw](https://github.com/openclaw) - AI agent framework
- [NVIDIA](https://developer.nvidia.com/isaac) - Isaac Sim support
- All [contributors](https://github.com/webthree549-bot/openclaw-ros-bridge/graphs/contributors)

## 📞 Contact

- Issues: [GitHub Issues](https://github.com/webthree549-bot/openclaw-ros-bridge/issues)
- Discussions: [GitHub Discussions](https://github.com/webthree549-bot/openclaw-ros-bridge/discussions)
- Email: dev@openclaw-ros.org

---

<p align="center">
  Made with ❤️ by the OpenClaw ROS Team
</p>
