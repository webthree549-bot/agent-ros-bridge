# Agent ROS Bridge

**Universal ROS1/ROS2 bridge for AI agents to control robots and embodied intelligence systems.**

[![CI](https://github.com/webthree549-bot/agent-ros-bridge/actions/workflows/ci.yml/badge.svg)](https://github.com/webthree549-bot/agent-ros-bridge/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/agent-ros-bridge.svg)](https://pypi.org/project/agent-ros-bridge/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

---

## 🔐 Security-First Design

**JWT authentication is always required and cannot be disabled.**

```bash
# Generate a secure secret (REQUIRED - no exceptions)
export JWT_SECRET=$(openssl rand -base64 32)
```

The bridge will **fail to start** without JWT_SECRET. This is by design — security is not optional.

See [SECURITY.md](SECURITY.md) for complete security guidelines.

---

## Quick Start

### Production (Native ROS)

**Requirements:** Ubuntu 20.04/22.04 with ROS1 Noetic or ROS2 Humble/Jazzy

```bash
# Install
pip install agent-ros-bridge

# Set required secret
export JWT_SECRET=$(openssl rand -base64 32)

# Start bridge
agent-ros-bridge --config config/bridge.yaml
```

### Docker Examples (Recommended for Testing)

All examples run in isolated Docker containers with mock robots (no ROS installation needed).

```bash
# Clone repository
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
cd agent-ros-bridge

# Generate JWT secret
export JWT_SECRET=$(openssl rand -base64 32)

# Run example in Docker
cd examples/quickstart
docker-compose up

# Test connection
curl http://localhost:8765/health
```

### Available Docker Examples

| Example | Description | Run |
|---------|-------------|-----|
| `examples/quickstart/` | Basic bridge | `docker-compose up` |
| `examples/fleet/` | Multi-robot fleet | `docker-compose up` |
| `examples/arm/` | Robot arm control | `docker-compose up` |

All examples include:
- Isolated Docker container
- Pre-configured JWT auth
- Mock robot simulation
- Localhost-only binding (127.0.0.1)

---

## Installation

### Via PyPI (Production)

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

---

## Usage

### Python API

```python
from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

# Bridge requires JWT_SECRET env var
bridge = Bridge()
bridge.transport_manager.register(WebSocketTransport({"port": 8765}))
await bridge.start()
```

### CLI

```bash
# Set required secret
export JWT_SECRET=$(openssl rand -base64 32)

# Start bridge
agent-ros-bridge --config config/bridge.yaml

# Generate token for client
python scripts/generate_token.py --secret $JWT_SECRET --role operator
```

---

## Features

- **Security-First**: JWT auth always required, no bypass
- **Multi-Protocol**: WebSocket, gRPC, MQTT
- **Multi-ROS**: ROS1 Noetic + ROS2 Humble/Jazzy
- **Fleet Orchestration**: Multi-robot coordination
- **Arm Control**: UR, xArm, Franka manipulation
- **Docker Examples**: Isolated testing environments
- **Production Ready**: Native Ubuntu deployment

---

## Documentation

| Document | Description |
|----------|-------------|
| [User Manual](docs/USER_MANUAL.md) | Complete guide (23,000+ words) |
| [API Reference](docs/API_REFERENCE.md) | Full API documentation |
| [Native ROS](docs/NATIVE_ROS.md) | Ubuntu/ROS installation |
| [Multi-ROS](docs/MULTI_ROS.md) | Fleet management |
| [Docker vs Native](docs/DOCKER_VS_NATIVE.md) | Deployment comparison |
| [SECURITY.md](SECURITY.md) | Security policy |

---

## Development

```bash
# Install dev dependencies
pip install -e ".[dev]"

# Run tests
make test

# Build package
make build
```

---

## License

[MIT License](LICENSE)

---

**Security is not optional. JWT auth always required.**
