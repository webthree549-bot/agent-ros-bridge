# Agent ROS Bridge

> ⚠️ **HONESTY NOTICE**  
> v0.4.0 was released prematurely. The PyPI package provides working ROS bridge 
> functionality (gateway_v2), but AI agent integrations (LangChain, AutoGPT, etc.) 
> are present as code but not yet integrated. We're actively fixing this in v0.4.1.  
> **See [ROADMAP.md](ROADMAP.md) for the real plan.**

---

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

---

## ✅ What's Working (v0.4.0)

- **Multi-Protocol Transports** — WebSocket, gRPC, MQTT
- **ROS1 & ROS2** — Compatible with both ROS versions
  - ROS2: Jazzy Jalisco (LTS), Humble Hawksbill (LTS), Iron Irwini, Rolling
  - ROS1: Noetic Ninjemys
- **Security** — JWT authentication, TLS encryption, mTLS support
- **Fleet Orchestration** — Multi-robot coordination
- **Docker Examples** — Isolated testing environments
- **Prometheus Metrics** — Basic observability

## 🚧 Coming in v0.5.0

- **LangChain Integration** — Planned, not yet integrated
- **AutoGPT Plugin** — Planned, not yet integrated
- **Agent Memory System** — Planned, not yet integrated
- **Tool Discovery** — Planned, not yet integrated
- **Action Confirmation** — Planned, not yet integrated
- **Real-time Dashboard** — Planned, not yet implemented

See [ROADMAP.md](ROADMAP.md) for details.

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

All examples run in isolated Docker containers with simulated robots (no ROS installation needed).

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

---

## Installation

### Via PyPI (Production)

```bash
pip install agent-ros-bridge
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

## Documentation

| Document | Description |
|----------|-------------|
| [User Manual](docs/USER_MANUAL.md) | Complete guide |
| [API Reference](docs/API_REFERENCE.md) | API documentation |
| [Native ROS](docs/NATIVE_ROS.md) | Ubuntu/ROS installation |
| [Multi-ROS](docs/MULTI_ROS.md) | Fleet management |
| [Docker vs Native](docs/DOCKER_VS_NATIVE.md) | Deployment comparison |
| [ROADMAP.md](ROADMAP.md) | Future plans |
| [POST_MORTEM.md](POST_MORTEM.md) | What we learned |

---

## Development

```bash
# Install dev dependencies
pip install -e ".[dev]"

# Run tests
pytest

# Build package
python -m build
```

---

## Author

**webthree549** <webthree549@gmail.com>

## License

[MIT License](LICENSE)

---

**Security is not optional. JWT auth always required.**
