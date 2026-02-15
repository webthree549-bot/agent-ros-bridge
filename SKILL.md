---
name: agent-ros-bridge
version: 0.3.3
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
        "requires": { "bins": ["python3"], "env": ["JWT_SECRET"] },
        "suggests": { "bins": ["docker"] },
        "env":
          {
            "JWT_SECRET":
              {
                "description": "Required: Secret key for JWT authentication. Bridge will fail to start without this.",
                "sensitive": true,
                "required": true
              },
            "BRIDGE_HOST":
              {
                "description": "Optional: Bind address (default: 127.0.0.1 for security)",
                "sensitive": false,
                "required": false
              },
          },
        "security":
          {
            "notes": "SECURITY-FIRST DESIGN: JWT authentication is always required and cannot be disabled. All examples run in Docker containers for isolation. Never expose to public networks without TLS and firewall rules.",
          },
        "install":
          [
            {
              "id": "pip",
              "kind": "pip",
              "package": "agent-ros-bridge",
              "label": "Agent ROS Bridge from PyPI"
            },
            {
              "id": "docker",
              "kind": "manual",
              "label": "Docker Desktop (optional but recommended)",
              "instruction": "For running examples in isolated containers. Install from https://www.docker.com/products/docker-desktop"
            }
          ],
        "category": "robotics",
        "tags": ["ros", "ros2", "robotics", "iot", "automation", "bridge", "embodied-intelligence", "arm", "navigation"],
      },
  }

---

# 🤖 Agent ROS Bridge

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

### Option 1: Docker Examples (Recommended for Testing)

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

### Available Examples

| Example | Description | Run |
|---------|-------------|-----|
| `examples/quickstart/` | Basic bridge with simulated robot | `docker-compose up` |
| `examples/fleet/` | Multi-robot fleet coordination | `docker-compose up` |
| `examples/arm/` | Robot arm control simulation | `docker-compose up` |

All examples:
- Run in isolated Docker containers
- Use JWT authentication (enforced)
- Include simulated robots (no hardware needed)
- Bind to localhost (127.0.0.1) by default

### Option 2: Native Installation (Production)

**Requirements:** Ubuntu 20.04/22.04 with ROS1 Noetic or ROS2 Humble/Jazzy

```bash
# Install from PyPI
pip install agent-ros-bridge

# Set required secret
export JWT_SECRET=$(openssl rand -base64 32)

# Start bridge
agent-ros-bridge --config config/bridge.yaml
```

See [docs/NATIVE_ROS.md](docs/NATIVE_ROS.md) for detailed native installation.

---

## Features

| Feature | Description |
|---------|-------------|
| **🔐 Security** | JWT auth always required, no bypass |
| **🤖 Multi-Robot** | Fleet orchestration & coordination |
| **🌐 Multi-Protocol** | WebSocket, MQTT, gRPC |
| **🔄 Multi-ROS** | ROS1 + ROS2 simultaneously |
| **🦾 Arm Control** | UR, xArm, Franka support |
| **📊 Monitoring** | Prometheus + Grafana |

---

## Documentation

| Document | Description |
|----------|-------------|
| [User Manual](docs/USER_MANUAL.md) | Complete guide (23,000+ words) |
| [API Reference](docs/API_REFERENCE.md) | Full API documentation |
| [Native ROS](docs/NATIVE_ROS.md) | Ubuntu/ROS installation |
| [Docker vs Native](docs/DOCKER_VS_NATIVE.md) | Deployment comparison |
| [SECURITY.md](SECURITY.md) | Security policy |

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

## Links

- **Documentation:** https://github.com/webthree549-bot/agent-ros-bridge/tree/main/docs
- **PyPI:** https://pypi.org/project/agent-ros-bridge/
- **GitHub:** https://github.com/webthree549-bot/agent-ros-bridge
- **Issues:** https://github.com/webthree549-bot/agent-ros-bridge/issues

---

**Security is not optional. JWT auth always required.**
