# Agent ROS Bridge

> 🎉 **v0.5.0 Coming Soon** — AI agent integrations are being rebuilt properly into gateway_v2.  
> See [ROADMAP.md](ROADMAP.md) and [v0.5.0_PLAN.md](v0.5.0_PLAN.md) for details.

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

## ✅ What's Working (v0.4.1 / v0.5.0-alpha)

### Core ROS Bridge
- **Multi-Protocol** — WebSocket, gRPC, MQTT transports
- **ROS1 & ROS2** — Full compatibility
  - ROS2: Jazzy Jalisco (LTS), Humble Hawksbill (LTS), Iron Irwini, Rolling
  - ROS1: Noetic Ninjemys
- **Security** — JWT (required), TLS, mTLS
- **Fleet Orchestration** — Multi-robot coordination
- **Docker Examples** — Isolated testing environments
- **Prometheus Metrics** — Basic observability

### 🚧 AI Agent Integrations (v0.5.0 Preview)

Code exists in `agent_ros_bridge/integrations/`:

- **Agent Memory** — SQLite/Redis backends with TTL
- **Safety Manager** — Action confirmation, emergency stop
- **Tool Discovery** — Auto-discover ROS, export to MCP/OpenAI
- **LangChain Adapter** — ROSBridgeTool, ROSAgent
- **AutoGPT Adapter** — AutoGPT plugin support
- **MCP Transport** — Model Context Protocol server
- **Dashboard** — Real-time web UI

**Status:** Implemented but not yet integrated into gateway_v2 core. Integration in progress.

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

```bash
# Clone repository
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
cd agent-ros-bridge

# Generate JWT secret
export JWT_SECRET=$(openssl rand -base64 32)

# Run example in Docker
cd examples/quickstart
docker-compose up
```

### Available Examples

| Example | Description |
|---------|-------------|
| `examples/quickstart/` | Basic bridge |
| `examples/fleet/` | Multi-robot fleet |
| `examples/arm/` | Robot arm control |

---

## Installation

### Via PyPI

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

### With AI Features (v0.5.0)

```python
from agent_ros_bridge import Bridge
from agent_ros_bridge.integrations import AgentMemory, SafetyManager, ROSBridgeTool

# Create bridge with AI features
bridge = Bridge()
bridge.memory = AgentMemory()
bridge.safety = SafetyManager()

# Use with LangChain
from langchain.agents import initialize_agent
tool = ROSBridgeTool(bridge)
agent = initialize_agent([tool], llm, agent="zero-shot-react-description")
```

---

## Documentation

| Document | Description |
|----------|-------------|
| [ROADMAP.md](ROADMAP.md) | Future plans and honest assessment |
| [v0.5.0_PLAN.md](v0.5.0_PLAN.md) | Detailed v0.5.0 execution plan |
| [POST_MORTEM.md](POST_MORTEM.md) | What we learned from v0.4.0 |
| [FEATURE_AUDIT.md](FEATURE_AUDIT.md) | Complete feature audit |
| [docs/](docs/) | Full documentation |

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
