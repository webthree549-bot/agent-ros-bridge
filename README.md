# Agent ROS Bridge

> 🎉 **v0.5.0 Released** — Complete AI agent integration with ROS robots.  
> After the honest v0.4.1 reset, v0.5.0 delivers on the original vision.

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

## ✅ What's Working (v0.5.0)

### Core ROS Bridge
- **Multi-Protocol** — WebSocket, gRPC, MQTT transports
- **ROS1 & ROS2** — Full compatibility
  - ROS2: Jazzy Jalisco (LTS), Humble Hawksbill (LTS), Iron Irwini, Rolling
  - ROS1: Noetic Ninjemys
- **Security** — JWT (required), TLS, mTLS
- **Fleet Orchestration** — Multi-robot coordination
- **Docker Examples** — Isolated testing environments
- **Prometheus Metrics** — Basic observability

### 🤖 AI Agent Integrations (v0.5.0 — Fully Integrated)

All features now **fully integrated** into gateway_v2:

| Feature | Status | Description |
|---------|--------|-------------|
| **Agent Memory** | ✅ Working | SQLite/Redis backends with TTL |
| **Safety Manager** | ✅ Working | Action confirmation, emergency stop |
| **Tool Discovery** | ✅ Working | Auto-discover ROS, MCP/OpenAI export |
| **LangChain** | ✅ Working | ROSBridgeTool, ROSAgent |
| **AutoGPT** | ✅ Working | Native plugin adapter |
| **MCP** | ✅ Working | Model Context Protocol (Claude Desktop) |
| **Dashboard** | ✅ Working | Real-time web UI |

---

## 🚀 Quick Start

### Installation

```bash
pip install agent-ros-bridge
```

### Set JWT Secret (Required)

```bash
export JWT_SECRET=$(openssl rand -base64 32)
```

### With LangChain

```python
from agent_ros_bridge import Bridge
from langchain.agents import initialize_agent

# Create bridge
bridge = Bridge()

# Get LangChain tool
tool = bridge.get_langchain_tool(["navigate", "move_arm"])

# Use with LangChain
agent = initialize_agent([tool], llm, agent="zero-shot-react-description")
agent.run("Navigate the robot to position (5, 3)")
```

### With AutoGPT

```python
from agent_ros_bridge import Bridge

bridge = Bridge()
adapter = bridge.get_autogpt_adapter()

# Get AutoGPT commands
commands = adapter.get_commands()
```

### MCP Server (Claude Desktop)

```python
from agent_ros_bridge import Bridge

bridge = Bridge()
mcp = bridge.get_mcp_server(mode="stdio")

# Start MCP server
await mcp.start()  # Claude Desktop can now control robots
```

### Dashboard

```python
from agent_ros_bridge import Bridge

bridge = Bridge()
dashboard = bridge.get_dashboard(port=8080)

# Start dashboard
await dashboard.start()  # http://localhost:8080
```

---

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [CHANGELOG.md](CHANGELOG.md) | Release notes |
| [ROADMAP.md](ROADMAP.md) | Future plans |
| [POST_MORTEM.md](POST_MORTEM.md) | Lessons learned from v0.4.0 |
| [FEATURE_AUDIT.md](FEATURE_AUDIT.md) | Complete feature analysis |
| [ACCOMPLISHMENTS.md](ACCOMPLISHMENTS.md) | What we built |
| [examples/v0.5.0_integrations/](examples/v0.5.0_integrations/) | Usage examples |

---

## 🏗️ Architecture

```python
agent_ros_bridge/
├── gateway_v2/          # Core bridge
│   ├── core.py         # Bridge class with AI integrations
│   ├── transports/     # WebSocket, gRPC, MQTT
│   └── connectors/     # ROS1, ROS2
│
└── integrations/        # AI features (v0.5.0)
    ├── memory.py       # Agent memory
    ├── safety.py       # Safety confirmation
    ├── discovery.py    # Tool discovery
    ├── langchain_adapter.py    # LangChain
    ├── autogpt_adapter.py      # AutoGPT
    ├── mcp_transport.py        # MCP protocol
    └── dashboard_server.py     # Web UI
```

**Key:** Everything properly wired together. No orphaned code.

---

## 🧪 Testing

```bash
# Run all tests
pytest

# Run integration tests
pytest tests/integrations/
```

---

## 📝 Examples

See [examples/v0.5.0_integrations/](examples/v0.5.0_integrations/):

- `langchain_example.py` — LangChain integration
- `autogpt_example.py` — AutoGPT integration
- `mcp_example.py` — MCP server for Claude Desktop
- `dashboard_example.py` — Web dashboard

---

## 🔧 Development

```bash
# Install dev dependencies
pip install -e ".[dev]"

# Run tests
pytest

# Build package
python -m build
```

---

## 📊 Version History

| Version | Date | Status |
|---------|------|--------|
| v0.5.0 | 2026-02-23 | ✅ Current — Full AI integration |
| v0.4.1 | 2026-02-23 | ✅ Honest release (cleanup) |
| v0.4.0 | 2026-02-23 | ⚠️ Retracted (false claims) |

---

## 👤 Author

**webthree549** <webthree549@gmail.com>

## 📄 License

[MIT License](LICENSE)

---

**Security is not optional. JWT auth always required.**
