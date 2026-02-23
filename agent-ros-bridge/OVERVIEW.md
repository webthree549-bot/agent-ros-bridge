# Agent ROS Bridge — Project Overview

**Status:** Pre-launch development  
**Mission:** Bridge AI agents to physical robots via ROS  
**Target:** Q2 2026 public launch

---

## Quick Links

| Document | Purpose |
|----------|---------|
| [README.md](README.md) | User documentation, quick start |
| [LAUNCH_STRATEGY.md](LAUNCH_STRATEGY.md) | Go-to-market plan, positioning |
| [TODO.md](TODO.md) | Detailed task list, sprint planning |
| [MCP_QUICKSTART.md](docs/MCP_QUICKSTART.md) | Claude Desktop integration |

---

## Project Status

### ✅ Complete (Ready for Use)

| Component | Status | Notes |
|-----------|--------|-------|
| Core bridge architecture | ✅ | ROSBridge, managers, registry |
| ROS1 connector | ✅ | Production-ready, no mocks |
| ROS2 connector | ✅ | Jazzy, Humble, Iron, Rolling support |
| WebSocket transport | ✅ | TLS, auth, CORS |
| gRPC transport | ✅ | Mutual TLS, client library |
| MCP server | ✅ | Claude Desktop integration |
| Configuration system | ✅ | YAML, env vars, auto-discovery |
| Security (JWT/TLS) | ✅ | Production-grade |

### 🚧 In Progress (This Sprint)

| Component | Status | ETA |
|-----------|--------|-----|
| Test suite | 🚧 | 1 week |
| CI/CD pipeline | 🚧 | 3 days |
| Docker images | 🚧 | 3 days |
| Documentation site | 📋 | 2 weeks |

### 📋 Planned (Post-Launch)

| Component | Status | Priority |
|-----------|--------|----------|
| Agent memory | 📋 | P0 |
| Tool discovery | 📋 | P0 |
| Action confirmation | 📋 | P0 |
| Multi-agent orchestration | 📋 | P1 |
| LangChain integration | 📋 | P1 |
| Prometheus metrics | 📋 | P1 |
| Real-time dashboard | 📋 | P2 |

---

## Architecture at a Glance

```
┌──────────────────────────────────────────────────────────────┐
│  AI AGENTS (Claude, OpenClaw, AutoGPT, LangChain)            │
└──────────────────────┬───────────────────────────────────────┘
                       │ MCP / WebSocket / gRPC
                       │ (TLS encrypted)
┌──────────────────────▼───────────────────────────────────────┐
│  AGENT ROS BRIDGE (edge/cloud)                               │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │  Transports │  │    Core     │  │     Connectors      │  │
│  │ • WebSocket │  │ • Actions   │  │ • ROS2 (rclpy)      │  │
│  │ • gRPC      │◄─┤ • Topics    │◄─┤ • ROS1 (rospy)      │  │
│  │ • MCP       │  │ • Sessions  │  │ • Remote ROS        │  │
│  └─────────────┘  └─────────────┘  └─────────────────────┘  │
└──────────────────────┬───────────────────────────────────────┘
                       │ ROS topics / DDS / roscore
┌──────────────────────▼───────────────────────────────────────┐
│  ROBOTS (ROS1/2 physical robots)                             │
└──────────────────────────────────────────────────────────────┘
```

---

## Why This Project Wins

### 1. **Built for Agentic AI**
Not just a ROS wrapper — designed specifically for AI agents:
- MCP native (Claude Desktop works out of the box)
- Multi-agent session management
- Planning to add memory, tool discovery, confirmation systems

### 2. **Production Security**
Most robotics tools ignore security:
- TLS for all transports
- JWT authentication
- Rate limiting hooks
- Audit logging

### 3. **Distributed by Design**
Bridge, agents, and robots can all be separate:
- Edge deployment (bridge near robots)
- Cloud deployment (bridge in K8s)
- Hybrid (agents remote, ROS local)

### 4. **ROS1 + ROS2**
One bridge handles both:
- Jazzy, Humble, Iron, Rolling (ROS2)
- Noetic (ROS1)
- Seamless migration path

---

## Getting Started (Right Now)

```bash
# Clone and install
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
cd agent-ros-bridge
pip install -e ".[all]"

# Run demo (no ROS required)
cd examples/actions
python3 actions_demo.py --mock

# Open http://localhost:8773
```

---

## Roadmap

### Phase 1: Foundation (Now - Week 4)
- ✅ Core architecture
- ✅ ROS1/2 connectors
- 🚧 Test suite
- 🚧 CI/CD
- 🚧 Docker images

### Phase 2: Agentic Features (Weeks 5-8)
- Agent memory
- Tool discovery
- Action confirmation
- Multi-agent coordination

### Phase 3: Ecosystem (Weeks 9-12)
- LangChain integration
- AutoGPT plugin
- Prometheus metrics
- Real-time dashboard

### Phase 4: Launch (Week 13+)
- Public release
- Community building
- Conference talks
- Partnerships

---

## Contributing

We need help with:
1. **Testing** — Write unit/integration tests
2. **ROS expertise** — Validate ROS1/2 connectors
3. **Agentic AI** — Design memory, planning features
4. **DevOps** — Docker, K8s, CI/CD
5. **Documentation** — Tutorials, examples

See [TODO.md](TODO.md) for specific tasks.

---

## Community

- **Discord:** [Join us](https://discord.gg/agent-ros-bridge)
- **Twitter:** [@AgentROSBridge](https://twitter.com/AgentROSBridge)
- **Discussions:** [GitHub Discussions](https://github.com/webthree549-bot/agent-ros-bridge/discussions)

---

## License

Apache 2.0 — See [LICENSE](LICENSE)

---

*Built with 🤖 for the agentic AI revolution*
