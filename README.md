# Agent ROS Bridge

**Universal ROS1/ROS2 bridge for AI agents to control robots and embodied intelligence systems.**

[![CI](https://github.com/webthree549-bot/agent-ros-bridge/actions/workflows/ci.yml/badge.svg)](https://github.com/webthree549-bot/agent-ros-bridge/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/agent-ros-bridge.svg)](https://pypi.org/project/agent-ros-bridge/)
[![Python](https://img.shields.io/badge/python-3.10%20%7C%203.11%20%7C%203.12-blue.svg)](https://pypi.org/project/agent-ros-bridge/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## What It Does

Agent ROS Bridge sits between your AI agent and your robots. It speaks WebSocket, MQTT, and gRPC on the agent side, and ROS1/ROS2 on the robot side — with JWT auth, agent memory, safety confirmation, and fleet orchestration built in.

```
┌─────────────────────────────────────────────────────────────┐
│                      AI AGENT LAYER                          │
│   LangChain · AutoGPT · Claude (MCP) · Custom Agents        │
└──────────────────────────┬──────────────────────────────────┘
                           │  WebSocket / MQTT / gRPC
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                   AGENT ROS BRIDGE                           │
│  ┌──────────────┐  ┌─────────────┐  ┌────────────────────┐  │
│  │  Transports  │  │  Core Bridge│  │  AI Integrations   │  │
│  │  WebSocket   │  │  JWT Auth   │  │  Memory (SQLite)   │  │
│  │  MQTT        │  │  RBAC       │  │  Safety Manager    │  │
│  │  gRPC*       │  │  Fleet Mgmt │  │  Tool Discovery    │  │
│  └──────────────┘  └─────────────┘  └────────────────────┘  │
└──────────────────────────┬──────────────────────────────────┘
                           │  rclpy / rospy
              ┌────────────┴────────────┐
              ▼                         ▼
       ROS2 (Jazzy/Humble)         ROS1 (Noetic)
```

> `*` gRPC transport: server starts, proto integration in progress.

---

## Feature Status

| Feature | Status | Notes |
|---------|--------|-------|
| WebSocket transport | ✅ Working | Full duplex, TLS support |
| MQTT transport | ✅ Working | paho v2 compatible |
| JWT authentication | ✅ Working | Per-transport, RBAC included |
| Agent memory (SQLite) | ✅ Working | TTL, append, list ops |
| Agent memory (Redis) | ✅ Working | Same API, Redis backend |
| Safety manager | ✅ Working | Emergency stop, policy levels |
| Fleet orchestration | ✅ Working | Task allocation, priority queue |
| LangChain adapter | ✅ Working | `ROSBridgeTool`, `ROSAgent` |
| MCP server (stdio) | ✅ Working | Claude Desktop integration |
| AutoGPT adapter | ✅ Working | Command discovery & execution |
| Web dashboard | ✅ Working | HTTP polling, emergency stop button |
| Simulated robot | ✅ Working | No ROS needed — perfect for testing |
| ROS2 connector | 🔧 In Progress | Connect/discover works; publish/subscribe being implemented |
| ROS1 connector | 🔧 In Progress | Structure complete; needs end-to-end testing |
| gRPC transport | 🔧 In Progress | Server starts; proto/service registration pending |
| Tool auto-discovery | 🔧 In Progress | Format converters ready; ROS introspection pending |

---

## Quick Start (no ROS required)

The fastest path uses the built-in **simulated robot** — a full bridge with a virtual TurtleBot that responds to commands over WebSocket, with zero ROS installation needed.

### 1. Install

```bash
pip install agent-ros-bridge
```

### 2. Set JWT secret

```bash
export JWT_SECRET=$(openssl rand -base64 32)
```

> The bridge will refuse to start without `JWT_SECRET`. This is intentional.

### 3. Start the simulated bridge

```bash
python examples/quickstart/simulated_robot.py
# WebSocket available at ws://localhost:8766
```

Or with Docker:

```bash
docker-compose -f examples/quickstart/docker-compose.yml up
```

### 4. Generate a token and connect

```bash
# Generate a JWT token
python scripts/generate_token.py --user myagent --roles operator

# Connect with wscat (npm install -g wscat)
wscat -c "ws://localhost:8766?token=<TOKEN>"
```

### 5. Send commands

```json
{"command": {"action": "list_robots"}}
{"command": {"action": "get_topics"}}
{"command": {"action": "move", "parameters": {"direction": "forward", "distance": 1.0}}}
{"command": {"action": "rotate", "parameters": {"angle": 90}}}
```

---

## Python API

### Basic bridge

```python
import asyncio, os
from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

os.environ["JWT_SECRET"] = "your-secret"   # or set in env before importing

async def main():
    bridge = Bridge()
    bridge.transport_manager.register(WebSocketTransport({"port": 8765}))
    async with bridge.run():
        print("Bridge running on ws://localhost:8765")
        while bridge.running:
            await asyncio.sleep(1)

asyncio.run(main())
```

### With LangChain

```python
from agent_ros_bridge import Bridge

bridge = Bridge()

# Get a LangChain-compatible tool
tool = bridge.get_langchain_tool(actions=["navigate", "move_arm", "get_status"])

# Plug into any LangChain agent
from langchain_core.tools import BaseTool
from langchain.agents import AgentExecutor, create_react_agent

agent = create_react_agent(llm, [tool], prompt)
executor = AgentExecutor(agent=agent, tools=[tool])
result = executor.invoke({"input": "Navigate the robot to zone A"})
```

### With Claude Desktop (MCP)

```python
from agent_ros_bridge import Bridge

bridge = Bridge()
mcp = bridge.get_mcp_server(mode="stdio")
await mcp.start()   # Claude Desktop can now control robots
```

Add to your `claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "agent-ros-bridge": {
      "command": "python",
      "args": ["-m", "agent_ros_bridge.integrations.mcp_transport"],
      "env": {
        "JWT_SECRET": "your-secret-here"
      }
    }
  }
}
```

### With AutoGPT

```python
from agent_ros_bridge import Bridge

bridge = Bridge()
adapter = bridge.get_autogpt_adapter()

# List available commands
commands = adapter.get_commands()

# Execute a command
result = await adapter.execute_command("ros_navigate", target="zone_a")
```

### Agent memory

```python
bridge = Bridge(config={"memory_backend": "sqlite", "memory_path": "/tmp/bridge.db"})

# Memory is automatically attached to Bridge.memory
await bridge.memory.set("last_position", {"x": 3.0, "y": 1.5}, ttl=3600)
pos = await bridge.memory.get("last_position")

# Append to a history log
await bridge.memory.append("agent:user1:actions", {"action": "navigate", "ts": "..."})
```

### Safety confirmation

```python
from agent_ros_bridge.integrations.safety import SafetyManager, SafetyLevel

safety = SafetyManager()
safety.register_policy("move_arm", SafetyLevel.DANGEROUS, "Arm movement requires confirmation")

# Register a callback to receive confirmation requests (e.g. send to operator dashboard)
def on_confirm_request(request):
    print(f"Operator approval needed: {request.id} — {request.message}")

safety.on_confirmation_request(on_confirm_request)

# Approve via request ID
await safety.confirm(request_id)

# Emergency stop
safety.trigger_emergency_stop("Obstacle detected")
```

### Fleet orchestration

```python
from agent_ros_bridge.fleet import FleetOrchestrator, FleetRobot, RobotCapability, Task

orchestrator = FleetOrchestrator()
await orchestrator.start()

# Register robots
await orchestrator.add_robot(FleetRobot(
    robot_id="tb4_01", name="TurtleBot4 #1",
    capabilities=RobotCapability(can_navigate=True, battery_hours=4.0)
))

# Submit tasks
task_id = await orchestrator.submit_task(Task(
    type="navigate", target_location="zone_b", priority=3
))

metrics = orchestrator.get_metrics()
print(f"Fleet: {metrics.idle_robots} idle, {metrics.active_robots} active")
```

---

## Docker

### Development / CI (no ROS)

```bash
# Build
docker build -t agent-ros-bridge .

# Run with JWT secret
docker run --rm -e JWT_SECRET=$JWT_SECRET -p 8765:8765 agent-ros-bridge
```

### With ROS2 Jazzy

```bash
docker build -f docker/Dockerfile.ros2 -t agent-ros-bridge:ros2 .
docker run --rm \
  -e JWT_SECRET=$JWT_SECRET \
  -e ROS_DOMAIN_ID=0 \
  -p 8765:8765 \
  agent-ros-bridge:ros2
```

### docker-compose (profile-based)

```bash
# ROS2 bridge only
JWT_SECRET=$JWT_SECRET docker-compose --profile ros2 up

# ROS1 bridge + roscore
JWT_SECRET=$JWT_SECRET docker-compose --profile ros1 up

# ROS2 with TurtleBot4 simulator
JWT_SECRET=$JWT_SECRET docker-compose --profile ros2 --profile sim up
```

---

## Connecting to a Real Robot (ROS2)

> ROS2 publish/subscribe implementation is in progress. The following shows the intended workflow:

```bash
# 1. Install with ROS2 support
pip install "agent-ros-bridge[ros2]"

# 2. Source your ROS2 environment
source /opt/ros/jazzy/setup.bash   # or humble

# 3. Set config pointing at your ROS domain
cat > config/gateway.yaml << EOF
name: my_robot_bridge
transports:
  websocket:
    port: 8765
    auth:
      enabled: true
      jwt_secret: ${JWT_SECRET}
connectors:
  ros2:
    enabled: true
    options:
      domain_id: 0
EOF

# 4. Start
agent-ros-bridge --config config/gateway.yaml
```

---

## CLI Reference

```bash
# Start with default config (WebSocket on 8765, gRPC on 50051)
agent-ros-bridge

# Start with a config file
agent-ros-bridge --config config/gateway.yaml

# Demo mode — WebSocket + greenhouse plugin, no ROS needed
agent-ros-bridge --demo

# Override ports
agent-ros-bridge --websocket-port 9000 --grpc-port 50052

# Print version
agent-ros-bridge --version
```

---

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `JWT_SECRET` | — | **Required.** JWT signing secret (min 32 chars). Generate: `openssl rand -base64 32` |
| `BRIDGE_CONFIG` | — | Path to YAML/JSON config file |
| `BRIDGE_LOG_LEVEL` | `INFO` | Log level: `DEBUG`, `INFO`, `WARNING`, `ERROR` |
| `BRIDGE_WEBSOCKET_PORT` | `8765` | WebSocket listen port |
| `BRIDGE_GRPC_PORT` | `50051` | gRPC listen port |

---

## Examples

| Example | Description | Run |
|---------|-------------|-----|
| [`examples/quickstart/`](examples/quickstart/) | Simulated robot — zero ROS needed | `docker-compose up` |
| [`examples/fleet/`](examples/fleet/) | 4-robot fleet with task scheduling | `./run.sh` |
| [`examples/auth/`](examples/auth/) | JWT auth, API keys, RBAC | `./run.sh` |
| [`examples/arm/`](examples/arm/) | Robot arm control (UR, xArm, Franka) | `./run.sh` |
| [`examples/actions/`](examples/actions/) | ROS Actions (nav2, manipulation) | `./run.sh` |
| [`examples/mqtt_iot/`](examples/mqtt_iot/) | MQTT IoT devices + bridge | `docker-compose up` |
| [`examples/metrics/`](examples/metrics/) | Prometheus metrics + Grafana | `docker-compose up` |
| [`examples/v0.5.0_integrations/`](examples/v0.5.0_integrations/) | LangChain, AutoGPT, MCP, Dashboard | `python <example>.py` |

---

## Project Structure

```
agent_ros_bridge/
│
├── gateway_v2/              # Core bridge
│   ├── core.py              # Bridge class — transport/connector/plugin lifecycle
│   ├── auth.py              # JWT authentication + RBAC
│   ├── config.py            # YAML/JSON/env config loader  (BRIDGE_* env vars)
│   ├── __main__.py          # CLI entry point  (agent-ros-bridge)
│   │
│   ├── transports/
│   │   ├── websocket.py     # WebSocket transport  ✅
│   │   ├── mqtt_transport.py# MQTT transport        ✅
│   │   └── grpc_transport.py# gRPC transport        🔧 in progress
│   │
│   ├── connectors/
│   │   ├── ros2_connector.py# ROS2 (rclpy)          🔧 in progress
│   │   └── ros1_connector.py# ROS1 (rospy)          🔧 in progress
│   │
│   └── plugins/
│       └── greenhouse_plugin.py  # Example plugin    ✅
│
├── integrations/            # AI agent integrations
│   ├── memory.py            # AgentMemory — SQLite/Redis with TTL  ✅
│   ├── safety.py            # SafetyManager — confirmation + e-stop ✅
│   ├── discovery.py         # ToolDiscovery — MCP/OpenAI export    🔧
│   ├── langchain_adapter.py # ROSBridgeTool, ROSAgent               ✅
│   ├── autogpt_adapter.py   # AutoGPT command adapter               ✅
│   ├── mcp_transport.py     # MCP stdio server                      ✅
│   └── dashboard_server.py  # aiohttp web dashboard                 ✅
│
├── fleet/
│   └── orchestrator.py      # FleetOrchestrator — multi-robot tasks ✅
│
├── plugins/
│   └── arm_robot.py         # UR / xArm / Franka arm plugin         🔧
│
└── actions/
    └── __init__.py          # ROS Action client (ROS2 + simulated)  ✅
```

---

## Security

- `JWT_SECRET` is always required — the bridge refuses to start without it.
- Authentication is opt-in per transport via config (`auth.enabled: true`). The simulated quickstart runs without auth for ease of development.
- RBAC roles: `admin` (all), `operator` (navigate/publish/subscribe), `viewer` (read-only).
- TLS: pass `tls_cert` / `tls_key` in transport config.
- Generate tokens: `python scripts/generate_token.py --user robot_agent --roles operator`

---

## Development

```bash
# Install dev + test dependencies
pip install -e ".[dev,test]"

# Run all tests
pytest

# Run specific suites
pytest tests/unit/ -v
pytest tests/integrations/ -v        # SafetyManager, Memory (no ROS needed)
pytest tests/integration/ -v         # requires running bridge

# Lint + format
ruff check .
black --check .

# Build package
python -m build
```

---

## Documentation

| Document | Description |
|----------|-------------|
| [docs/ARCHITECTURE_V2.md](docs/ARCHITECTURE_V2.md) | Three-layer gateway architecture |
| [docs/API_REFERENCE.md](docs/API_REFERENCE.md) | Full Python API reference |
| [docs/NATIVE_ROS.md](docs/NATIVE_ROS.md) | Native Ubuntu/ROS installation |
| [docs/MULTI_ROS.md](docs/MULTI_ROS.md) | Multi-robot / multi-ROS-version setup |
| [docs/DOCKER_VS_NATIVE.md](docs/DOCKER_VS_NATIVE.md) | Deployment trade-offs |
| [docs/DDS_ARCHITECTURE.md](docs/DDS_ARCHITECTURE.md) | ROS2 DDS / domain isolation |
| [docs/troubleshooting.md](docs/troubleshooting.md) | Common issues and solutions |
| [CHANGELOG.md](CHANGELOG.md) | Release history |
| [CONTRIBUTING.md](CONTRIBUTING.md) | How to contribute |
| [REVIEW.md](REVIEW.md) | Static + functional code review |

---

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). Priority areas for contribution:

- **ROS2 Connector** — `ros2_connector.py`: complete `_cmd_publish()`, `subscribe()`, and `_ros_msg_to_dict()` using `rosidl_runtime_py`
- **gRPC Transport** — extract `proto/bridge.proto`, generate stubs, register service
- **Safety approval endpoint** — add a WebSocket command (`safety.confirm`) so operators can approve dangerous actions from the agent side
- **Tool auto-discovery** — implement `_discover_topics/services/actions()` in `integrations/discovery.py`

---

## License

[MIT License](LICENSE) — © Agent ROS Bridge Contributors
