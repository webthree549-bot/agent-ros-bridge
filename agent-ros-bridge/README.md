# Agent ROS Bridge

[![PyPI](https://img.shields.io/pypi/v/agent-ros-bridge.svg)](https://pypi.org/project/agent-ros-bridge/)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![macOS](https://img.shields.io/badge/macOS-supported-brightgreen.svg)](docs/MACOS.md)
[![ROS](https://img.shields.io/badge/ROS-apt%20install-blue.svg)](docs/ROS_RELEASE.md)
[![ClawHub](https://img.shields.io/badge/ClawHub-Skill-blue)](https://clawhub.ai/skills/agent-ros-bridge)
[![OpenClaw](https://img.shields.io/badge/OpenClaw-Integrated-orange)](https://openclaw.ai)

A secure, multi-agent bridge for ROS1/2 robot control. Connect AI agents like Claude Desktop to ROS robots via the Model Context Protocol (MCP), WebSocket, or gRPC.

## Features

- 🔗 **Multi-Agent Support** — Multiple AI agents can connect simultaneously
- 🌐 **Multiple Transports** — WebSocket, gRPC, and MCP interfaces
- 🤖 **ROS1 & ROS2** — Compatible with both ROS versions
  - ROS2: Jazzy Jalisco (LTS), Humble Hawksbill (LTS), Iron Irwini, Rolling
  - ROS1: Noetic Ninjemys
- 🍎 **macOS Support** — Homebrew, Docker, RoboStack, or native pip
- 📦 **ROS Community Release** — `apt install ros-${ROS_DISTRO}-agent-ros-bridge`
- 🔐 **Secure** — JWT authentication and session management
- 🧩 **Plugin Architecture** — Easy to extend with custom actions and topics

## Quick Start

### Installation

```bash
# Basic install (no ROS - for client-only use)
pip install agent-ros-bridge

# With MCP support (for Claude Desktop)
pip install agent-ros-bridge[mcp]

# With ROS2 support
pip install agent-ros-bridge[mcp,ros2]

# With all optional dependencies (except ROS1)
pip install agent-ros-bridge[all]
```

### Platform Support

| Platform | Status | Installation |
|----------|--------|--------------|
| **Linux (Ubuntu)** | ✅ Full support | apt, pip, Docker |
| **macOS** | ✅ Supported | pip, Homebrew, Docker |
| **Windows** | ⚠️ Experimental | WSL2 recommended |

### ROS1/2 Setup

| Distribution | Codename | Status | rclpy | Python | Platforms |
|-------------|----------|--------|-------|--------|-----------|
| ROS2 | **Jazzy Jalisco** | ✅ LTS (recommended) | 7.x | 3.12 | Ubuntu 24.04 |
| ROS2 | **Humble Hawksbill** | ✅ LTS | 3.x | 3.10 | Ubuntu 22.04, macOS* |
| ROS2 | **Iron Irwini** | ✅ Supported | 6.x | 3.10 | Ubuntu 22.04 |
| ROS2 | Rolling Ridley | ✅ Development | latest | 3.12 | Ubuntu 24.04 |
| ROS1 | **Noetic Ninjemys** | ✅ LTS | N/A | 2.7/3.8+ | Ubuntu 20.04 |

\* macOS via Conda or Docker

**Linux (Ubuntu/Debian):**

```bash
# Jazzy Jalisco (LTS, Python 3.12, recommended)
source /opt/ros/jazzy/setup.bash

# Humble Hawksbill (LTS, Python 3.10)
source /opt/ros/humble/setup.bash

# ROS Community Release (when available)
sudo apt install ros-${ROS_DISTRO}-agent-ros-bridge
```

**macOS:**

```bash
# Option 1: Using Homebrew (recommended for macOS)
brew install openclaw/tap/agent-ros-bridge

# Option 2: Using Conda (for ROS2 support)
conda install -c conda-forge -c robostack ros-humble-desktop
pip install agent-ros-bridge[mcp]

# Option 3: Using Docker (no local ROS needed)
docker run -it --rm \
  -p 8765:8765 \
  -v ~/.openclaw:/config \
  ghcr.io/webthree549-bot/agent-ros-bridge:latest

# Option 4: Direct pip (client-only, no ROS)
pip install agent-ros-bridge
```

**ROS1 (rospy):**

```bash
# ROS1 must be installed from your distribution
# No pip package available
source /opt/ros/noetic/setup.bash
```

### macOS-Specific Notes

On macOS, ROS2 is not officially supported by Open Robotics, but works via:

1. **RoboStack (Conda)** — Most reliable:
   ```bash
   conda install -c conda-forge -c robostack ros-humble-desktop
   ```

2. **Docker Desktop** — Easiest setup:
   ```bash
   docker-compose up  # See docker-compose.macos.yml
   ```

3. **Native pip** — Client-only (connects to remote ROS):
   ```bash
   pip install agent-ros-bridge
   # Connect to remote ROS via rosbridge_server
   ```

Claude Desktop config location on macOS:
```
~/Library/Application Support/Claude/claude_desktop_config.json
```

### Basic Usage

```python
import asyncio
from agent_ros_bridge import ROSBridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
from agent_ros_bridge.gateway_v2.connectors.ros2 import ROS2Connector

# Create bridge
bridge = ROSBridge(ros_version=2)

# Add WebSocket transport
bridge.transport_manager.register(
    WebSocketTransport({"host": "0.0.0.0", "port": 8765})
)

# Add ROS2 connector
bridge.connector_manager.register(
    ROS2Connector({"domain_id": 0})
)

# Register actions
@bridge.action("navigate")
async def navigate(x: float, y: float, theta: float = 0.0):
    # Your navigation logic here
    return {"status": "success", "position": {"x": x, "y": y}}

# Start the bridge
asyncio.run(bridge.start())
```

### Claude Desktop Integration

1. Install with MCP support:
   ```bash
   pip install agent-ros-bridge[mcp]
   ```

2. Configure Claude Desktop (`~/Library/Application Support/Claude/claude_desktop_config.json`):
   ```json
   {
     "mcpServers": {
       "ros": {
         "command": "python3",
         "args": ["-m", "agent_ros_bridge.mcp"],
         "env": {
           "JWT_SECRET": "your-secret-key"
         }
       }
     }
   }
   ```

3. Restart Claude Desktop — ROS tools will appear automatically.

## Architecture

### Distributed Deployment

Agent ROS Bridge is designed for **distributed operation**. Components can run on separate machines:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              NETWORK                                         │
│  ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐       │
│  │   OpenClaw      │     │  Claude Desktop │     │   Other Agents  │       │
│  │   (Cloud/Edge)  │     │   (User Laptop) │     │   (Anywhere)    │       │
│  │                 │     │                 │     │                 │       │
│  │  • AI Agent     │     │  • MCP Client   │     │  • WebSocket    │       │
│  │  • Task Planner │     │  • Local Bridge │     │  • gRPC Client  │       │
│  └────────┬────────┘     └────────┬────────┘     └────────┬────────┘       │
│           │                       │                       │                │
│           │ gRPC/WebSocket        │ MCP (stdio)           │ WebSocket/gRPC │
│           │ (TLS)                 │                       │ (TLS)          │
│           │                       │                       │                │
│  ┌────────▼───────────────────────▼───────────────────────▼────────┐       │
│  │                     AGENT ROS BRIDGE                             │       │
│  │                    (Edge Server / Robot)                         │       │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │       │
│  │  │   gRPC      │  │  WebSocket  │  │   MCP       │  Transports  │       │
│  │  │   Server    │  │   Server    │  │   Server    │              │       │
│  │  │   (0.0.0.0) │  │   (0.0.0.0) │  │   (stdio)   │              │       │
│  │  └─────────────┘  └─────────────┘  └─────────────┘              │       │
│  │                                                                │       │
│  │  ┌─────────────────────────────────────────────────────────┐  │       │
│  │  │                  ROSBridge Core                         │  │       │
│  │  │  • Action Registry  • Topic Manager  • Session Manager  │  │       │
│  │  │  • Multi-Agent Auth • Rate Limiting  • Audit Logging    │  │       │
│  │  └─────────────────────────────────────────────────────────┘  │       │
│  │                                                                │       │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐   │       │
│  │  │   ROS2      │  │   ROS1      │  │   Remote ROS        │   │       │
│  │  │  Connector  │  │  Connector  │  │   (rosbridge_server)│   │       │
│  │  │  (Local)    │  │  (Local)    │  │   (Network)         │   │       │
│  │  └─────────────┘  └─────────────┘  └─────────────────────┘   │       │
│  └────────────────────────────────────────────────────────────────┘       │
│                              │                                             │
│                              │ ROS Network ( DDS / roscore )               │
│                              │                                             │
│  ┌───────────────────────────▼──────────────────────────────────┐        │
│  │                        ROS ROBOTS                              │        │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐   │        │
│  │  │   Robot 1   │  │   Robot 2   │  │   Robot N           │   │        │
│  │  │  (ROS2)     │  │  (ROS1)     │  │  (Any ROS)          │   │        │
│  │  └─────────────┘  └─────────────┘  └─────────────────────┘   │        │
│  └────────────────────────────────────────────────────────────────┘        │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Deployment Patterns

**Pattern 1: Edge Bridge (Recommended)**
- Bridge runs on edge device (Jetson, NUC) near robots
- OpenClaw/cloud connects remotely via WebSocket/gRPC over TLS
- Low latency for robot control, secure remote access

**Pattern 2: Local Bridge**
- Bridge runs on same machine as OpenClaw (MCP mode)
- Connects to remote ROS via `rosbridge_server` WebSocket
- Useful for development/testing

**Pattern 3: Cloud Bridge**
- Bridge runs in cloud/container
- Robots connect outbound to bridge (reverse connection)
- Useful for fleets with dynamic IPs/firewalls

### Network Security

For remote/distributed deployments, always use TLS:

```python
# WebSocket with TLS
WebSocketTransport({
    "host": "0.0.0.0",
    "port": 8765,
    "tls": {
        "cert": "/path/to/server.crt",
        "key": "/path/to/server.key"
    },
    "auth": {"enabled": True}
})

# gRPC with TLS
GRPCServer({
    "host": "0.0.0.0",
    "port": 50051,
    "tls": {
        "cert": "/path/to/server.crt",
        "key": "/path/to/server.key"
    }
})
```

## Project Structure

```
agent-ros-bridge/
├── agent_ros_bridge/
│   ├── __init__.py              # ROSBridge core
│   ├── config.py                # Configuration management
│   ├── gateway_v2/
│   │   ├── transports/          # WebSocket, gRPC servers/clients
│   │   ├── connectors/          # ROS1/2 connectors
│   │   └── auth/                # Authentication utilities
│   └── mcp/                     # MCP server implementation
├── config/
│   └── bridge.yaml.example      # Example distributed config
├── examples/
│   └── actions/                 # Demo dashboard
├── docs/                        # Documentation
└── pyproject.toml               # Package configuration
```

## Configuration

### Distributed Deployment Config

Create a `bridge.yaml` for production deployments:

```yaml
bridge:
  ros_version: 2
  log_level: INFO

transports:
  websocket:
    enabled: true
    host: 0.0.0.0
    port: 8765
    tls:
      cert: /etc/ssl/certs/bridge.crt
      key: /etc/ssl/private/bridge.key
    auth:
      enabled: true
      jwt_secret: ${JWT_SECRET}
  
  grpc:
    enabled: true
    host: 0.0.0.0
    port: 50051
    tls:
      cert: /etc/ssl/certs/bridge.crt
      key: /etc/ssl/private/bridge.key
      ca: /etc/ssl/certs/ca.crt  # For mutual TLS

connectors:
  ros2:
    enabled: true
    domain_id: 0
```

Load configuration:

```python
from agent_ros_bridge import load_config

# Auto-discover config from standard locations
config = load_config()

# Or specify path
config = load_config("/etc/agent-ros-bridge/bridge.yaml")

# Access values
ws_port = config.get('transports', 'websocket', 'port')
```

### Environment Variables

All config options can be set via environment:

| Variable | Description |
|----------|-------------|
| `BRIDGE_CONFIG` | Path to config file |
| `BRIDGE_ROS_VERSION` | ROS version (1 or 2) |
| `BRIDGE_WS_HOST` | WebSocket bind host |
| `BRIDGE_WS_PORT` | WebSocket port |
| `BRIDGE_WS_TLS_CERT` | TLS certificate path |
| `BRIDGE_WS_TLS_KEY` | TLS key path |
| `BRIDGE_JWT_SECRET` | JWT signing secret |
| `BRIDGE_ROS2_DOMAIN_ID` | ROS2 domain ID |
| `OPENCLAW_ENDPOINT` | Remote OpenClaw URL |
| `OPENCLAW_API_KEY` | OpenClaw API key |

## Remote Agent Connection

### OpenClaw Connecting to Remote Bridge

When OpenClaw runs separate from the bridge (cloud/edge deployment):

```python
from agent_ros_bridge.gateway_v2.transports import GRPCClient

# Connect to remote bridge
client = GRPCClient({
    "host": "bridge.robot.local",
    "port": 50051,
    "tls": {
        "enabled": True,
        "ca": "/path/to/ca.crt"  # Verify server certificate
    }
})

await client.connect()
channel = client.get_channel()
# Use channel for gRPC calls...
```

### Bridge Connecting to Remote ROS

When bridge runs separate from ROS (cloud/container deployment):

```python
from agent_ros_bridge import ROSBridge

# Bridge connects to remote rosbridge_server
bridge = ROSBridge(ros_version=2)

# Configure remote ROS connection
bridge.config['remote_ros'] = {
    "uri": "wss://ros-master.robot.local:9090",
    "auth": {"token": "${ROS_TOKEN}"}
}
```

## OpenClaw Integration (Privileged)

Agent ROS Bridge includes privileged integration with [OpenClaw](https://openclaw.ai) for cloud orchestration and fleet management.

### Features

- **Cloud Orchestration** — Centralized control from OpenClaw dashboard
- **Fleet Management** — Manage multiple robots from one interface
- **Centralized Logging** — All logs aggregated in OpenClaw cloud
- **Telemetry** — Real-time metrics and monitoring
- **Remote Commands** — Execute actions remotely from OpenClaw

### Quick Start

```python
from agent_ros_bridge import ROSBridge
from agent_ros_bridge.openclaw import connect_to_openclaw

bridge = ROSBridge(ros_version=2)

# Connect to OpenClaw cloud
integration = await connect_to_openclaw(
    bridge,
    api_key="${OPENCLAW_API_KEY}",
    bridge_id="warehouse-bot-01"
)

# Your bridge is now manageable from OpenClaw dashboard
```

### Configuration

```yaml
# bridge.yaml
openclaw:
  enabled: true
  endpoint: wss://api.openclaw.ai/v1/bridge
  api_key: ${OPENCLAW_API_KEY}
  bridge_id: ${HOSTNAME}
  telemetry_enabled: true
  heartbeat_interval: 30
```

### Install from ClawHub

```bash
# Install as OpenClaw Skill
clawhub skill install agent-ros-bridge

# Or via pip with OpenClaw support
pip install agent-ros-bridge[openclaw]
```

## Demo

Run the actions demo (no ROS required for basic test):

```bash
cd examples/actions

# Without ROS (simulation mode)
python3 actions_demo.py --mock

# With ROS2
python3 actions_demo.py --ros-version 2

# With ROS1
python3 actions_demo.py --ros-version 1
```

Then open http://localhost:8773 in your browser.

The demo provides:
- **Dashboard UI** at http://localhost:8773
- **WebSocket API** at ws://localhost:8765  
- **gRPC API** at localhost:50051

Example WebSocket message:
```json
{
  "type": "action",
  "action": "navigate",
  "params": {"x": 5.0, "y": 3.0, "theta": 0.0}
}
```

## Development

```bash
# Clone the repo
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
cd agent-ros-bridge

# Install in editable mode with dev dependencies
pip install -e ".[all,dev]"

# Run tests
pytest

# Format code
black agent_ros_bridge/
```

## License

MIT License — See [LICENSE](LICENSE) for details.

## Author

**webthree549** <webthree549@gmail.com>

## Contributing

Contributions welcome! Please read [CONTRIBUTING.md](CONTRIBUTING.md) first.

## Support

- 📖 [Documentation](https://docs.openclaw.ai/agent-ros-bridge)
- 🐛 [Issue Tracker](https://github.com/webthree549-bot/agent-ros-bridge/issues)
- 💬 [Discussions](https://github.com/webthree549-bot/agent-ros-bridge/discussions)
