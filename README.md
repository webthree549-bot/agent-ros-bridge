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

### Native ROS (Recommended for Development)

**Single ROS Version (auto-detect):**
```bash
# One-line installer
curl -sSL https://raw.githubusercontent.com/webthree549-bot/agent-ros-bridge/main/scripts/install-native.sh | bash

# Run with native ROS
source /opt/ros/humble/setup.bash  # or noetic, jazzy
python run_bridge.py
```

**Dual ROS1 + ROS2 (simultaneous):**
```bash
# Source both ROS environments
source /opt/ros/noetic/setup.bash
source /opt/ros/humble/setup.bash

# Run dual bridge
python run_bridge_dual_ros.py
```

See [docs/NATIVE_ROS.md](docs/NATIVE_ROS.md) for detailed native installation.

### Web Dashboard
```bash
# Start bridge (in another terminal)
python demo/mock_bridge.py

# Start dashboard
python dashboard/server.py
# Open http://localhost:8080 in browser
```

### Fleet Orchestration (Multi-Robot)
```bash
# Start fleet orchestrator with 4 simulated robots
python demo/fleet_demo.py

# Query fleet status
wscat -c ws://localhost:8771
> {"command": {"action": "fleet.status"}}
> {"command": {"action": "fleet.metrics"}}
> {"command": {"action": "fleet.submit_task", "parameters": {"type": "navigate", "target": "zone_a"}}}
```

### Arm Robot Control (Manipulation)
```bash
# Control UR5 arm
python demo/arm_demo.py --arm-type ur --ros-version ros2 --demo pick_place

# Control xArm
python demo/arm_demo.py --arm-type xarm --ros-version ros2

# Interactive control
python demo/arm_demo.py --arm-type ur --ros-version ros2 --demo interactive
# Then: wscat -c ws://localhost:8772
```

### ROS Actions (Navigation, Planning)
```bash
# Navigation action demo
python demo/actions_demo.py --action navigate

# Manipulation trajectory demo
python demo/actions_demo.py --action manipulate

# Interactive action control
python demo/actions_demo.py --action interactive
# Then: wscat -c ws://localhost:8773
# Send: {"command": {"action": "actions.navigate", "parameters": {"x": 5.0, "y": 3.0}}}
```

### Prometheus Metrics (Monitoring)
```bash
# Start metrics server
python demo/metrics_demo.py

# View metrics
curl http://localhost:9090/metrics

# For Grafana:
# 1. Import dashboards/grafana-dashboard.json
# 2. Add Prometheus data source: http://localhost:9090
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
- **Fleet Orchestration**: Task allocation, load balancing, coordination
- **Arm Robot Control**: UR, xArm, Franka manipulation support
- **ROS Actions**: Navigation, motion planning, long-running tasks
- **Prometheus Metrics**: Production monitoring with Grafana dashboards
- **Multi-ROS**: Connect to multiple ROS1/ROS2 endpoints simultaneously
- **Dual ROS**: Run ROS1 + ROS2 in the same bridge instance
- **Remote ROS**: Connect to robots over the network
- **Plugin System**: Build custom applications
- **ROS Support**: ROS1 Noetic, ROS2 Humble/Jazzy/Iron
- **Cloud-Native**: Docker, Kubernetes ready

## Documentation

| Document | Description |
|----------|-------------|
| **[User Manual](docs/USER_MANUAL.md)** | Complete guide (23,000+ words) - Installation, tutorials, troubleshooting |
| **[API Reference](docs/API_REFERENCE.md)** | Full API docs - Classes, methods, examples |
| **[Native ROS](docs/NATIVE_ROS.md)** | Ubuntu/ROS installation and setup |
| **[Multi-ROS](docs/MULTI_ROS.md)** | Fleet management and coordination |
| **[Architecture](docs/ARCHITECTURE.md)** | System design (placeholder) |

**Quick Links:**
- [Installation Guide](docs/USER_MANUAL.md#installation)
- [WebSocket Protocol](docs/API_REFERENCE.md#websocket-protocol)
- [Fleet Management](docs/USER_MANUAL.md#fleet-management)
- [Troubleshooting](docs/USER_MANUAL.md#troubleshooting)

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
