# Agent ROS Bridge

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
python examples/quickstart/mock_bridge.py  # Simulated robot
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
cd examples/quickstart && ./run.sh

# Start dashboard
python dashboard/server.py
# Open http://localhost:8080 in browser
```

### Fleet Orchestration (Multi-Robot)
```bash
# Start fleet orchestrator with 4 simulated robots
cd examples/fleet && ./run.sh

# Query fleet status
wscat -c ws://localhost:8771
> {"command": {"action": "fleet.status"}}
> {"command": {"action": "fleet.metrics"}}
> {"command": {"action": "fleet.submit_task", "parameters": {"type": "navigate", "target": "zone_a"}}}
```

### Arm Robot Control (Manipulation)
```bash
# Control UR5 arm
cd examples/arm && ./run.sh --arm-type ur --demo pick_place

# Control xArm
cd examples/arm && ./run.sh --arm-type xarm

# Interactive control
cd examples/arm && ./run.sh --arm-type ur --demo interactive
# Then: wscat -c ws://localhost:8772
```

### ROS Actions (Navigation, Planning)
```bash
# Navigation action demo
cd examples/actions && ./run.sh --action navigate

# Manipulation trajectory demo
cd examples/actions && ./run.sh --action manipulate

# Interactive action control
cd examples/actions && ./run.sh --action interactive
# Then: wscat -c ws://localhost:8773
# Send: {"command": {"action": "actions.navigate", "parameters": {"x": 5.0, "y": 3.0}}}
```

### Prometheus Metrics (Monitoring)
```bash
# Start metrics server
cd examples/metrics && ./run.sh

# View metrics
curl http://localhost:9090/metrics

# For Grafana:
# 1. Import dashboards/grafana-dashboard.json
# 2. Add Prometheus data source: http://localhost:9090
```

## Project Structure

```
agent-ros-bridge/                 # Repository root
├── agent_ros_bridge/            # ⭐ Core source package
│   ├── gateway_v2/              # Main gateway implementation
│   │   ├── core.py              # Bridge class
│   │   ├── auth.py              # Authentication
│   │   ├── transports/          # Communication protocols
│   │   └── connectors/          # ROS connectors
│   ├── fleet/                   # Fleet orchestration
│   ├── plugins/                 # Robot plugins (arm, etc.)
│   ├── actions/                 # ROS actions support
│   └── metrics/                 # Prometheus metrics
├── tests/                       # Test suite
├── examples/                    # ⭐ Runnable examples (7 demos)
│   ├── quickstart/              #   Basic bridge usage
│   ├── fleet/                   #   Multi-robot coordination
│   ├── auth/                    #   JWT authentication
│   ├── mqtt_iot/                #   IoT sensor integration
│   ├── actions/                 #   ROS navigation/actions
│   ├── arm/                     #   Robotic arm control
│   └── metrics/                 #   Prometheus monitoring
├── docs/                        # Documentation (40,000+ words)
├── scripts/                     # Utility scripts
├── config/                      # Configuration templates
├── docker/                      # Docker containers
├── dashboard/                   # Web dashboard
├── Makefile                     # Build automation
└── pyproject.toml              # Package configuration
```

See [Repository Structure](docs/REPOSITORY_STRUCTURE.md) for complete details.

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

## Development

The repository maintains strict separation between **source code** and **build artifacts**:

```bash
# Clone and setup
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
cd agent-ros-bridge

# Install in development mode
make install-dev

# Development workflow
make format      # Format code
make test        # Run tests
make check       # Lint + test

# Clean build artifacts
make clean       # Remove all generated files
make build       # Create wheel and sdist
```

See [CONTRIBUTING.md](CONTRIBUTING.md) and [Repository Structure](docs/REPOSITORY_STRUCTURE.md) for details.

## Documentation

| Document | Description |
|----------|-------------|
| **[User Manual](docs/USER_MANUAL.md)** | Complete guide (23,000+ words) - Installation, tutorials, troubleshooting |
| **[API Reference](docs/API_REFERENCE.md)** | Full API docs - Classes, methods, examples |
| **[Native ROS](docs/NATIVE_ROS.md)** | Ubuntu/ROS installation and setup |
| **[Multi-ROS](docs/MULTI_ROS.md)** | Fleet management and coordination |
| **[Docker vs Native](docs/DOCKER_VS_NATIVE.md)** | Deployment strategy comparison |
| **[DDS Architecture](docs/DDS_ARCHITECTURE.md)** | ROS2/DDS relationship explained |
| **[Repository Structure](docs/REPOSITORY_STRUCTURE.md)** | Clean source/build separation |

**Quick Links:**
- [Installation Guide](docs/USER_MANUAL.md#installation)
- [Docker vs Native Decision Matrix](docs/DOCKER_VS_NATIVE.md#decision-matrix)
- [Repository Structure](docs/REPOSITORY_STRUCTURE.md)
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

## License

[MIT License](LICENSE)

---

Made with ❤️ by the Agent ROS Bridge Team
