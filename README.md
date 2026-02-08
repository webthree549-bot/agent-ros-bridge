# openclaw-ros-bridge v1.0.0
[![CI Auto Test](https://github.com/webthree549-bot/openclaw-ros-bridge/actions/workflows/ci-auto-test.yml/badge.svg)](https://github.com/webthree549-bot/openclaw-ros-bridge/actions/workflows/ci-auto-test.yml)
[![Docker Build](https://github.com/webthree549-bot/openclaw-ros-bridge/actions/workflows/docker-build.yml/badge.svg)](https://github.com/webthree549-bot/openclaw-ros-bridge/actions/workflows/docker-build.yml)
[![MIT License](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Python 3.8+](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![ROS1 Noetic/ROS2 Humble/Jazzy](https://img.shields.io/badge/ROS-1%2F2-orange.svg)](https://www.ros.org/)

Universal ROS1/ROS2 + OpenClaw Adaptive Bridge Framework for **Embodied Intelligence**. A production-grade solution that connects OpenClaw AI Agents with ROS1/ROS2 ecosystems, supporting seamless hardware integration, real-time scheduling, fault resilience, and full-stack observability. Designed for embedded deployment (Jetson/RPi) and cross-platform development (Ubuntu/WSL2), with one-click automation and Docker isolation.

## Core Features
### 🔌 Full ROS Compatibility (Version Agnostic)
- **Native support for ROS1 Noetic / ROS2 Humble / ROS2 Jazzy** - fully version agnostic
- **Auto-detection**: Detects ROS version at runtime via environment or system inspection
- **Unified API**: Same code works across ROS1/ROS2 without modifications
- **Communication layer isolation**: rospy/rclpy implementation details hidden from business logic
- **Auto-adaptive build**: Catkin (ROS1) / Colcon (ROS2) automatically selected
- **Future-proof**: Adding new ROS distributions requires only config updates, no code changes

### 🤖 Seamless OpenClaw Integration (Version Agnostic)
- **OpenClaw v1.x / v2.x auto-adaptation** - runtime version detection from config
- **Protocol conversion**: ROS Message ↔ OpenClaw JSON (handles v1.x prefixing vs v2.x raw format)
- **TCP config auto-loading**: Different ports/timeouts per OpenClaw version (v1: 8888, v2: 9999)
- **Auto-reconnection & heartbeat** for stable AI Agent ↔ hardware communication
- **Business ID injection**: Automatic metadata tagging for multi-tenant deployments

### 🦿 Embodied Intelligence Critical Capabilities
- **Hardware Abstraction Layer (HAL)**: Unified sensor/actuator interface (temperature, camera, arm, motor, etc.)
- **Fault Self-Recovery**: Auto-reconnect/restart/safe state for communication/node/hardware failures
- **Real-Time Scheduling**: Linux SCHED_FIFO priority + CPU affinity binding (low-latency for embodied tasks)
- **Full-Stack Observability**: Graded logging (DEBUG-FATAL), performance monitoring (CPU/memory/latency), real-time dashboard, remote log streaming

### 🚀 Engineering & Deployment
- **One-Click Automation**: Build/Run/Test/Deploy scripts (auto-env detect, dependency install, compilation)
- **Cross-Platform Support**: Ubuntu 20.04/22.04/24.04 (x86_64/ARM64), Windows WSL2 (GUI/USB passthrough fixed)
- **Embedded Optimization**: Jetson (Xavier NX/AGX/Orin), Raspberry Pi 4/5 (power optimization + boot auto-start)
- **Docker Containerization**: Isolated ROS1/ROS2 containers, network bridging for OpenClaw/hardware, one-click start

### 🧪 Complete Test Suite
- Full coverage: Unit (core modules) + Integration (end-to-end) + Performance (latency/throughput) tests
- Mock mode (no hardware/OpenClaw required) for development/testing
- Auto-generated coverage reports (HTML) & CI/CD pipeline for parallel ROS1/ROS2 testing

### 📚 Open-Source Community Standards
- MIT License (commercial-friendly, no usage restrictions)
- Complete community docs: CONTRIBUTING.md, CODE_OF_CONDUCT.md, PR/Issue templates
- Semantic Versioning, CHANGELOG.md, detailed code comments & auto-generated API docs
- English-native documentation & code (aligned with international open-source practices)

## Tech Stack & Compatibility Matrix
### Supported Versions (Auto-Detect / Env Specify)
| Component   | Supported Versions       | OS Requirement          | Embedded Support               | Python Version |
|-------------|--------------------------|-------------------------|--------------------------------|----------------|
| ROS1        | Noetic Ninjemys (LTS)    | Ubuntu 20.04 / WSL2     | Jetson JetPack4.6+, RPi 4      | 3.8+           |
| ROS2        | Humble Hawksbill (LTS)   | Ubuntu 22.04 / WSL2     | Jetson JetPack5.1+, RPi 5      | 3.8+           |
| ROS2        | Jazzy Jalisco            | Ubuntu 24.04 / WSL2     | Jetson JetPack6.0+             | 3.10+          |
| OpenClaw    | v1.x / v2.x              | All above OS            | All supported embedded devices | 3.8+           |

### Core Dependencies
Install all dependencies via `pip3 install -r requirements.txt` (auto-resolved for ROS1/ROS2).
- **Core**: `pyyaml>=6.0.1`, `psutil>=5.9.5`, `python-dotenv>=1.0.0`, `pyserial>=3.5`, `requests>=2.31.0`
- **ROS1**: `rospy>=1.15.0`, `catkin-tools>=0.9.0`, `ros1_bridge>=0.11.0`, `ros-noetic-rosbridge-suite`
- **ROS2**: `rclpy>=3.8.0`, `colcon-common-extensions>=0.3.0`, `ros-humble-rosbridge-suite`, `launch_ros`
- **Embedded**: `jetson-stats>=4.0.0` (Jetson), `rpi.gpio>=0.7.1` (RPi), `nvml>=11.5` (NVIDIA)
- **Testing/Docs**: `pytest>=7.4.0`, `pytest-cov>=4.1.0`, `pytest-benchmark>=4.0.0`, `sphinx>=7.2.6`

## Quick Start (3 Steps)
### Prerequisites
- Ubuntu 20.04/22.04/24.04 or Windows WSL2 (matching OS version)
- Sudo privileges (for dependency installation/deployment)
- Internet access (for first build; fully offline after initial setup)

### Step 1: Clone the Repository
```bash
git clone https://github.com/webthree549-bot/openclaw-ros-bridge.git
cd openclaw-ros-bridge
```

### Step 2: One-Click Build
Auto-detect ROS version, install dependencies, and compile the project:
```bash
# Grant execute permission to all scripts
chmod +x scripts/*.sh
# One-click build
./scripts/build.sh
```

### Step 3: One-Click Run Demo
#### Option 1: Production-Grade Greenhouse Demo (Sensor Upstream + Actuator Downstream)
```bash
./scripts/run_demo.sh --greenhouse
```

#### Option 2: Embodied Intelligence Arm Manipulation Demo
```bash
./scripts/run_demo.sh --arm
```
> Press `Ctrl+C` for graceful shutdown (all nodes/hardware/connections closed properly).

## Docker One-Click Deployment (Recommended)
Isolated ROS1/ROS2 environment, no local ROS installation required:
### Step 1: Build Docker Images
```bash
./scripts/docker_build.sh
```

### Step 2: Start All Containers
```bash
docker-compose -f docker/docker-compose.yml up -d
```

### Step 3: View Logs
```bash
docker-compose -f docker/docker-compose.yml logs -f openclaw-ros-bridge
```

### Stop Containers
```bash
docker-compose -f docker/docker-compose.yml down
```

## One-Click Testing
Run the **complete test suite** (unit + integration + performance) and generate an HTML coverage report:
```bash
./scripts/run_tests.sh
```
- Test results: Real-time terminal output
- Coverage report: `test_coverage/index.html` (open in browser for detailed coverage)
- Auto-adapts to the detected ROS version (no manual configuration)

## Cross-Platform / Embedded Deployment
### WSL2 One-Click Setup
Fix ROS GUI (rqt/RViz) and USB hardware passthrough for Windows WSL2:
```bash
sudo ./scripts/deploy_wsl2.sh
```

### Jetson One-Click Deployment
Auto-detect JetPack version (4.6+→ROS1, 5.1+→ROS2 Humble, 6.0+→ROS2 Jazzy) + optional boot auto-start:
```bash
sudo ./scripts/deploy_jetson.sh
```

### Raspberry Pi One-Click Deployment
Adapted for RPi4/5 + Ubuntu 20.04/22.04 (hardware driver + framework deployment):
```bash
sudo ./scripts/deploy_rpi.sh
```

## Observability & Debugging
### Real-Time Performance Dashboard
Built-in dashboard (auto-start with demos) monitors:
- ROS/OpenClaw communication latency & throughput
- Node CPU/memory usage
- Sensor/actuator hardware status
- Fault recovery count & current state

### Graded Logging
- 5 log levels: `DEBUG` / `INFO` / `WARN` / `ERROR` / `FATAL`
- Default log directory: `logs/` (log rotation enabled)
- Remote log streaming (ROS/HTTP) configurable in `config/debug_config.yaml`

### Mock Mode (No Hardware/OpenClaw)
Develop/test business logic **without any hardware/OpenClaw connection**:
```bash
export MOCK_MODE=true
./scripts/run_demo.sh --greenhouse
```

## Documentation
- **Quick Start**: [docs/quickstart.md](docs/quickstart.md)
- **Troubleshooting**: [docs/troubleshooting.md](docs/troubleshooting.md)
- **WSL2 Setup**: [docs/wsl2_setup.md](docs/wsl2_setup.md)
- **Embedded Deployment**: [docs/embedded_deploy.md](docs/embedded_deploy.md)
- **API Docs**: [docs/api/index.html](docs/api/index.html) (generate via `./scripts/gen_api_docs.sh`)

## Open-Source Community
### License
This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details (commercial-friendly, free to use/modify/distribute).

### Contribution Guidelines
See [CONTRIBUTING.md](CONTRIBUTING.md) for PR/Issue submission rules, code style, and testing requirements.

### Code of Conduct
This project adheres to the [Contributor Covenant Code of Conduct](CODE_OF_CONDUCT.md) (v2.1). By participating, you agree to uphold this code.

### Versioning
We follow **Semantic Versioning (SemVer)** - see [CHANGELOG.md](CHANGELOG.md) for version history. Future updates follow `MAJOR.MINOR.PATCH` rules:
- `MAJOR`: Breaking changes
- `MINOR`: New features (backward-compatible)
- `PATCH`: Bug fixes (backward-compatible)

### Issue Reporting
Submit issues via [GitHub Issues](https://github.com/webthree549-bot/openclaw-ros-bridge/issues) using the provided templates. Include **environment details, reproduction steps, and logs** for fast debugging.

## Maintainers
- **Author**: OpenClaw-ROS Dev Team
- **Email**: dev@openclaw-ros.org
- **GitHub**: [webthree549-bot/openclaw-ros-bridge](https://github.com/webthree549-bot/openclaw-ros-bridge)

## Acknowledgments
- ROS Official Team for ROS1/ROS2 ecosystem
- NVIDIA Jetson/Raspberry Pi communities for embedded support
- OpenClaw AI for the embodied intelligence agent framework