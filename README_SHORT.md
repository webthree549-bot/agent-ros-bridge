# OpenClaw ROS Bridge v2.0.0

<p align="center">
  <img src="https://img.shields.io/badge/ROS-1%2F2-orange.svg" alt="ROS">
  <img src="https://img.shields.io/badge/Python-3.8%2B-blue.svg" alt="Python">
  <img src="https://img.shields.io/badge/License-MIT-green.svg" alt="License">
  <a href="https://clawhub.com/skills/openclaw-ros-bridge">
    <img src="https://img.shields.io/badge/ClawHub-Available-blueviolet" alt="ClawHub">
  </a>
</p>

<p align="center">
  <a href="https://github.com/webthree549-bot/openclaw-ros-bridge/actions/workflows/ci.yml">
    <img src="https://github.com/webthree549-bot/openclaw-ros-bridge/actions/workflows/ci.yml/badge.svg" alt="CI">
  </a>
  <a href="https://github.com/webthree549-bot/openclaw-ros-bridge/actions/workflows/release.yml">
    <img src="https://github.com/webthree549-bot/openclaw-ros-bridge/actions/workflows/release.yml/badge.svg" alt="Release">
  </a>
  <a href="https://codecov.io/gh/webthree549-bot/openclaw-ros-bridge">
    <img src="https://codecov.io/gh/webthree549-bot/openclaw-ros-bridge/branch/main/graph/badge.svg" alt="Coverage">
  </a>
  <a href="https://pypi.org/project/openclaw-ros-bridge/">
    <img src="https://img.shields.io/pypi/v/openclaw-ros-bridge.svg" alt="PyPI">
  </a>
  <a href="https://hub.docker.com/r/openclaw/ros-bridge">
    <img src="https://img.shields.io/docker/pulls/openclaw/ros-bridge.svg" alt="Docker Pulls">
  </a>
</p>

<p align="center">
  <a href="https://openclaw-ros-bridge.readthedocs.io">
    <img src="https://img.shields.io/badge/docs-mkdocs-blue.svg" alt="Documentation">
  </a>
  <a href="https://securityscorecards.dev/viewer/?uri=github.com/webthree549-bot/openclaw-ros-bridge">
    <img src="https://api.securityscorecards.dev/projects/github.com/webthree549-bot/openclaw-ros-bridge/badge" alt="OpenSSF Scorecard">
  </a>
</p>

---

**Universal Robot Gateway** - Multi-protocol, multi-robot, cloud-native connectivity platform for AI agents to control ROS-based robots and embodied intelligence systems.

## 🚀 Quick Start

### Option 1: ClawHub (Recommended for OpenClaw Users)

```bash
# Install via ClawHub
openclaw skills add openclaw-ros-bridge

# Start demo
openclaw-skill-ros-bridge demo
```

### Option 2: PyPI

```bash
pip install openclaw-ros-bridge

# Start gateway
openclaw-gateway --demo
```

### Option 3: Docker

```bash
docker run -p 8765:8765 ghcr.io/webthree549-bot/openclaw-ros-bridge:latest
```

## ✨ Features

- **Multi-Protocol** - WebSocket, gRPC, MQTT, TCP, HTTP
- **Multi-Robot** - ROS1, ROS2, industrial arms, drones, mobile robots
- **Fleet Management** - Control multiple robots as a unified fleet
- **Plugin System** - Build custom robot applications
- **Cloud-Native** - Kubernetes, Docker, auto-scaling
- **Production-Ready** - CI/CD, security scanning, observability

## 📖 Documentation

- [Full Documentation](https://openclaw-ros-bridge.readthedocs.io)
- [ClawHub Skill Page](https://clawhub.com/skills/openclaw-ros-bridge)
- [API Reference](https://openclaw-ros-bridge.readthedocs.io/api)
- [Architecture](docs/ARCHITECTURE_V2.md)

## 🛠️ Development

```bash
git clone https://github.com/webthree549-bot/openclaw-ros-bridge.git
cd openclaw-ros-bridge
make install-dev
make test
```

See [CONTRIBUTING.md](CONTRIBUTING.md) for details.

## 📄 License

[MIT License](LICENSE)

## 🤝 Contributing

Contributions welcome! See [CONTRIBUTING.md](CONTRIBUTING.md).

---

<p align="center">
  Made with ❤️ by the OpenClaw ROS Team<br>
  <a href="https://clawhub.com">Available on ClawHub</a>
</p>
