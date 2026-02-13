# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- New Gateway v2 architecture with multi-protocol support (WebSocket, gRPC, MQTT, TCP)
- Plugin system with hot-reload capability
- Multi-robot fleet management
- ROS2 connector with auto-discovery
- Greenhouse demo plugin
- Comprehensive CI/CD pipeline with GitHub Actions
- Docker multi-architecture builds (amd64, arm64)
- Security scanning with Trivy and Bandit
- Pre-commit hooks for code quality
- Full test suite with pytest
- Documentation site with MkDocs

### Changed
- Refactored from single-purpose TCP bridge to universal robot gateway
- Improved configuration system with YAML/JSON/Env support
- Enhanced error handling and logging

### Deprecated
- Legacy v1 TCP server (replaced by Gateway v2)

### Removed
- None

### Fixed
- None

### Security
- Added security scanning to CI pipeline
- Implemented authentication framework

## [1.0.0] - 2024-01-15

### Added
- Initial release of OpenClaw ROS Bridge
- ROS1 Noetic and ROS2 Humble/Jazzy support
- Hardware Abstraction Layer (HAL)
- Fault self-recovery mechanisms
- Real-time scheduling support
- Docker containerization
- Mock mode for testing without hardware
- TCP server for OpenClaw integration
- Greenhouse demo application
- Complete test suite (unit + integration)
- Documentation and deployment scripts

[Unreleased]: https://github.com/webthree549-bot/openclaw-ros-bridge/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/webthree549-bot/openclaw-ros-bridge/releases/tag/v1.0.0
