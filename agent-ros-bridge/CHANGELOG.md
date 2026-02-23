# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Test suite with pytest
- CI/CD pipeline with GitHub Actions
- Docker multi-arch images (amd64, arm64)
- API documentation site

## [0.4.0] - 2026-02-23

### Added
- Core ROSBridge class with action registry, session management
- ROS2 connector (Jazzy, Humble, Iron, Rolling support)
- ROS1 connector (Noetic support)
- WebSocket transport with TLS, JWT auth, CORS
- gRPC transport with mutual TLS
- gRPC client for remote bridge connections
- MCP server for Claude Desktop integration
- Configuration system (YAML, env vars)
- Example dashboard with HTML/JS
- Comprehensive README with distributed architecture
- OpenClaw integration (privileged cloud features)
- macOS support (Homebrew, Docker, RoboStack)
- ROS Community release support (bloom, package.xml)

### Phase 2: Agentic AI Features
- **Agent Memory System** — SQLite/Redis backends with TTL support
- **Tool Discovery** — Auto-discover ROS tools, export to MCP/OpenAI format
- **Action Confirmation** — Safety levels with emergency stop and audit logging

### Phase 3: Observability
- **Prometheus Metrics** — Action counts, latency, connections, topics
- **OpenTelemetry Tracing** — Distributed tracing with Jaeger/OTLP export
- **Real-time Dashboard** — Web UI with live telemetry and emergency stop
- **Health Checks** — Liveness/readiness probes

### Phase 4: Ecosystem Integration
- **LangChain Integration** — ROSBridgeTool and ROSAgent for LangChain agents
- **AutoGPT Plugin** — Native AutoGPT plugin support
- **ROS2 Actions** — Navigation2 and MoveIt action clients
- **Multi-Agent Orchestration** — Fleet management capabilities

### Security
- JWT authentication for agent verification
- TLS encryption for WebSocket and gRPC
- Mutual TLS support for gRPC
- MIT License — Simple, permissive open source licensing

### Changed
- License changed from Apache-2.0 to MIT
- Author: webthree549 <webthree549@gmail.com>

## [0.1.0] - 2026-02-22

### Added
- Initial project structure
- Basic transport layer
- ROS2 mock connector (for development)

[Unreleased]: https://github.com/webthree549-bot/agent-ros-bridge/compare/v0.4.0...HEAD
[0.4.0]: https://github.com/webthree549-bot/agent-ros-bridge/releases/tag/v0.4.0
[0.1.0]: https://github.com/webthree549-bot/agent-ros-bridge/releases/tag/v0.1.0
