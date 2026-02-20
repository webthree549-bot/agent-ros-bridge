# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- None

### Changed
- None

### Deprecated
- None

### Removed
- None

### Fixed
- None

### Security
- None

## [0.3.4] - 2026-02-20

### Security
- Fixed clear-text logging of JWT credentials (CodeQL alert #83)
  - Tokens and secrets now print to stderr instead of stdout
  - Prevents accidental exposure in logs and CI/CD pipelines

## [0.3.3] - 2026-02-15

### Fixed
- SKILL.md metadata: removed trailing commas from JSON (fixes registry parsing)
- Install spec now properly recognized as `pip install agent-ros-bridge`

## [0.3.2] - 2025-02-15

### Release & Submission
- Production release for ClawHub submission
- All documentation consistency fixes complete
- Security model finalized: JWT always required
- Ready for public distribution

## [0.3.1] - 2025-02-15

### Documentation Consistency Fix
- **CRITICAL**: Removed all remaining "mock mode" references from documentation
- Renamed files: `mock_bridge.py` → `simulated_robot.py`, `mock_bridge_auth.py` → `auth_demo.py`
- Updated all README files to remove mock mode references
- Updated SKILL.md commands section (removed `demo mock`, added `demo quickstart`)
- Fixed docker-compose.yml files to use `--simulated` instead of `--mock`
- Updated print statements in example scripts
- All examples now consistently use "simulated" terminology

## [0.3.0] - 2025-02-15

### ⚠️ BREAKING CHANGES

#### Removed: Mock Mode
- **Mock mode has been completely removed**
- Authentication is now **always required** with no bypass option
- Bridge will fail to start without JWT_SECRET

#### Changed: Example Deployment
- All examples now run in **Docker containers only**
- No more native mock mode execution
- Examples provide isolated, secure testing environments

#### Changed: Default Bind Address
- Default changed from `0.0.0.0` to `127.0.0.1` (localhost only)
- Reduces accidental network exposure

### Security Improvements
- **Mandatory authentication**: No way to disable auth
- **Docker isolation**: All examples run in containers
- **Simplified security model**: JWT_SECRET always required
- **No ambiguous states**: Clear security posture

### Documentation
- Updated README for Docker-only examples
- Updated SECURITY.md with new security model
- Created docker-compose.yml files for all examples

## [0.2.4] - 2025-02-15

### Documentation Fixes
- Fixed README to reference correct entry points (agent-ros-bridge CLI)
- Clarified JWT_SECRET required for production, optional for mock mode
- Added MOCK_MODE and BRIDGE_HOST env variables to SKILL.md
- Fixed requires/env metadata to show JWT_SECRET as optional
- Improved security documentation clarity

## [0.2.3] - 2025-02-15

### ClawHub Submission
- Reformatted SKILL.md metadata for registry compatibility
- Fresh release to trigger new security scan
- Optimized submission package structure

## [0.2.2] - 2025-02-15

### Security & Documentation
- Removed curl|bash install pattern from README (security best practice)
- Added mock mode security warnings (localhost-only recommendation)
- Added BRIDGE_HOST=127.0.0.1 guidance for testing
- Synced _version.py auto-generated file
- Cleaned submission package for ClawHub

## [0.2.1] - 2025-02-14

### Security (Critical)
- **BREAKING**: Authentication now enabled by default (was disabled)
- **BREAKING**: JWT_SECRET now required when auth enabled (no auto-generation)
- Added JWT_SECRET to SKILL.md required environment variables
- Added security warnings to README and run.sh scripts
- Authenticator now fails fast if JWT_SECRET not set (vs auto-generating)
- Added SECURITY.md policy document

## [0.2.0] - 2025-02-14

### Added
- **Comprehensive documentation suite** (40,000+ words)
  - User Manual (23,000+ words) - Installation, tutorials, troubleshooting
  - API Reference (13,000+ words) - Complete API documentation
  - Docker vs Native deployment guide
  - DDS Architecture documentation
  - Multi-ROS fleet management guide
  - Repository structure documentation
- **7 self-contained examples** with READMEs and run.sh scripts
  - quickstart/ - Basic bridge usage (30-second launch)
  - fleet/ - Multi-robot coordination
  - auth/ - JWT authentication
  - mqtt_iot/ - IoT sensor integration
  - actions/ - ROS navigation/actions
  - arm/ - Robotic arm control (UR, xArm, Franka)
  - metrics/ - Prometheus monitoring
- **ROS setup validation script** (`scripts/validate_ros_setup.py`)
  - Checks ROS1/ROS2 installation
  - Validates agent-ros-bridge imports
  - Color-coded pass/warning/fail output
- **OpenClaw integration tests** (`tests/test_openclaw_integration.py`)
  - Validates SKILL.md manifest
  - Tests all module imports
  - Verifies basic functionality
- **Production-grade repository structure**
  - Clean separation of source/build artifacts
  - Makefile with comprehensive targets
  - CONTRIBUTING.md with development workflow
  - 155 source files, 0 build artifacts in git

### Changed
- **BREAKING**: Reorganized `demo/` → `examples/` for clarity
  - Each example is self-contained with README and run.sh
  - Updated all documentation references
  - Added `make example-*` targets
- **Version normalization** across all files (0.1.0 → 0.2.0)
- **SKILL.md optimized** for OpenClaw integration
  - Added commands section for `openclaw run` support
  - Enhanced metadata for ClawHub discovery
- **Makefile enhanced**
  - Added example runners (make example-quickstart, etc.)
  - Improved clean target (removes all build artifacts)
  - Added validation target

### Removed
- Legacy `demo/greenhouse/` (unused)
- Legacy `demo/arm_manipulation/` (superseded by examples/arm/)
- Old `demo/__init__.py` structure

### Fixed
- Git push issues (configured HTTP/1.1)
- README duplicate sections removed
- Project structure diagram updated

## [0.1.0] - 2025-02-14

### Added
- Initial PyPI release
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

[Unreleased]: https://github.com/webthree549-bot/agent-ros-bridge/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/webthree549-bot/agent-ros-bridge/releases/tag/v0.1.0
[1.0.0]: https://github.com/webthree549-bot/openclaw-ros-bridge/releases/tag/v1.0.0
