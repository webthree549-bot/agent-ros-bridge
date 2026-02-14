# Repository Structure

Agent ROS Bridge follows production-grade repository organization with strict separation between **source code** and **build artifacts**.

## Directory Layout

```
agent-ros-bridge/                          # Repository root
│
├── 📁 agent_ros_bridge/                   # ⭐ Source code (Python package)
│   ├── __init__.py                        # Package initialization
│   ├── gateway_v2/                        # Core gateway implementation
│   │   ├── core.py                        # Bridge class
│   │   ├── auth.py                        # Authentication
│   │   ├── connectors/                    # ROS connectors
│   │   │   ├── ros1_connector.py          # ROS1 support
│   │   │   └── ros2_connector.py          # ROS2 support
│   │   └── transports/                    # Communication protocols
│   │       ├── websocket.py               # WebSocket transport
│   │       └── mqtt_transport.py          # MQTT transport
│   ├── fleet/                             # Fleet orchestration
│   │   └── orchestrator.py                # Multi-robot management
│   ├── plugins/                           # Robot plugins
│   │   └── arm_robot.py                   # Arm robot control
│   ├── actions/                           # ROS actions
│   └── metrics/                           # Prometheus metrics
│
├── 📁 tests/                              # ⭐ Test source code
│   ├── unit/                              # Unit tests
│   ├── integration/                       # Integration tests
│   └── test_openclaw_integration.py       # OpenClaw validation
│
├── 📁 demo/                               # ⭐ Demo scripts
│   ├── mock_bridge.py                     # Mock robot demo
│   ├── fleet_demo.py                      # Fleet demo
│   ├── arm_demo.py                        # Arm robot demo
│   └── actions_demo.py                    # ROS actions demo
│
├── 📁 docs/                               # ⭐ Documentation source
│   ├── USER_MANUAL.md                     # Complete user guide
│   ├── API_REFERENCE.md                   # API documentation
│   ├── NATIVE_ROS.md                      # Native ROS setup
│   ├── MULTI_ROS.md                       # Fleet management
│   ├── DOCKER_VS_NATIVE.md                # Deployment guide
│   └── DDS_ARCHITECTURE.md                # DDS explanation
│
├── 📁 scripts/                            # ⭐ Utility scripts
│   ├── validate_ros_setup.py              # ROS validation
│   └── generate_token.py                  # JWT token generator
│
├── 📁 config/                             # ⭐ Configuration templates
│   ├── bridge.yaml                        # Bridge configuration
│   └── bridge-auth.yaml                   # Auth configuration
│
├── 📁 docker/                             # ⭐ Docker files
│   ├── Dockerfile.ros1                    # ROS1 container
│   └── Dockerfile.ros2                    # ROS2 container
│
├── 📁 dashboards/                         # ⭐ Monitoring dashboards
│   └── grafana-dashboard.json             # Grafana dashboard
│
├── 📁 dashboard/                          # ⭐ Web dashboard
│   ├── server.py                          # Dashboard server
│   └── static/                            # Static assets
│
├── 📄 Makefile                            # ⭐ Build automation
├── 📄 pyproject.toml                      # ⭐ Package configuration
├── 📄 .gitignore                          # ⭐ Ignore patterns
├── 📄 README.md                           # ⭐ Project readme
├── 📄 CONTRIBUTING.md                     # ⭐ Contribution guide
├── 📄 LICENSE                             # ⭐ MIT License
├── 📄 CHANGELOG.md                        # ⭐ Version history
├── 📄 SKILL.md                            # ⭐ OpenClaw skill manifest
│
├── 📄 run_bridge.py                       # ⭐ Production entry point
├── 📄 run_bridge_dual_ros.py              # ⭐ Dual ROS entry point
└── 📄 docker-compose.yml                  # ⭐ Docker orchestration
```

## Build Artifacts (Ignored)

The following are **NOT** in git and are generated during build:

```
# Build output (gitignored)
build/                          # Python build directory
dist/                           # Distribution packages (*.whl, *.tar.gz)
*.egg-info/                     # Package metadata
.eggs/                          # Egg cache

# Python cache (gitignored)
__pycache__/                    # Bytecode cache
*.pyc                           # Compiled Python
*.pyo                           # Optimized Python
*.so                            # C extensions

# Test artifacts (gitignored)
.pytest_cache/                  # Test cache
.coverage                       # Coverage data
htmlcov/                        # HTML coverage reports
.tox/                           # Tox environments

# Environment (gitignored)
.env                            # Local environment
.venv/                          # Virtual environment
venv/                           # Virtual environment

# IDE (gitignored)
.vscode/                        # VSCode settings
.idea/                          # IntelliJ settings
*.swp                           # Vim swap files

# OS (gitignored)
.DS_Store                       # macOS metadata
Thumbs.db                       # Windows thumbnails

# Project specific (gitignored)
logs/                           # Log files
*.log                           # Log files
prometheus-data/                # Prometheus data
grafana-data/                   # Grafana data
mqtt-data/                      # MQTT persistence
```

## Clean Development Workflow

### Start Clean

```bash
# Clone repository
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
cd agent-ros-bridge

# Verify clean state
git status                    # Should show: "nothing to commit, working tree clean"
make clean                    # Remove any stray artifacts
```

### Development

```bash
# Install in development mode
make install-dev

# Make changes to source files only
# Edit files in: agent_ros_bridge/, tests/, demo/, docs/

# Format and test
make format                   # Auto-format code
make test                     # Run tests
```

### Before Commit

```bash
# Ensure clean state
make clean
git status                    # Only source files should appear

# Verify no build artifacts
make check                    # Run lint + test
```

### Build Distribution

```bash
# Create clean build
make clean
make build

# Verify output
ls dist/                      # Should contain: *.whl, *.tar.gz
```

## Quick Commands

| Command | Purpose |
|---------|---------|
| `make clean` | Remove all build artifacts |
| `make build` | Create wheel and sdist |
| `make test` | Run all tests |
| `make lint` | Check code style |
| `make format` | Auto-format code |
| `make check` | Run lint + test |
| `make validate` | Validate ROS setup |

## File Count

- **Source files in git**: ~154 files
- **Build artifacts**: 0 (all ignored)
- **Total repository size**: ~8,000 lines of code

## Principles

1. **Source Only**: Only hand-written source code is committed
2. **Generated Code**: All build artifacts are generated, not stored
3. **Reproducible**: Clean checkout + `make build` produces identical output
4. **Platform Agnostic**: No OS-specific files in git
5. **IDE Neutral**: No IDE-specific files in git

## Verification

Check that your repository is clean:

```bash
# Should return 0 untracked files
git ls-files -o --exclude-standard | wc -l

# Should return 0
find . -name "__pycache__" -type d | wc -l

# Should return 0
find . -name "*.pyc" -type f | wc -l
```

If any of these return non-zero, run:

```bash
make clean
```
