# Contributing to Agent ROS Bridge

Thank you for your interest in contributing! This guide explains how to set up your development environment and keep the repository clean.

## Repository Structure

We maintain a strict separation between **source code** and **build artifacts**:

```
agent-ros-bridge/                 # ← Repository root
├── agent_ros_bridge/            # ← Source code only
│   ├── __init__.py
│   ├── gateway_v2/
│   ├── plugins/
│   └── ...
├── tests/                       # ← Test source code
├── demo/                        # ← Demo scripts
├── docs/                        # ← Documentation source
├── scripts/                     # ← Utility scripts
├── config/                      # ← Configuration examples
├── docker/                      # ← Docker files
├── dashboards/                  # ← Grafana dashboards
├── Makefile                     # ← Build automation
├── pyproject.toml              # ← Package config
└── .gitignore                   # ← Ignore patterns

# Build artifacts (NOT in git):
build/                          # ← Build output (gitignored)
dist/                           # ← Distribution files (gitignored)
*.egg-info/                     # ← Package metadata (gitignored)
__pycache__/                    # ← Python cache (gitignored)
.pytest_cache/                  # ← Test cache (gitignored)
.coverage                       # ← Coverage data (gitignored)
```

## Quick Start

### 1. Clone and Setup

```bash
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
cd agent-ros-bridge
make install-dev
```

### 2. Verify Clean State

```bash
make clean        # Remove any build artifacts
make check        # Run linting and tests
```

### 3. Run Tests

```bash
make test         # Run all tests
make test-unit    # Unit tests only
make test-openclaw # OpenClaw integration tests
```

## Development Workflow

### Before Making Changes

```bash
# Start from clean state
git checkout main
git pull origin main
make clean
```

### Making Changes

1. **Create a branch:**
   ```bash
   git checkout -b feature/your-feature-name
   ```

2. **Make your changes** (edit source files only)

3. **Format and lint:**
   ```bash
   make format       # Auto-format code
   make lint         # Check for issues
   ```

4. **Test:**
   ```bash
   make test         # Run tests
   ```

5. **Build (optional):**
   ```bash
   make build        # Create distribution packages
   ```

### Before Committing

```bash
# Ensure clean state
make clean

# Check what files will be committed
git status

# Only source files should appear (no __pycache__, no dist/)
```

## Clean Build Policy

**Never commit build artifacts.** The following are automatically ignored:

| Pattern | What It Ignores |
|---------|-----------------|
| `__pycache__/` | Python bytecode cache |
| `*.pyc`, `*.pyo` | Compiled Python files |
| `build/` | Build output directory |
| `dist/` | Distribution packages |
| `*.egg-info/` | Package metadata |
| `.pytest_cache/` | Test cache |
| `.coverage` | Coverage data |
| `htmlcov/` | HTML coverage reports |

### Checking Cleanliness

```bash
# List all ignored files that exist
 git check-ignore $(git ls-files -o --exclude-standard)

# Ensure no build artifacts are staged
git diff --cached --name-only | grep -E "(pyc|__pycache__|dist/|build/)" && echo "ERROR: Build artifacts detected!"
```

## Build Commands

### Development Build
```bash
make install-dev    # Editable install with dev dependencies
```

### Production Build
```bash
make clean          # Clean first
make build          # Create wheel and sdist
ls dist/            # View built packages
```

### Installation from Build
```bash
pip install dist/agent_ros_bridge-*.whl
```

## Testing

### Test Structure

```
tests/
├── unit/                    # Unit tests
│   └── test_core.py
├── integration/             # Integration tests
│   └── test_integration.py
└── test_openclaw_integration.py  # OpenClaw tests
```

### Running Tests

```bash
# All tests
make test

# Specific test categories
make test-unit           # Unit tests only
make test-int            # Integration tests
make test-openclaw       # OpenClaw integration

# With coverage
pytest --cov=agent_ros_bridge tests/
```

### Writing Tests

```python
# tests/unit/test_example.py
def test_example():
    from agent_ros_bridge import Bridge
    bridge = Bridge()
    assert bridge is not None
```

## Code Style

We use:
- **black** for formatting
- **ruff** for linting
- **isort** for import sorting

### Auto-format

```bash
make format
```

### Check Style

```bash
make lint
```

## Documentation

Documentation is in `docs/` as Markdown files:

- `USER_MANUAL.md` - User guide
- `API_REFERENCE.md` - API documentation
- `NATIVE_ROS.md` - Native ROS setup
- `MULTI_ROS.md` - Fleet management
- `DOCKER_VS_NATIVE.md` - Deployment comparison
- `DDS_ARCHITECTURE.md` - DDS explanation

### View Documentation Locally

```bash
make serve-docs
# Open http://localhost:8000
```

## Docker Development

For consistent environments:

```bash
# Build images
make docker-build

# Run ROS2 bridge
make docker-up

# Stop
make docker-down
```

## Common Issues

### "Build artifacts detected"

If `git status` shows `__pycache__` or `dist/`:

```bash
make clean
git status  # Should be clean now
```

### "Import errors during tests"

```bash
make install-dev  # Reinstall in development mode
```

### "Tests pass locally but fail in CI"

```bash
make clean
make check
```

## Release Process

1. **Update version:**
   ```bash
   # Version is managed by hatch-vcs based on git tags
   git tag v0.2.0
   ```

2. **Build and test:**
   ```bash
   make clean
   make build
   make check
   ```

3. **Upload to PyPI:**
   ```bash
   make upload-test  # TestPyPI first
   make upload       # Production PyPI
   ```

## Questions?

- **Issues:** https://github.com/webthree549-bot/agent-ros-bridge/issues
- **Discussions:** https://github.com/webthree549-bot/agent-ros-bridge/discussions

## License

By contributing, you agree that your contributions will be licensed under the MIT License.
