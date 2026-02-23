# Contributing to Agent ROS Bridge

Thank you for your interest in contributing! This document provides guidelines for contributing to the project.

## Getting Started

1. **Fork the repository** on GitHub
2. **Clone your fork** locally:
   ```bash
   git clone https://github.com/YOUR_USERNAME/agent-ros-bridge.git
   cd agent-ros-bridge
   ```
3. **Set up development environment**:
   ```bash
   pip install -e ".[all,dev]"
   ```
4. **Create a branch** for your feature:
   ```bash
   git checkout -b feature/your-feature-name
   ```

## Development Workflow

### Before You Start

- Check [TODO.md](TODO.md) for open tasks
- Comment on an issue to claim it: "Working on #123"
- Create an issue if one doesn't exist for your feature/bug

### Code Standards

We use:
- **black** for code formatting (line length: 100)
- **ruff** for linting
- **mypy** for type checking
- **pytest** for testing

Run checks before committing:
```bash
# Format code
black agent_ros_bridge/ tests/

# Lint
ruff agent_ros_bridge/ tests/

# Type check
mypy agent_ros_bridge/

# Run tests
pytest
```

### Writing Tests

All new features must include tests:

```python
# tests/unit/test_action_registry.py
import pytest
from agent_ros_bridge import ActionRegistry

@pytest.fixture
def registry():
    return ActionRegistry()

def test_register_action(registry):
    registry.register("test", lambda: None)
    assert "test" in registry.list_actions()

def test_get_action(registry):
    handler = lambda: "result"
    registry.register("test", handler)
    assert registry.get("test") == handler
```

Test organization:
- `tests/unit/` — Unit tests for individual components
- `tests/integration/` — Integration tests for component interactions
- `tests/e2e/` — End-to-end tests for full workflows

### Commit Messages

Follow [Conventional Commits](https://www.conventionalcommits.org/):

```
feat: add action confirmation system
fix: handle WebSocket disconnect gracefully
docs: update README with distributed deployment
test: add unit tests for ROS2 connector
refactor: simplify transport manager
```

Types:
- `feat` — New feature
- `fix` — Bug fix
- `docs` — Documentation only
- `test` — Adding tests
- `refactor` — Code refactoring
- `perf` — Performance improvements
- `chore` — Maintenance tasks

### Pull Request Process

1. **Update documentation** if needed (README, docstrings, etc.)
2. **Add to CHANGELOG.md** under [Unreleased]
3. **Ensure CI passes** (tests, linting, type checking)
4. **Fill out PR template**:
   - What changed?
   - Why?
   - How to test?
   - Related issues
5. **Request review** from maintainers
6. **Address feedback** promptly

## Code Review

PRs will be reviewed for:
- **Correctness** — Does it work? Are edge cases handled?
- **Testing** — Are there adequate tests?
- **Documentation** — Is it documented?
- **Style** — Does it follow project conventions?
- **Performance** — Any obvious bottlenecks?

## Areas Needing Help

Priority areas for contributions:

### 🔴 High Priority
- **Testing** — Write unit/integration tests
- **Documentation** — Tutorials, examples, API docs
- **ROS2 Testing** — Validate with real robots
- **CI/CD** — GitHub Actions improvements

### 🟡 Medium Priority
- **Connectors** — New robot platforms (TurtleBot, UR, etc.)
- **Integrations** — LangChain, AutoGPT, etc.
- **Observability** — Metrics, tracing, dashboards

### 🟢 Low Priority
- **Benchmarks** — Performance testing
- **Simulations** — Gazebo, Isaac Sim connectors

## Development Setup

### With ROS2 (recommended for full testing)

```bash
# Install ROS2 Jazzy
# https://docs.ros.org/en/jazzy/Installation.html

source /opt/ros/jazzy/setup.bash

# Install package
pip install -e ".[all,dev]"

# Run tests
pytest
```

### Without ROS (mock mode)

```bash
# Install without ROS dependencies
pip install -e ".[websocket,grpc,mcp,dev]"

# Run tests (some will be skipped)
pytest -m "not requires_ros"
```

### Docker Development

```bash
# Build development image
docker build -f Dockerfile.dev -t agent-ros-bridge:dev .

# Run with local code mounted
docker run -it --rm \
  -v $(pwd):/app \
  -p 8765:8765 \
  agent-ros-bridge:dev
```

## Reporting Issues

When reporting bugs, include:

1. **Description** — What happened?
2. **Reproduction steps** — How to reproduce?
3. **Expected behavior** — What should happen?
4. **Environment**:
   - OS
   - Python version
   - ROS version
   - Package version
5. **Logs** — Relevant error messages

Example:
```markdown
**Description**
WebSocket transport fails to start with TLS enabled.

**Steps to Reproduce**
1. Configure WebSocket with TLS cert/key
2. Start bridge
3. See error

**Expected Behavior**
Server should start with wss://

**Environment**
- OS: Ubuntu 22.04
- Python: 3.11
- ROS2: Jazzy
- Version: 0.1.0

**Logs**
```
[ERROR] Failed to load TLS certificate: [Errno 2] No such file
```
```

## Security

If you discover a security vulnerability:

1. **DO NOT** open a public issue
2. Email security@openclaw.ai with details
3. Include reproduction steps and impact assessment
4. Allow time for fix before disclosure

## Community

- **Discord**: [Join us](https://discord.gg/agent-ros-bridge)
- **Discussions**: [GitHub Discussions](https://github.com/webthree549-bot/agent-ros-bridge/discussions)
- **Office Hours**: Fridays at 10am PT (Discord)

## Code of Conduct

This project follows the [Contributor Covenant](https://www.contributor-covenant.org/) Code of Conduct:

- Be respectful and inclusive
- Welcome newcomers
- Focus on constructive feedback
- Respect differing viewpoints

Report unacceptable behavior to conduct@openclaw.ai.

## License

By contributing, you agree that your contributions will be licensed under the [MIT License](LICENSE).

## Questions?

- Check [FAQ.md](docs/FAQ.md) (coming soon)
- Ask in [Discussions](https://github.com/webthree549-bot/agent-ros-bridge/discussions)
- Join [Discord](https://discord.gg/agent-ros-bridge)

Thank you for contributing! 🤖
