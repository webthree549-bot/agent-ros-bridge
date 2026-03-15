# Contributing to Agent ROS Bridge

Thank you for your interest in contributing! 🎉

## Quick Start

1. **Fork** the repository
2. **Clone** your fork: `git clone https://github.com/YOUR_USERNAME/agent-ros-bridge.git`
3. **Install** dev dependencies: `pip install -e ".[dev]"`
4. **Create** a branch: `git checkout -b feature/your-feature`
5. **Make** your changes
6. **Test**: `pytest`
7. **Submit** a PR

## Development Setup

```bash
# Clone your fork
git clone https://github.com/YOUR_USERNAME/agent-ros-bridge.git
cd agent-ros-bridge

# Create virtual environment
python -m venv venv
source venv/bin/activate  # or `venv\Scripts\activate` on Windows

# Install in editable mode with dev dependencies
pip install -e ".[dev]"

# Verify installation
agent-ros-bridge --version
```

## Code Standards

### Python Style

We use:
- **Ruff** for linting
- **Black** for formatting
- **MyPy** for type checking

```bash
# Run all checks
make check

# Or individually
ruff check .
black --check .
mypy agent_ros_bridge
```

### Commit Messages

Use [Conventional Commits](https://www.conventionalcommits.org/):

```
feat: add new transport protocol
fix: resolve WebSocket reconnection issue
docs: update API examples
refactor: simplify bridge initialization
test: add fleet orchestrator tests
```

### Testing (TDD Workflow)

We follow **Test-Driven Development (TDD)**:
1. Write failing test first
2. Write minimal code to pass
3. Refactor while keeping tests green

```bash
# Run unit tests (fast, no Docker needed)
pytest tests/unit -v

# Run with coverage
pytest tests/unit --cov=agent_ros_bridge --cov-report=html

# Run E2E tests (requires ROS2 Docker container)
# Start the container first:
docker-compose -f docker-compose.ros2.yml up -d
# Then run E2E tests:
pytest tests/e2e -v

# Run integration tests (requires ROS2 Docker container)
pytest tests/integration -v

# Run specific test file
pytest tests/unit/test_bridge.py

# Run specific test
pytest tests/unit/test_bridge.py::test_bridge_initialization
```

**CI/CD runs:** Unit tests only (E2E/Integration require local Docker)
**Local development:** Run all test suites before committing

## What to Contribute

### 🐛 Bug Reports

- Check existing issues first
- Include minimal reproduction code
- Include environment details (OS, Python version, ROS version)
- Include error messages and logs

### 💡 Feature Requests

- Open a GitHub Discussion first for major features
- Describe the use case
- Explain why existing solutions don't work

### 📝 Documentation

- Fix typos
- Add examples
- Improve clarity
- Translate to other languages

### 🧪 Code Contributions

**Good first issues:**
- Documentation improvements
- Additional examples
- Test coverage
- Type hints

**Advanced contributions:**
- New transport protocols
- New robot connectors
- Performance optimizations
- Security enhancements

## Pull Request Process

1. **Update documentation** if needed
2. **Add tests** for new features
3. **Ensure all checks pass**: `make check`
4. **Fill out the PR template** completely
5. **Request review** from maintainers

## Code Review

- Be respectful and constructive
- Explain the "why" behind suggestions
- Approve when satisfied
- Ask questions if unclear

## Security

See [SECURITY.md](SECURITY.md) for vulnerability reporting.

**Never**:
- Commit secrets or tokens
- Disable security features for convenience
- Expose internal systems in examples

## Questions?

- [GitHub Discussions](https://github.com/webthree549-bot/agent-ros-bridge/discussions)
- [Discord](https://discord.gg/agent-ros-bridge)

## Recognition

Contributors will be:
- Listed in [CONTRIBUTORS.md](CONTRIBUTORS.md)
- Mentioned in release notes
- Added to the Hall of Fame (significant contributions)

---

Thank you for making Agent ROS Bridge better! 🚀
