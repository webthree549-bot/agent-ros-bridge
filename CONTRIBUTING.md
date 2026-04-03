# Contributing to Agent ROS Bridge

Thank you for your interest in contributing! This document provides guidelines for contributing to the project.

## Code of Conduct

- Be respectful and inclusive
- Focus on constructive feedback
- Respect differing viewpoints
- Prioritize safety in all discussions

## How to Contribute

### Reporting Bugs

1. **Check existing issues** first
2. **Use the bug template** when creating a new issue
3. **Include**:
   - Steps to reproduce
   - Expected vs actual behavior
   - Environment details (OS, version, etc.)
   - Logs or error messages

### Suggesting Features

1. **Open a discussion** first for major features
2. **Describe the use case**
3. **Explain the benefit** to users
4. **Consider safety implications** (especially for robotics)

### Pull Requests

#### Before Submitting

1. **Fork the repository**
2. **Create a feature branch**: `git checkout -b feature/my-feature`
3. **Write tests** (TDD required)
4. **Run the test suite**: `pytest tests/`
5. **Ensure coverage** stays above 65%
6. **Update documentation**
7. **Follow code style** (black + ruff)

#### PR Checklist

- [ ] Tests pass (`pytest tests/`)
- [ ] Coverage maintained (`pytest --cov`)
- [ ] Code formatted (`black .`)
- [ ] Linting passes (`ruff check .`)
- [ ] Type hints added
- [ ] Documentation updated
- [ ] Changelog updated
- [ ] Security implications considered

#### PR Template

```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Breaking change
- [ ] Documentation update

## Testing
- [ ] Unit tests added/updated
- [ ] E2E tests pass
- [ ] Manual testing performed

## Safety Check
- [ ] No unsafe robot operations
- [ ] Human-in-the-loop preserved
- [ ] Shadow mode unaffected

## Checklist
- [ ] Code follows style guide
- [ ] Self-review completed
- [ ] Comments added for complex logic
- [ ] Documentation updated
```

## Development Setup

### Prerequisites

```bash
# Python 3.11+
python --version

# Git
git --version

# Docker (optional)
docker --version
```

### Setup

```bash
# Clone repository
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
cd agent-ros-bridge

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -e ".[dev]"

# Install pre-commit hooks
pre-commit install

# Verify setup
pytest tests/unit/ -v
```

### Running Tests

```bash
# All tests
pytest tests/ -v

# Specific test file
pytest tests/unit/test_gateway.py -v

# With coverage
pytest tests/ --cov=agent_ros_bridge --cov-report=html

# E2E tests (requires Docker)
pytest tests/e2e/ -v

# Safety tests only
pytest tests/unit/safety/ -v
```

### Code Style

We use:
- **black** for formatting
- **ruff** for linting
- **mypy** for type checking

```bash
# Format code
black .

# Check linting
ruff check .

# Fix auto-fixable issues
ruff check . --fix

# Type checking
mypy agent_ros_bridge/
```

## Project Structure

```
agent_ros_bridge/
├── agent_ros_bridge/      # Main package
│   ├── gateway_v2/        # Core gateway
│   ├── integrations/      # Protocol adapters
│   ├── safety/            # Safety systems
│   ├── shadow/            # Shadow mode
│   └── web/               # Dashboard
├── tests/                 # Test suite
│   ├── unit/              # Unit tests
│   ├── e2e/               # End-to-end tests
│   └── security/          # Security tests
├── docs/                  # Documentation
├── config/                # Configuration files
└── scripts/               # Utility scripts
```

## Writing Tests

### Unit Tests

```python
import pytest
from agent_ros_bridge.safety.validator import SafetyValidator

class TestSafetyValidator:
    """Test safety validation logic."""
    
    def test_dangerous_action_detection(self):
        validator = SafetyValidator()
        
        result = validator.validate(
            action="emergency_stop",
            parameters={}
        )
        
        assert result.is_dangerous == True
        assert result.requires_approval == True
    
    def test_safe_action_approval(self):
        validator = SafetyValidator()
        
        result = validator.validate(
            action="get_status",
            parameters={}
        )
        
        assert result.is_dangerous == False
```

### TDD Requirements

All new features must follow Test-Driven Development:

1. **Write test first** (red)
2. **Implement feature** (green)
3. **Refactor** (while keeping tests green)

### Test Coverage

- Minimum: 65% overall
- Safety-critical code: 90%+
- New features: 80%+

## Documentation

### Code Documentation

```python
def validate_command(action: str, parameters: dict) -> ValidationResult:
    """
    Validate a robot command for safety.
    
    Args:
        action: Command action (e.g., 'move', 'rotate')
        parameters: Command parameters
        
    Returns:
        ValidationResult with safety status
        
    Raises:
        SafetyError: If command violates safety constraints
        
    Example:
        >>> result = validate_command('move', {'distance': 2.0})
        >>> result.is_safe
        True
    """
    # Implementation
```

### Documentation Files

Update relevant docs when adding features:
- `README.md` - User-facing overview
- `docs/API_REFERENCE.md` - API changes
- `docs/DEPLOYMENT_GUIDE.md` - Deployment changes
- `CHANGELOG.md` - Release notes

## Safety Considerations

**CRITICAL**: All contributions must consider safety implications.

### Questions to Ask

1. Could this change allow unsafe robot operations?
2. Does it bypass human-in-the-loop?
3. Could it affect shadow mode data integrity?
4. Are there edge cases that could cause harm?

### Safety-Critical Changes

Require additional review:
- Safety configuration changes
- Human approval logic
- Shadow mode logging
- Emergency stop functionality

Tag PRs with `safety-critical` label.

## Security

### Reporting Security Issues

**DO NOT** open public issues for security bugs.

Instead:
1. Email: security@agent-ros-bridge.ai
2. Encrypt with PGP if sensitive
3. Allow 30 days for response
4. Coordinate disclosure

### Security Best Practices

- Never commit secrets
- Use parameterized queries
- Validate all inputs
- Follow OWASP guidelines
- Run security scans: `bandit -r agent_ros_bridge/`

## Performance

### Benchmarking

Add benchmarks for performance-critical code:

```python
# tests/benchmarks/test_performance.py
import pytest
import time

def test_command_latency(benchmark):
    def send_command():
        # Command sending logic
        pass
    
    result = benchmark(send_command)
    assert result.stats.mean < 0.050  # 50ms max
```

### Profiling

```bash
# CPU profiling
python -m cProfile -o profile.stats script.py

# Memory profiling
python -m memory_profiler script.py
```

## Release Process

1. Update version in `__init__.py`
2. Update `CHANGELOG.md`
3. Create git tag: `git tag v0.7.0`
4. Push tag: `git push origin v0.7.0`
5. GitHub Actions builds and publishes

## Community

### Communication Channels

- **GitHub Issues**: Bug reports, feature requests
- **GitHub Discussions**: General questions, ideas
- **Slack**: Real-time chat (invite-only)
- **Email**: contact@agent-ros-bridge.ai

### Recognition

Contributors will be:
- Listed in CONTRIBUTORS.md
- Mentioned in release notes
- Invited to Slack community

## Questions?

- Read [FAQ](docs/FAQ.md)
- Open a [Discussion](https://github.com/agent-ros-bridge/discussions)
- Email: contact@agent-ros-bridge.ai

## License

By contributing, you agree that your contributions will be licensed under the MIT License.

---

**Thank you for making AI robotics safer! 🤖🛡️**