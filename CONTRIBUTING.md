# Contributing to OpenClaw ROS Bridge

Thank you for your interest in contributing! This document provides guidelines for contributing to the project.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Development Workflow](#development-workflow)
- [Coding Standards](#coding-standards)
- [Testing](#testing)
- [Documentation](#documentation)
- [Pull Request Process](#pull-request-process)
- [Release Process](#release-process)

## Code of Conduct

This project adheres to the [Contributor Covenant Code of Conduct](CODE_OF_CONDUCT.md). By participating, you are expected to uphold this code.

## Getting Started

### Prerequisites

- Python 3.8+
- Git
- Docker (optional, for testing)
- ROS1 Noetic or ROS2 Humble/Jazzy (optional)

### Setup Development Environment

```bash
# Clone the repository
git clone https://github.com/webthree549-bot/openclaw-ros-bridge.git
cd openclaw-ros-bridge

# Install pre-commit hooks
pip install pre-commit
pre-commit install

# Install in editable mode with dev dependencies
pip install -e ".[dev,test]"
```

## Development Workflow

### 1. Create a Branch

```bash
git checkout -b feature/your-feature-name
# or
git checkout -b fix/your-bug-fix
```

Branch naming conventions:
- `feature/` - New features
- `fix/` - Bug fixes
- `docs/` - Documentation updates
- `refactor/` - Code refactoring
- `test/` - Test additions/improvements

### 2. Make Changes

- Write clean, readable code
- Follow [PEP 8](https://pep8.org/) style guide
- Add type hints where appropriate
- Update documentation as needed

### 3. Test Your Changes

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=openclaw_ros_bridge

# Run specific test
pytest tests/test_specific.py -v

# Run linting
ruff check .
black --check .
mypy openclaw_ros_bridge/
```

### 4. Commit Changes

We use [Conventional Commits](https://www.conventionalcommits.org/):

```bash
# Examples:
git commit -m "feat: add MQTT transport support"
git commit -m "fix: resolve ROS2 discovery issue"
git commit -m "docs: update installation guide"
git commit -m "refactor: simplify connector registry"
git commit -m "test: add integration tests for greenhouse plugin"
```

Commit types:
- `feat:` - New features
- `fix:` - Bug fixes
- `docs:` - Documentation
- `style:` - Formatting changes
- `refactor:` - Code refactoring
- `perf:` - Performance improvements
- `test:` - Tests
- `chore:` - Maintenance tasks

### 5. Push and Create PR

```bash
git push origin feature/your-feature-name
```

Then create a Pull Request on GitHub.

## Coding Standards

### Python Style Guide

- **Formatter**: Black with 100 character line length
- **Linter**: Ruff
- **Import Sorting**: isort with black profile
- **Type Checking**: mypy (optional but encouraged)

### Code Structure

```python
"""Module docstring."""

# Standard library imports
import os
from typing import Dict, List

# Third-party imports
import yaml

# Local imports
from openclaw_ros_bridge.base import BaseClass


class MyClass:
    """Class docstring.
    
    Attributes:
        attribute1: Description of attribute1.
    """
    
    def __init__(self, param1: str) -> None:
        """Initialize MyClass.
        
        Args:
            param1: Description of param1.
        """
        self.param1 = param1
    
    def method(self, arg: int) -> bool:
        """Method docstring.
        
        Args:
            arg: Description of arg.
            
        Returns:
            Description of return value.
            
        Raises:
            ValueError: When arg is invalid.
        """
        if arg < 0:
            raise ValueError("arg must be non-negative")
        return True
```

### Documentation Strings

Use Google-style docstrings:

```python
def function(arg1: int, arg2: str) -> bool:
    """Short description.
    
    Longer description if needed.
    
    Args:
        arg1: Description of arg1.
        arg2: Description of arg2.
        
    Returns:
        Description of return value.
        
    Raises:
        ValueError: When arg1 is invalid.
        TypeError: When arg2 is wrong type.
    """
    pass
```

## Testing

### Test Structure

```
tests/
├── unit/           # Unit tests
│   ├── test_core.py
│   └── test_transports/
├── integration/    # Integration tests
│   ├── test_ros2_connector.py
│   └── test_end_to_end.py
├── fixtures/       # Test fixtures
└── conftest.py     # Pytest configuration
```

### Writing Tests

```python
import pytest
from openclaw_ros_bridge.core import Message


class TestMessage:
    """Test Message class."""
    
    def test_message_creation(self):
        """Test creating a message."""
        msg = Message(header=Header())
        assert msg.header is not None
    
    def test_message_validation(self):
        """Test message validation."""
        with pytest.raises(ValueError):
            Message(header=None)


@pytest.mark.integration
class TestROS2Connector:
    """Integration tests for ROS2 connector."""
    
    @pytest.fixture
    async def connector(self):
        connector = ROS2Connector()
        yield connector
        await connector.disconnect()
    
    async def test_discovery(self, connector):
        """Test robot discovery."""
        robots = await connector.discover()
        assert isinstance(robots, list)
```

### Running Tests

```bash
# All tests
pytest

# Only unit tests
pytest tests/unit/

# Only integration tests
pytest tests/integration/ -m integration

# With coverage
pytest --cov=openclaw_ros_bridge --cov-report=html

# Parallel execution
pytest -n auto
```

## Documentation

### Building Documentation

```bash
# Install docs dependencies
pip install -e ".[docs]"

# Build docs
mkdocs build

# Serve docs locally
mkdocs serve
```

### Documentation Style

- Use clear, concise language
- Include code examples
- Add diagrams where helpful (Mermaid supported)
- Keep README.md up to date

## Pull Request Process

1. **Update documentation** - Update README.md, docstrings, and guides as needed
2. **Add tests** - All new features must include tests
3. **Update CHANGELOG.md** - Add entry under Unreleased section
4. **Ensure CI passes** - All checks must pass
5. **Request review** - Tag maintainers for review
6. **Address feedback** - Make requested changes
7. **Merge** - Maintainers will merge when approved

### PR Checklist

- [ ] Code follows style guidelines
- [ ] Tests added/updated
- [ ] Documentation updated
- [ ] CHANGELOG.md updated
- [ ] Pre-commit hooks pass
- [ ] CI passes
- [ ] Branch is up to date with main

## Release Process

1. Update version in `__init__.py`
2. Update CHANGELOG.md with release date
3. Create git tag: `git tag -a v1.2.3 -m "Release v1.2.3"`
4. Push tag: `git push origin v1.2.3`
5. GitHub Actions will automatically:
   - Create GitHub release
   - Build and push Docker images
   - Publish to PyPI
   - Update documentation

## Questions?

- Open an issue for bug reports or feature requests
- Start a discussion for general questions
- Contact maintainers: dev@openclaw-ros.org

## Attribution

This contributing guide is adapted from:
- [Atom Contributing Guide](https://github.com/atom/atom/blob/master/CONTRIBUTING.md)
- [Conventional Commits](https://www.conventionalcommits.org/)

Thank you for contributing!
