# ADR-0007: Code Quality and Linting Standards

## Status

**Accepted**

## Context

As the project grew to 18,000+ lines of code with multiple contributors, code quality issues emerged:
- Inconsistent code style across files
- Missing docstrings in public APIs
- Type hint inconsistencies
- Unused imports and variables
- Security issues (clear-text logging of secrets)

Code review was spending too much time on style issues rather than architecture. We needed automated, enforceable standards.

## Decision

We will implement **comprehensive code quality standards** enforced through automated tooling in CI/CD.

### Tool Stack

| Tool | Purpose | Configuration |
|------|---------|---------------|
| **Black** | Code formatting | Line length: 100 |
| **Ruff** | Fast linting | Python 3.8+ target |
| **MyPy** | Type checking | Strict mode for new code |
| **Bandit** | Security scanning | Via pre-commit |
| **Trivy** | Container scanning | Critical/High severity |

### Code Style Standards

#### Formatting (Black)

```toml
[tool.black]
line-length = 100
target-version = ["py38", "py39", "py310", "py311", "py312"]
```

#### Linting (Ruff)

```toml
[tool.ruff.lint]
select = [
    "E",   # pycodestyle errors
    "F",   # Pyflakes
    "W",   # pycodestyle warnings
    "I",   # isort
    "N",   # pep8-naming
    "D",   # pydocstyle
    "UP",  # pyupgrade
    "B",   # flake8-bugbear
    "C4",  # flake8-comprehensions
    "SIM", # flake8-simplify
    "ARG", # flake8-unused-arguments
]
ignore = [
    "D100",  # Missing docstring in public module
    "D104",  # Missing docstring in public package
    "D107",  # Missing docstring in __init__ (covered in ADR-0007 cleanup)
]
```

### Documentation Standards

#### Google Style Docstrings

```python
def connect(self, uri: str, timeout: float = 30.0) -> bool:
    """Connect to robot at specified URI.
    
    Args:
        uri: Connection URI (e.g., 'ros2://localhost:9090')
        timeout: Connection timeout in seconds
        
    Returns:
        True if connection successful, False otherwise
        
    Raises:
        ConnectionError: If connection fails after retries
        
    Example:
        >>> connector = ROS2Connector()
        >>> success = await connector.connect('ros2://localhost')
    """
```

### Type Hint Requirements

```python
# Required: Function signatures
def publish(self, topic: str, message: Dict[str, Any]) -> bool:
    ...

# Required: Class attributes
class Bridge:
    transports: Dict[str, Transport]
    running: bool = False
    
# Required: Return types
async def get_status(self) -> RobotStatus:
    ...
```

### CI Enforcement

```yaml
# .github/workflows/ci.yml
jobs:
  quality:
    steps:
      - name: Lint with Ruff
        run: ruff check .
        # Must pass - no warnings allowed

      - name: Format check with Black
        run: black --check .
        # Must pass - no formatting issues

      - name: Type check with MyPy
        run: mypy agent_ros_bridge
        # Must pass - no type errors
```

### Pre-commit Hooks

```yaml
# .pre-commit-config.yaml
repos:
  - repo: https://github.com/psf/black
    hooks:
      - id: black
        
  - repo: https://github.com/astral-sh/ruff-pre-commit
    hooks:
      - id: ruff
        args: ['--fix', '--exit-non-zero-on-fix']
        
  - repo: https://github.com/pre-commit/mirrors-mypy
    hooks:
      - id: mypy
```

## Consequences

### Positive

- **Consistency** — Uniform code style across entire codebase
- **Automation** — No manual style review needed
- **Early Detection** — Issues caught in pre-commit, not PR review
- **Security** — Automated security scanning prevents vulnerabilities
- **Professionalism** — High code quality standards

### Negative

- **Initial Friction** — Contributors must learn and follow standards
- **Tool Dependencies** — Project depends on specific tool versions
- **False Positives** — Occasionally need `# noqa` comments
- **CI Time** — Additional checks increase build time

### Neutral

- **Refactoring** — Large initial effort to bring codebase into compliance
- **Learning Curve** — New contributors must understand the standards

## Cleanup Results (March 2026)

| Metric | Before | After |
|--------|--------|-------|
| Linting Errors | 398 | 0 ✅ |
| Docstring Warnings | 69 | 0 ✅ |
| Formatting Issues | 12 files | 0 ✅ |
| Security Issues | 2 | 0 ✅ |

### Tools Applied

```bash
# Auto-fix formatting
black .

# Auto-fix linting
ruff check --fix .

# Manual docstring additions
# 46 __init__ methods documented
# 3 packages documented

# Security fixes
# - Removed clear-text JWT logging
# - Fixed token validation timing attacks
```

## Alternatives Considered

### Manual Code Review Only

**Rejected:** Doesn't scale, inconsistent enforcement, wastes reviewer time on style.

### flake8 + isort + pydocstyle

**Rejected:** Ruff is faster (Rust-based) and consolidates all tools. Single configuration.

### Less Strict Standards

**Rejected:** Led to quality issues in v0.4.x. Strict standards prevent technical debt.

## References

- [Code Quality Assessment](../../DEVOPS_TDD_ASSESSMENT.md)
- [Cleanup Report](../../CLEANUP_REPORT.md)
- [pyproject.toml](../../pyproject.toml) — Tool configuration
- [Black Documentation](https://black.readthedocs.io/)
- [Ruff Documentation](https://docs.astral.sh/ruff/)
