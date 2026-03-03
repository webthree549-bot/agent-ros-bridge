# ADR-0006: Test-Driven Development Workflow

## Status

**Accepted**

## Context

The project experienced quality issues in early versions (v0.4.0 retraction due to false claims). Root causes included:
- Features implemented before tests
- Insufficient test coverage
- No systematic testing approach
- Integration issues discovered late

We needed a development workflow that ensures:
- Features are properly tested
- Claims are verifiable
- Regressions are caught early
- Code quality is maintained

## Decision

We will adopt **Test-Driven Development (TDD)** as the primary development workflow with the following strict rules:

### TDD Cycle

```
┌─────────┐     ┌─────────┐     ┌─────────┐
│  Red    │ ──► │  Green  │ ──► │ Refactor│
│ (failing│     │ (passing│     │ (clean  │
│  test)  │     │  test)  │     │  code)  │
└─────────┘     └─────────┘     └────┬────┘
     ▲                               │
     └───────────────────────────────┘
```

### Rules

1. **Write Tests FIRST** — Before any implementation code
2. **Tests Must Fail** — Verify they catch missing features
3. **Minimum Code** — Only write what's needed to pass
4. **Refactor After Green** — Clean up with passing tests
5. **Commit Pattern:**
   - `test: add tests for feature X` (Red)
   - `feat: implement feature X` (Green)
   - `refactor: clean up feature X` (Refactor)

### Test Structure

```
tests/
├── unit/              # Unit tests (fast, isolated)
│   ├── test_core.py
│   ├── test_auth.py
│   └── test_*.py
├── integration/       # Integration tests
│   └── test_integration.py
├── e2e/              # End-to-end tests
│   └── test_ros2_real_bridge.py
├── physical/         # Hardware-in-loop tests
│   └── test_physical_robot.py
└── conftest.py       # Shared fixtures
```

### Coverage Requirements

| Module | Minimum Coverage |
|--------|------------------|
| Core (auth, config, messages) | 90% |
| Transports | 80% |
| Connectors | 75% |
| Integrations | 70% |
| Overall | 80% |

### CI Enforcement

```yaml
# In CI pipeline
- name: Test with coverage
  run: pytest --cov=agent_ros_bridge --cov-fail-under=80

- name: Lint
  run: ruff check .  # Must pass

- name: Format check
  run: black --check .  # Must pass

- name: Type check
  run: mypy agent_ros_bridge  # Must pass (no || true)
```

### Pre-commit Hooks

```yaml
- repo: local
  hooks:
    - id: check-tdd
      name: Check TDD Compliance
      entry: python scripts/check_tdd.py
      stages: [pre-commit]
```

## Consequences

### Positive

- **Quality** — Higher code quality and fewer bugs
- **Confidence** — Changes can be made without fear
- **Documentation** — Tests serve as executable documentation
- **Design** — TDD leads to better modular design
- **Regression Prevention** — CI catches breaking changes

### Negative

- **Initial Velocity** — Slower at first (more code to write)
- **Learning Curve** — Team must learn TDD discipline
- **Maintenance** — Tests must be maintained alongside code
- **Time Investment** — More time spent on testing

### Neutral

- **Test Code Ratio** — ~1:1 test code to production code
- **CI Time** — Longer CI runs due to comprehensive testing

## TDD Violations and Remediation

### Past Violations

| Feature | Violation | Remediation |
|---------|-----------|-------------|
| ROS1 Connector | Implemented before tests | Added `test_ros1_connector_tdd.py` |
| gRPC Transport | Implemented before tests | Added `test_grpc_transport_tdd.py` |
| ROS2 Connector | Partial tests | Completed test coverage |

### Current Compliance

- ✅ All new features follow TDD
- ✅ Pre-commit hooks enforce test presence
- ✅ CI blocks PRs without tests
- ✅ Coverage gates at 80%

## Alternatives Considered

### Test-After Development

**Rejected:** Leads to untestable code, lower coverage, and missing edge cases.

### No Formal Testing Process

**Rejected:** Resulted in v0.4.0 retraction. Unacceptable for production code.

### BDD (Behavior-Driven Development)

**Considered:** BDD is complementary to TDD. We use BDD-style test names within TDD workflow.

## References

- [TDD Workflow Documentation](../TDD_WORKFLOW.md)
- [Test Strategy](../TEST_STRATEGY.md)
- [Test Directory](../../tests/)
- [Kent Beck - TDD Book](https://www.amazon.com/Test-Driven-Development-Kent-Beck/dp/0321146530)
