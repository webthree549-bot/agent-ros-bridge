# TDD Workflow for agent-ros-bridge

This project follows **Test Driven Development (TDD)** principles.

## TDD Cycle

```
┌─────────┐     ┌─────────┐     ┌─────────┐
│  Red    │ ──► │  Green  │ ──► │ Refactor│
│ (failing│     │ (passing│     │ (clean  │
│  test)  │     │  test)  │     │  code)  │
└─────────┘     └─────────┘     └────┬────┘
     ▲                               │
     └───────────────────────────────┘
```

## Rules

1. **Write tests FIRST** - Before any implementation
2. **Tests must fail initially** - Verify they catch missing features
3. **Minimum code to pass** - Only write what's needed
4. **Refactor after green** - Clean up once tests pass

## Example Workflow

### Step 1: Write Failing Test (Red)

```python
# tests/unit/test_my_feature.py
def test_new_feature_should_work():
    """Test that new feature returns expected result."""
    from agent_ros_bridge.my_module import MyFeature
    
    feature = MyFeature()
    result = feature.do_something()
    
    assert result == "expected"  # This will fail - feature doesn't exist yet
```

Run test to confirm it fails:
```bash
pytest tests/unit/test_my_feature.py -v
# Expected: FAILED - ImportError or AttributeError
```

### Step 2: Implement Minimum Code (Green)

```python
# agent_ros_bridge/my_module.py
class MyFeature:
    def do_something(self):
        return "expected"  # Minimum to pass test
```

Run test to confirm it passes:
```bash
pytest tests/unit/test_my_feature.py -v
# Expected: PASSED
```

### Step 3: Refactor (Keep Green)

Clean up the implementation while keeping tests passing:
```python
# agent_ros_bridge/my_module.py
class MyFeature:
    """Clean, documented implementation."""
    
    def do_something(self) -> str:
        """Return the expected result."""
        # More robust implementation here
        return self._compute_result()
    
    def _compute_result(self) -> str:
        # Internal helper
        return "expected"
```

## Commit Messages

Follow conventional commits with TDD context:

```
test: add tests for feature X          # Red phase - failing tests
feat: implement feature X              # Green phase - make tests pass
refactor: clean up feature X           # Refactor phase
```

## What NOT To Do

❌ **Don't write implementation first**
```python
# WRONG - implementation before tests
class BadFeature:
    def complex_logic(self):
        # Lots of code...
        return result

# Then try to write tests... might miss edge cases!
```

❌ **Don't skip the Red phase**
```python
# WRONG - writing tests that pass immediately
# You haven't verified the test actually catches failures
```

❌ **Don't over-implement**
```python
# WRONG - adding features not in tests
class OverEngineered:
    def required_feature(self):
        return "ok"
    
    def extra_feature_1(self):  # Not tested!
        pass
    
    def extra_feature_2(self):  # Not tested!
        pass
```

## Test Structure

```python
class TestFeatureName:
    """Test suite for FeatureName following TDD."""
    
    def test_should_handle_happy_path(self):
        """Test normal operation."""
        pass
    
    def test_should_handle_edge_case_1(self):
        """Test boundary condition."""
        pass
    
    def test_should_handle_error_condition(self):
        """Test error handling."""
        pass
```

## Running Tests

```bash
# Run all tests
pytest tests/unit/

# Run specific test file
pytest tests/unit/test_my_feature.py

# Run with coverage
pytest tests/unit/ --cov=agent_ros_bridge

# Run TDD cycle (watch mode)
ptw tests/unit/test_my_feature.py
```

## Coverage Requirements

- Minimum 80% code coverage
- 100% coverage for critical paths
- All public APIs must have tests

## Current Status

### Recent TDD Violations (Fixed)
- ❌ ROS1 connector - implemented before tests
- ❌ gRPC transport - implemented before tests

**Fix**: Added TDD tests retroactively (`test_ros1_connector_tdd.py`, `test_grpc_transport_tdd.py`)

### Following TDD Going Forward
All new features must follow:
1. Write tests first
2. See them fail
3. Implement
4. See them pass
5. Refactor

## Resources

- [Test Driven Development by Kent Beck](https://www.amazon.com/Test-Driven-Development-Kent-Beck/dp/0321146530)
- [TDD Manifesto](https://tddmanifesto.com/)
- [Clean Code by Robert C. Martin](https://www.amazon.com/Clean-Code-Handbook-Software-Craftsmanship/dp/0132350882)
