# Test-Driven Development (TDD) Strategy

## TDD Cycle

```
┌─────────┐    ┌─────────┐    ┌─────────┐
│  RED    │───▶│ GREEN   │───▶│ REFACTOR│
│(Write   │    │(Minimum │    │(Improve │
│ Test)   │    │ Code)   │    │ Code)   │
└─────────┘    └─────────┘    └─────────┘
     ▲                              │
     └──────────────────────────────┘
```

## Test Pyramid for Agent ROS Bridge

```
                    /\
                   /  \
                  / E2E \         <- 5% of tests
                 / (Slow) \        <- Real systems
                /__________\
               /            \
              / Integration   \    <- 15% of tests
             /  (Medium)       \   <- Docker/DB/ROS
            /____________________\
           /                      \
          /       Unit Tests        \  <- 80% of tests
         /         (Fast)            \ <- <100ms each
        /______________________________\
```

## Test Categories

### 1. Unit Tests (80% of tests)

**Characteristics:**
- Fast (< 100ms per test)
- Isolated (no external dependencies)
- Deterministic (always same result)
- Single responsibility

**Coverage Targets:**
| Module | Current | Target | Priority |
|--------|---------|--------|----------|
| Core | 43% | 95% | 🔴 High |
| Transports | 52% | 90% | 🔴 High |
| Safety | 79% | 95% | 🟡 Medium |
| Blueprint | 90% | 95% | 🟡 Medium |
| Utils | 91% | 95% | 🟢 Low |

**Example TDD Cycle:**

```python
# Step 1: RED - Write failing test
def test_lcm_message_encode():
    msg = LCMMessage(channel="test", data=b"hello")
    encoded = msg.encode()
    assert len(encoded) > 0
    assert b"test" in encoded

# Step 2: GREEN - Minimum implementation
class LCMMessage:
    def encode(self) -> bytes:
        return self.channel.encode() + b":" + self.data

# Step 3: REFACTOR - Improve implementation
class LCMMessage:
    def encode(self) -> bytes:
        # Proper binary encoding with length prefixes
        channel_bytes = self.channel.encode()
        header = struct.pack("!I", len(channel_bytes))
        return header + channel_bytes + self.data
```

### 2. Integration Tests (15% of tests)

**Characteristics:**
- Medium speed (1-5s per test)
- Real dependencies (Docker, Redis, ROS)
- Test component interactions

**Test Areas:**
- Transport layer with real protocols
- Database interactions
- ROS2 integration
- Service orchestration

**Example:**
```python
@pytest.mark.integration
def test_ros2_bridge_integration():
    """Test bridge with real ROS2."""
    with ROS2Container() as ros:
        bridge = Bridge()
        bridge.connect_to_ros(ros)
        
        # Publish test message
        bridge.publish("/test", {"data": "hello"})
        
        # Verify received
        msg = ros.wait_for_message("/test", timeout=5)
        assert msg["data"] == "hello"
```

### 3. E2E Tests (5% of tests)

**Characteristics:**
- Slow (5-30s per test)
- Full system
- Real-world scenarios

**Test Scenarios:**
- AI agent sends command → Robot moves
- Multi-robot coordination
- Failure recovery
- Performance benchmarks

**Example:**
```python
@pytest.mark.e2e
def test_ai_to_robot_command():
    """Full flow: AI → Bridge → ROS → Robot."""
    # Start full system
    with TestEnvironment() as env:
        ai = env.connect_ai_agent()
        robot = env.connect_robot()
        
        # AI sends command
        ai.say("move forward 1 meter")
        
        # Verify robot received
        cmd = robot.wait_for_command(timeout=10)
        assert cmd.action == "move"
        assert cmd.distance == 1.0
```

## TDD Workflow by Feature

### Feature: LCM Transport

```
1. Write test for LCMMessage encoding
   → Test fails (RED)

2. Implement LCMMessage.encode()
   → Test passes (GREEN)

3. Refactor for proper binary format
   → Test still passes (REFACTOR)

4. Write test for LCMPublisher
   → Test fails (RED)

5. Implement LCMPublisher
   → Test passes (GREEN)

6. Continue cycle...
```

### Feature: Blueprint Autoconnect

```
1. Test: Modules with matching streams should autoconnect
   → Fails (RED)

2. Implement stream matching logic
   → Passes (GREEN)

3. Refactor for clarity
   → Passes (REFACTOR)

4. Test: Direction validation (out → in only)
   → Fails (RED)

5. Add direction validation
   → Passes (GREEN)
```

## Test Naming Convention

```python
# Format: test_<unit>_<scenario>_<expected_result>

def test_lcm_transport_start_with_valid_config_succeeds():
    pass

def test_lcm_transport_start_with_invalid_port_raises_error():
    pass

def test_blueprint_autoconnect_matching_streams_creates_connections():
    pass

def test_safety_validator_trajectory_exceeding_velocity_limit_fails():
    pass
```

## Mock Strategy

### When to Mock

| Dependency | Mock? | Reason |
|------------|-------|--------|
| ROS2 | Yes | External service, slow |
| Database | Yes | External dependency |
| Network | Yes | Non-deterministic |
| File System | Sometimes | Use tmpdir |
| Time | Yes | Non-deterministic |
| Random | Yes | Non-deterministic |

### Mock Examples

```python
# Mock ROS2
@pytest.fixture
def mock_ros2():
    with patch('rclpy.create_node') as mock:
        mock_node = MagicMock()
        mock.return_value = mock_node
        yield mock_node

# Mock Time
@pytest.fixture
def frozen_time():
    with patch('time.time', return_value=1234567890):
        yield

# Mock Network
@pytest.fixture
def mock_websocket():
    with patch('websockets.connect') as mock:
        mock_conn = AsyncMock()
        mock.return_value = mock_conn
        yield mock_conn
```

## Property-Based Testing

Use Hypothesis for property-based tests:

```python
from hypothesis import given, strategies as st

@given(st.text(), st.binary())
def test_lcm_message_roundtrip(channel, data):
    """Encoding then decoding should return original."""
    original = LCMMessage(channel=channel, data=data)
    encoded = original.encode()
    decoded = LCMMessage.decode(encoded)
    
    assert decoded.channel == original.channel
    assert decoded.data == original.data

@given(st.floats(min_value=0, max_value=10))
def test_rate_limiter_allows_under_limit(requests_per_sec):
    limiter = RateLimiter(max_requests=10, window=1)
    
    # Should allow 5 requests
    for _ in range(5):
        assert limiter.allow_request("client")
```

## Mutation Testing

Use mutmut to verify test quality:

```bash
# Install
pip install mutmut

# Run mutation testing
mutmut run --paths-to-mutate=agent_ros_bridge

# View results
mutmut results
mutmut show 1  # Show details of mutation 1
```

## Continuous Testing

### Pre-commit (Fast)
- Unit tests only
- < 30 seconds
- Block commit on failure

### Pre-push (Medium)
- Integration tests
- < 5 minutes
- Warn on failure

### CI/CD (Complete)
- All tests
- Coverage check
- Security scan

## Test Data Management

### Fixtures

```python
@pytest.fixture
def sample_trajectory():
    return {
        "waypoints": [
            {"x": 0, "y": 0},
            {"x": 1, "y": 0},
            {"x": 2, "y": 0}
        ],
        "velocities": [0.5, 0.5, 0.5]
    }

@pytest.fixture
def safety_limits():
    return {
        "max_linear_velocity": 1.0,
        "max_angular_velocity": 2.0,
        "workspace_bounds": {"x_min": -10, "x_max": 10}
    }
```

### Factories

```python
import factory

class RobotModuleFactory(factory.Factory):
    class Meta:
        model = RobotModule
    
    name = factory.Sequence(lambda n: f"robot_{n}")
    max_speed = 1.0

class MessageFactory(factory.Factory):
    class Meta:
        model = Message
    
    header = factory.SubFactory(HeaderFactory)
    payload = factory.Dict({"data": "test"})
```

## Coverage Strategy

### Minimum Coverage by Module

```yaml
# .codecov.yml
coverage:
  status:
    project:
      default:
        target: 80%
        threshold: 2%
    patch:
      default:
        target: 90%

  ignore:
    - "tests/**/*"
    - "docs/**/*"
    - "examples/**/*"
```

### Coverage Reports

Generate reports in CI:

```bash
pytest --cov=agent_ros_bridge \
       --cov-report=html \
       --cov-report=xml \
       --cov-report=term-missing
```

## Performance Testing

### Benchmarks

```python
import pytest
import time

def test_safety_validation_performance():
    """Safety validation should complete in <10ms."""
    validator = SafetyValidator()
    trajectory = generate_large_trajectory(1000)
    
    start = time.time()
    result = validator.validate(trajectory)
    elapsed = (time.time() - start) * 1000
    
    assert elapsed < 10, f"Took {elapsed}ms"

@pytest.mark.benchmark
def test_lcm_throughput(benchmark):
    """LCM should handle 100K messages/sec."""
    transport = LCMTransport({})
    
    def publish_1000():
        for i in range(1000):
            transport.publish(f"msg_{i}")
    
    benchmark(publish_1000)
```

## Security Testing

### Static Analysis

```bash
# Bandit
bandit -r agent_ros_bridge -f json -o bandit.json

# Safety (dependencies)
safety check

# Semgrep
semgrep --config=auto agent_ros_bridge
```

### Dynamic Testing

```python
def test_sql_injection_prevention():
    malicious_input = "'; DROP TABLE robots; --"
    result = InputValidator.validate_robot_id(malicious_input)
    assert not result.valid

def test_xss_prevention():
    malicious_input = "<script>alert('xss')</script>"
    sanitized = sanitize_input(malicious_input)
    assert "<script>" not in sanitized
```

## Chaos Testing

```python
@pytest.mark.chaos
def test_bridge_survives_network_partition():
    with NetworkPartition(duration=10):
        # Bridge should continue operating
        assert bridge.is_healthy()

@pytest.mark.chaos  
def test_bridge_recovers_from_ros_disconnect():
    ros.kill()
    time.sleep(5)
    ros.restart()
    
    # Bridge should reconnect
    assert bridge.wait_for_ros(timeout=30)
```

## Test Maintenance

### Regular Tasks

- [ ] Review flaky tests weekly
- [ ] Update tests when requirements change
- [ ] Remove obsolete tests
- [ ] Refactor tests for clarity
- [ ] Update test data

### Anti-patterns to Avoid

1. **Testing implementation details**
   ```python
   # BAD
   def test_internal_counter():
       obj = MyClass()
       obj._counter = 5  # Testing private state
       assert obj._counter == 5
   
   # GOOD
   def test_public_behavior():
       obj = MyClass()
       obj.increment()
       assert obj.get_count() == 1
   ```

2. **Brittle tests**
   ```python
   # BAD
   def test_error_message():
       with pytest.raises(ValueError) as exc:
           do_something()
       assert str(exc.value) == "Exact error message"  # Brittle
   
   # GOOD
   def test_error_type():
       with pytest.raises(ValueError, match="error"):
           do_something()
   ```

3. **Slow tests**
   ```python
   # BAD
   def test_with_sleep():
       do_something()
       time.sleep(5)  # Don't do this
       assert result
   
   # GOOD
   def test_with_async():
       result = await do_something()
       assert result
   ```

## Success Metrics

| Metric | Target | Current |
|--------|--------|---------|
| Unit Test Coverage | 95% | 32% |
| Integration Test Coverage | 70% | - |
| E2E Test Coverage | 50% | - |
| Test Execution Time | <5 min | - |
| Flaky Test Rate | <1% | - |
| Mutation Score | >80% | - |
