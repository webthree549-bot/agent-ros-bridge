# Test-Driven Development Guide for Agent ROS Bridge v0.6.1+

## TDD Philosophy

**Red → Green → Refactor**

1. **Red**: Write a failing test first
2. **Green**: Write minimal code to pass the test
3. **Refactor**: Clean up while keeping tests green

**No production code without a failing test first.**

---

## TDD Workflow for ROS Components

### Example: Safety Validator Node

#### Step 1: Write Failing Test (Red)

```python
# tests/unit/safety/test_validator.py
import pytest
from agent_ros_bridge.safety.validator import SafetyValidator

class TestSafetyValidator:
    """TDD for Safety Validator - Week 1 Sprint"""
    
    def test_validator_rejects_excessive_velocity(self):
        """RED: Validator should reject velocity > max_limit"""
        # Arrange
        validator = SafetyValidator()
        limits = SafetyLimits(max_linear_velocity=1.0)
        trajectory = Trajectory(
            velocities=[0.5, 1.5, 2.0]  # 2.0 > 1.0 limit
        )
        
        # Act
        result = validator.validate(trajectory, limits)
        
        # Assert
        assert result.approved is False
        assert "velocity exceeds limit" in result.rejection_reason
    
    def test_validator_approves_safe_trajectory(self):
        """RED: Validator should approve safe trajectory"""
        validator = SafetyValidator()
        limits = SafetyLimits(max_linear_velocity=1.0)
        trajectory = Trajectory(
            velocities=[0.1, 0.5, 0.8]  # All < 1.0
        )
        
        result = validator.validate(trajectory, limits)
        
        assert result.approved is True
        assert result.certificate is not None
```

**Run tests - they should FAIL:**
```bash
pytest tests/unit/safety/test_validator.py -v
# Expected: 2 failures (SafetyValidator doesn't exist yet)
```

#### Step 2: Minimal Implementation (Green)

```python
# src/agent_ros_bridge/safety/validator.py

class SafetyValidator:
    """Minimal implementation to pass tests"""
    
    def validate(self, trajectory, limits):
        """Check if trajectory is safe"""
        for velocity in trajectory.velocities:
            if velocity > limits.max_linear_velocity:
                return ValidationResult(
                    approved=False,
                    rejection_reason=f"velocity exceeds limit: {velocity} > {limits.max_linear_velocity}"
                )
        
        return ValidationResult(
            approved=True,
            certificate=SafetyCertificate()  # Minimal certificate
        )
```

**Run tests - they should PASS:**
```bash
pytest tests/unit/safety/test_validator.py -v
# Expected: 2 passed
```

#### Step 3: Refactor

```python
# src/agent_ros_bridge/safety/validator.py (refactored)

class SafetyValidator:
    """Hardware-enforced safety validator"""
    
    def __init__(self):
        self.validation_count = 0
        self.rejection_count = 0
    
    def validate(self, trajectory, limits):
        """
        Validate trajectory against safety limits.
        
        Args:
            trajectory: Motion trajectory to validate
            limits: SafetyLimits configuration
            
        Returns:
            ValidationResult with approval status and certificate
        """
        self.validation_count += 1
        
        # Check velocity constraints
        velocity_result = self._check_velocity(trajectory, limits)
        if not velocity_result.passed:
            self.rejection_count += 1
            return ValidationResult(
                approved=False,
                rejection_reason=velocity_result.reason
            )
        
        # All checks passed
        return ValidationResult(
            approved=True,
            certificate=self._generate_certificate(trajectory)
        )
    
    def _check_velocity(self, trajectory, limits):
        """Check velocity constraints"""
        for velocity in trajectory.velocities:
            if velocity > limits.max_linear_velocity:
                return CheckResult(
                    passed=False,
                    reason=f"velocity exceeds limit: {velocity} > {limits.max_linear_velocity}"
                )
        return CheckResult(passed=True)
    
    def _generate_certificate(self, trajectory):
        """Generate safety certificate"""
        return SafetyCertificate(
            plan_hash=trajectory.hash(),
            issued_at=time.time(),
            expires_at=time.time() + 30.0  # 30-second validity
        )
```

**Run tests - still PASS:**
```bash
pytest tests/unit/safety/test_validator.py -v
# Expected: 2 passed (refactoring didn't break anything)
```

---

## TDD Patterns for ROS Nodes

### Pattern 1: ROS Node with Services

```python
# tests/unit/ai/test_intent_parser.py

class TestIntentParser:
    """TDD for Intent Parser Node"""
    
    @pytest.fixture
    def intent_parser(self):
        """Create intent parser node for testing"""
        rclpy.init()
        node = IntentParserNode()
        yield node
        node.destroy_node()
        rclpy.shutdown()
    
    def test_parse_navigate_intent(self, intent_parser):
        """RED: Should parse 'go to kitchen' as NAVIGATE intent"""
        request = ParseIntent.Request()
        request.utterance = "go to kitchen"
        
        response = ParseIntent.Response()
        intent_parser.parse_intent_callback(request, response)
        
        assert response.intent.type == "NAVIGATE"
        assert response.intent.confidence > 0.8
        assert any(e.type == "LOCATION" and e.value == "kitchen" 
                   for e in response.intent.entities)
    
    def test_parse_with_low_confidence(self, intent_parser):
        """RED: Should return UNKNOWN for unclear utterances"""
        request = ParseIntent.Request()
        request.utterance = "xyz abc 123"
        
        response = ParseIntent.Response()
        intent_parser.parse_intent_callback(request, response)
        
        assert response.intent.type == "UNKNOWN"
        assert response.intent.confidence < 0.5
```

### Pattern 2: ROS Actions

```python
# tests/unit/ai/test_motion_planner.py

class TestMotionPlanner:
    """TDD for Motion Planner Action Server"""
    
    @pytest.fixture
    def motion_planner(self):
        rclpy.init()
        node = MotionPlannerNode()
        yield node
        node.destroy_node()
        rclpy.shutdown()
    
    def test_plan_motion_success(self, motion_planner):
        """RED: Should generate valid motion plan"""
        goal = PlanMotion.Goal()
        goal.primitive.type = "NAVIGATE"
        goal.primitive.target_pose = PoseStamped()
        goal.primitive.target_pose.pose.position.x = 1.0
        
        # Execute action
        result = motion_planner.execute_plan(goal)
        
        assert result.success is True
        assert len(result.plan.primitives) > 0
        assert result.plan.safety_certificate is not None
    
    def test_plan_motion_invalid_goal(self, motion_planner):
        """RED: Should fail for unreachable goal"""
        goal = PlanMotion.Goal()
        goal.primitive.target_pose = PoseStamped()
        goal.primitive.target_pose.pose.position.x = 999999.0  # Unreachable
        
        result = motion_planner.execute_plan(goal)
        
        assert result.success is False
        assert "unreachable" in result.error_message.lower()
```

### Pattern 3: Message Definitions

```python
# tests/unit/test_messages.py

class TestIntentMessage:
    """TDD for Intent.msg"""
    
    def test_intent_message_serialization(self):
        """RED: Intent message should serialize/deserialize correctly"""
        intent = Intent()
        intent.type = "NAVIGATE"
        intent.confidence = 0.95
        
        entity = Entity()
        entity.type = "LOCATION"
        entity.value = "kitchen"
        entity.confidence = 0.98
        intent.entities.append(entity)
        
        # Serialize and deserialize
        serialized = serialize(intent)
        deserialized = deserialize(serialized, Intent)
        
        assert deserialized.type == "NAVIGATE"
        assert deserialized.confidence == pytest.approx(0.95)
        assert len(deserialized.entities) == 1
        assert deserialized.entities[0].value == "kitchen"
```

---

## Test Categories for v0.6.1

### Unit Tests (200+ tests)

```python
# tests/unit/safety/test_validator.py
# tests/unit/ai/test_intent_parser.py
# tests/unit/ai/test_context_manager.py
# tests/unit/ai/test_motion_planner.py
# tests/unit/ai/test_execution_monitor.py

def test_unit_example():
    """Fast, isolated, deterministic"""
    pass
```

### Integration Tests (50+ tests)

```python
# tests/integration/test_intent_to_planning.py
# tests/integration/test_safety_integration.py
# tests/integration/test_end_to_end.py

def test_integration_example():
    """Test component interactions"""
    # Intent parser → Context manager → Motion planner
    pass
```

### Simulation Tests (1000+ scenarios)

```python
# tests/simulation/test_navigation.py
# tests/simulation/test_manipulation.py
# tests/simulation/test_safety_scenarios.py

def test_simulation_example():
    """Test in Gazebo simulation"""
    # Launch simulation
    # Execute scenario
    # Validate behavior
    pass
```

---

## TDD Checklist for Engineers

### Before Writing Code

- [ ] Test file exists with failing tests
- [ ] Test names describe behavior (not implementation)
- [ ] Tests cover happy path and edge cases
- [ ] Tests are independent (no shared state)

### While Writing Code

- [ ] Write minimal code to pass tests
- [ ] No code without corresponding test
- [ ] Run tests frequently (every few minutes)
- [ ] Keep tests passing

### After Tests Pass

- [ ] Refactor to improve design
- [ ] Check code coverage (>90% for new code)
- [ ] Add documentation
- [ ] Run full test suite

### Before Commit

- [ ] All tests passing
- [ ] Code coverage >90%
- [ ] No linting errors
- [ ] Documentation updated

---

## Test Naming Conventions

### Good Names

```python
def test_validator_rejects_velocity_above_limit():
    """Describes behavior, not implementation"""
    pass

def test_intent_parser_returns_navigate_for_go_to_location():
    """Clear input/output relationship"""
    pass

def test_motion_planner_generates_certificate_for_valid_plan():
    """Expected outcome is explicit"""
    pass
```

### Bad Names

```python
def test_validator():
    """Too vague"""
    pass

def test_parse():
    """Doesn't describe what or outcome"""
    pass

def test_motion_planner_1():
    """Numbered tests are unclear"""
    pass
```

---

## Running Tests

### During Development

```bash
# Run specific test file (fast feedback)
pytest tests/unit/safety/test_validator.py -v

# Run specific test
pytest tests/unit/safety/test_validator.py::TestSafetyValidator::test_validator_rejects_excessive_velocity -v

# Run with coverage
pytest tests/unit/safety/test_validator.py --cov=agent_ros_bridge.safety --cov-report=term-missing
```

### Before Commit

```bash
# Run all unit tests
pytest tests/unit/ -v --cov=agent_ros_bridge --cov-report=html

# Run integration tests
pytest tests/integration/ -v

# Run simulation tests (if environment ready)
pytest tests/simulation/ -v --simulation

# Full test suite
pytest tests/ -v --cov=agent_ros_bridge --cov-report=xml
```

---

## Common TDD Mistakes

### 1. Writing Tests After Code

❌ **Wrong:**
```python
# Wrote implementation first
def my_function():
    return 42

# Now writing test (backwards!)
def test_my_function():
    assert my_function() == 42
```

✅ **Right:**
```python
# Test first (fails)
def test_my_function():
    assert my_function() == 42

# Then implementation
# def my_function(): ...
```

### 2. Testing Implementation Details

❌ **Wrong:**
```python
def test_uses_internal_cache():
    """Tests implementation, not behavior"""
    parser = IntentParser()
    parser.parse("go to kitchen")
    assert parser._cache["go to kitchen"] is not None  # Internal detail!
```

✅ **Right:**
```python
def test_returns_cached_result_for_same_input():
    """Tests behavior (caching is optimization)"""
    parser = IntentParser()
    result1 = parser.parse("go to kitchen")
    result2 = parser.parse("go to kitchen")
    assert result1 == result2
    # Implementation can change, behavior stays same
```

### 3. Tests Too Complex

❌ **Wrong:**
```python
def test_everything():
    """Tests too many things at once"""
    system = ComplexSystem()
    system.setup()
    system.configure()
    result = system.run()
    assert result.success
    assert result.data is not None
    assert result.timestamp > 0
    assert system.state == "COMPLETE"
    # 50 more assertions...
```

✅ **Right:**
```python
def test_system_returns_success_on_valid_input():
    """One behavior per test"""
    pass

def test_system_populates_data_field():
    """Separate test for data"""
    pass

def test_system_updates_state_on_completion():
    """Separate test for state"""
    pass
```

---

## Week 1 TDD Tasks

### ENG-1 (Agent AI)

```python
# tests/unit/ai/test_intent_parser_interface.py
# Tests before implementation!

def test_parse_intent_service_exists():
    """RED: ParseIntent service should be available"""
    pass

def test_intent_message_has_required_fields():
    """RED: Intent.msg should have type, confidence, entities"""
    pass

def test_entity_message_has_required_fields():
    """RED: Entity.msg should have type, value, confidence"""
    pass
```

### ENG-2 (ROS Safety)

```python
# tests/unit/safety/test_safety_messages.py

def test_safety_certificate_has_30s_validity():
    """RED: Certificate expires after 30 seconds"""
    pass

def test_trajectory_validation_rejects_unsafe():
    """RED: Unsafe trajectories rejected"""
    pass

def test_emergency_stop_service_available():
    """RED: Emergency stop service exists"""
    pass
```

### ENG-3 (Motion Planning)

```python
# tests/unit/ai/test_motion_messages.py

def test_motion_plan_includes_safety_certificate():
    """RED: All plans must have safety certificate"""
    pass

def test_plan_motion_action_accepts_primitive():
    """RED: PlanMotion action accepts MotionPrimitive"""
    pass
```

### ENG-4 (Simulation)

```python
# tests/simulation/test_environment.py

def test_turtlebot_model_spawns():
    """RED: TurtleBot3 model loads in Gazebo"""
    pass

def test_warehouse_world_loads():
    """RED: Warehouse world loads without errors"""
    pass
```

---

## Summary

**TDD Rules for v0.6.1:**

1. **No production code without failing test first**
2. **Write minimal code to pass tests**
3. **Refactor with tests green**
4. **Tests are documentation - make them readable**
5. **Coverage >90% for new code**
6. **Run tests continuously during development**

**Remember:** Tests are your safety net. They enable confident refactoring and prove your code works.

---

**Document Version:** 1.0  
**Last Updated:** 2026-03-06  
**Status:** Active TDD Guide
