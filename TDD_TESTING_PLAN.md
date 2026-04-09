# TDD Testing & Validation Plan

**Date:** April 8, 2026  
**Phase:** Priority 1 - Testing & Validation  
**Approach:** Test-Driven Development

---

## ✅ Step 1: Current State Assessment

### Test Suite Status
```
New Tests (tools + exceptions): 42/42 ✅ PASSING
Total Tests Collected: 2,653 tests
Status: Running full suite...
```

### Coverage Analysis Needed
- [ ] Identify untested modules
- [ ] Find coverage gaps
- [ ] Prioritize high-risk areas

---

## 🎯 Step 2: TDD - Write Tests First

### Gap 1: ROSServiceCallTool Tests (Missing)

**Write Test First:**
```python
# tests/unit/tools/test_rosservice_call.py
class TestROSServiceCallTool:
    """TDD tests for ROSServiceCallTool."""
    
    def test_tool_attributes(self):
        """Test tool has correct metadata."""
        tool = ROSServiceCallTool()
        assert tool.name == "rosservice_call"
        assert "ROS service" in tool.description
    
    def test_execute_without_ros(self):
        """Test graceful fallback when ROS unavailable."""
        tool = ROSServiceCallTool()
        with patch.dict("sys.modules", {"rclpy": None}):
            result = tool.execute(service="/test")
            assert not result.success
            assert "ROS2 not available" in result.error
```

### Gap 2: Action Client Tests (Need More Coverage)

**Write Test First:**
```python
# tests/unit/actions/test_factory.py
class TestActionFactory:
    """TDD tests for action client factory."""
    
    def test_create_ros2_client(self):
        """Test factory creates ROS2 client."""
        client = create_action_client("ros2", "/test", "TestAction")
        assert isinstance(client, ROS2ActionClient)
```

### Gap 3: Integration Tests for AI Agent Examples

**Write Test First:**
```python
# tests/integration/test_openclaw_bridge.py
class TestOpenClawIntegration:
    """TDD tests for OpenClaw bridge integration."""
    
    def test_nl_interpretation(self):
        """Test natural language interpretation."""
        bridge = OpenClawROSBridge()
        result = bridge.process_natural_language("Go to kitchen")
        assert result["action"] == "navigate"
```

---

## 🔧 Step 3: Run Tests (Red Phase)

**Expected:** Tests fail (code doesn't exist yet or needs fixing)

```bash
pytest tests/unit/tools/test_rosservice_call.py -v
# Expected: FAIL (tests written, implementation incomplete)
```

---

## 💻 Step 4: Implement Code (Green Phase)

### Implement ROSServiceCallTool
```python
# agent_ros_bridge/tools/rosservice_call.py
class ROSServiceCallTool(ROSTool):
    """Tool for calling ROS services."""
    
    name = "rosservice_call"
    description = "Call a ROS service"
    
    def execute(self, service: str, **kwargs) -> ToolResult:
        """Execute service call."""
        if not self._has_ros2():
            return ToolResult(
                success=False,
                error="ROS2 not available"
            )
        # Implementation...
```

---

## ✅ Step 5: Verify Tests Pass

```bash
pytest tests/unit/tools/test_rosservice_call.py -v
# Expected: PASS
```

---

## 📊 Test Coverage Goals

| Module | Current | Target | Gap |
|--------|---------|--------|-----|
| tools/base.py | 100% | 100% | ✅ Done |
| tools/rostopic_echo.py | 40% | 80% | ⚠️ Add tests |
| tools/rosservice_call.py | 0% | 80% | ❌ Write tests |
| actions/factory.py | ? | 80% | ❌ Check & add |
| exceptions.py | 100% | 100% | ✅ Done |

---

## 🎯 Action Items

### Immediate (Today)
1. ✅ New tests passing: 42/42
2. 🔄 Run full test suite
3. 📝 Generate coverage report
4. 🎯 Identify top 3 gaps

### This Week
1. Write tests for ROSServiceCallTool
2. Write tests for action factory
3. Add integration tests
4. Reach 70% overall coverage

---

## 🏆 Success Criteria

- [ ] All new tests pass
- [ ] Coverage maintained or improved
- [ ] No regressions
- [ ] TDD cycle documented

**Status:** In Progress 🔄
