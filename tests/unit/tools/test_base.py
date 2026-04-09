"""Tests for tools base classes.

TDD Approach:
1. Test base tool interface
2. Test ToolResult dataclass
3. Test validation methods
"""

import pytest

from agent_ros_bridge.tools.base import ROSTool, ToolResult


class TestToolResult:
    """Test ToolResult dataclass."""

    def test_default_creation(self):
        """Test creating ToolResult with defaults."""
        result = ToolResult(success=True, output="test output")

        assert result.success is True
        assert result.output == "test output"
        assert result.error is None
        assert result.data == {}
        assert result.execution_time_ms == 0.0

    def test_full_creation(self):
        """Test creating ToolResult with all fields."""
        result = ToolResult(
            success=False,
            output="",
            error="Test error",
            data={"key": "value"},
            execution_time_ms=123.45,
        )

        assert result.success is False
        assert result.output == ""
        assert result.error == "Test error"
        assert result.data == {"key": "value"}
        assert result.execution_time_ms == 123.45

    def test_success_result(self):
        """Test successful result."""
        result = ToolResult(
            success=True,
            output="Command executed successfully",
            data={"result": "data"},
            execution_time_ms=50.0,
        )

        assert result.success is True
        assert result.error is None

    def test_failure_result(self):
        """Test failed result."""
        result = ToolResult(
            success=False,
            output="",
            error="Command failed: timeout",
            execution_time_ms=5000.0,
        )

        assert result.success is False
        assert result.error is not None


class ConcreteTool(ROSTool):
    """Concrete implementation for testing."""

    name = "test_tool"
    description = "A test tool"
    version = "1.0.0"

    def execute(self, param: str = "default", **kwargs) -> ToolResult:
        """Execute test tool."""
        if param == "fail":
            return ToolResult(
                success=False,
                output="",
                error="Intentional failure",
            )

        return ToolResult(
            success=True,
            output=f"Result: {param}",
            data={"param": param},
        )


class TestROSTool:
    """Test ROSTool base class."""

    def test_tool_attributes(self):
        """Test tool has required attributes."""
        tool = ConcreteTool()

        assert tool.name == "test_tool"
        assert tool.description == "A test tool"
        assert tool.version == "1.0.0"

    def test_execute_success(self):
        """Test successful execution."""
        tool = ConcreteTool()
        result = tool.execute(param="test_value")

        assert result.success is True
        assert result.output == "Result: test_value"
        assert result.data["param"] == "test_value"

    def test_execute_failure(self):
        """Test failed execution."""
        tool = ConcreteTool()
        result = tool.execute(param="fail")

        assert result.success is False
        assert result.error == "Intentional failure"

    def test_validate_params_default(self):
        """Test default parameter validation."""
        tool = ConcreteTool()
        valid, error = tool.validate_params({})

        assert valid is True
        assert error == ""

    def test_get_schema(self):
        """Test schema generation."""
        tool = ConcreteTool()
        schema = tool.get_schema()

        assert schema["name"] == "test_tool"
        assert schema["description"] == "A test tool"
        assert schema["version"] == "1.0.0"


class TestToolInheritance:
    """Test that tools properly inherit from base."""

    def test_abstract_execute(self):
        """Test that execute is abstract."""

        class IncompleteTool(ROSTool):
            name = "incomplete"
            description = "Incomplete tool"

        with pytest.raises(TypeError):
            IncompleteTool()

    def test_concrete_tool_instantiation(self):
        """Test that concrete tools can be instantiated."""
        tool = ConcreteTool()
        assert tool is not None
