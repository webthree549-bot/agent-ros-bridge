"""Tests for custom exceptions.

TDD Approach:
1. Test exception hierarchy
2. Test exception attributes
3. Test error messages
4. Test exception chaining
"""

import pytest

from agent_ros_bridge.exceptions import (
    AgentROSBridgeError,
    ConfigurationError,
    RobotConnectionError,
    SafetyValidationError,
    ToolExecutionError,
    TransportError,
    ValidationError,
)


class TestAgentROSBridgeError:
    """Test base exception."""

    def test_basic_exception(self):
        """Test basic exception creation."""
        exc = AgentROSBridgeError("Test error")
        
        assert str(exc) == "Test error"
        assert exc.details == {}

    def test_exception_with_details(self):
        """Test exception with details."""
        details = {"key": "value", "number": 42}
        exc = AgentROSBridgeError("Test error", details)
        
        assert exc.details == details
        assert exc.details["key"] == "value"

    def test_exception_inheritance(self):
        """Test that all exceptions inherit from base."""
        exceptions = [
            RobotConnectionError("test"),
            SafetyValidationError("test"),
            TransportError("test"),
            ToolExecutionError("test"),
            ConfigurationError("test"),
            ValidationError("test"),
        ]
        
        for exc in exceptions:
            assert isinstance(exc, AgentROSBridgeError)


class TestRobotConnectionError:
    """Test RobotConnectionError."""

    def test_without_robot_id(self):
        """Test error without robot ID."""
        exc = RobotConnectionError("Connection failed")
        
        assert "Connection failed" in str(exc)
        assert "Robot connection failed:" in str(exc)
        assert exc.robot_id is None
        assert exc.details == {"robot_id": None}

    def test_with_robot_id(self):
        """Test error with robot ID."""
        exc = RobotConnectionError("Connection failed", robot_id="bot_01")
        
        assert "bot_01" in str(exc)
        assert exc.robot_id == "bot_01"
        assert exc.details["robot_id"] == "bot_01"

    def test_error_message_format(self):
        """Test error message formatting."""
        exc = RobotConnectionError("Timeout after 30s", robot_id="arm_01")
        
        expected = "Robot connection failed for arm_01: Timeout after 30s"
        assert str(exc) == expected


class TestSafetyValidationError:
    """Test SafetyValidationError."""

    def test_without_violation_type(self):
        """Test error without violation type."""
        exc = SafetyValidationError("Velocity limit exceeded")
        
        assert "Safety validation failed" in str(exc)
        assert "Velocity limit exceeded" in str(exc)
        assert exc.violation_type is None

    def test_with_violation_type(self):
        """Test error with violation type."""
        exc = SafetyValidationError("Too fast", violation_type="velocity")
        
        assert exc.violation_type == "velocity"
        assert exc.details["violation_type"] == "velocity"

    def test_various_violation_types(self):
        """Test different violation types."""
        violations = [
            ("velocity", "Speed too high"),
            ("workspace", "Out of bounds"),
            ("force", "Excessive force"),
            ("collision", "Collision detected"),
        ]
        
        for vtype, message in violations:
            exc = SafetyValidationError(message, violation_type=vtype)
            assert exc.violation_type == vtype


class TestToolExecutionError:
    """Test ToolExecutionError."""

    def test_without_tool_name(self):
        """Test error without tool name."""
        exc = ToolExecutionError("Tool crashed")
        
        assert "Tool execution failed" in str(exc)
        assert exc.tool_name is None

    def test_with_tool_name(self):
        """Test error with tool name."""
        exc = ToolExecutionError("Not found", tool_name="rostopic_echo")
        
        assert "rostopic_echo" in str(exc)
        assert exc.tool_name == "rostopic_echo"
        assert exc.details["tool_name"] == "rostopic_echo"


class TestOtherExceptions:
    """Test other exception types."""

    def test_transport_error(self):
        """Test TransportError."""
        exc = TransportError("Connection reset")
        
        assert "Connection reset" in str(exc)
        assert isinstance(exc, AgentROSBridgeError)

    def test_configuration_error(self):
        """Test ConfigurationError."""
        exc = ConfigurationError("Invalid config")
        
        assert "Invalid config" in str(exc)
        assert isinstance(exc, AgentROSBridgeError)

    def test_validation_error(self):
        """Test ValidationError."""
        exc = ValidationError("Invalid parameter")
        
        assert "Invalid parameter" in str(exc)
        assert isinstance(exc, AgentROSBridgeError)


class TestExceptionChaining:
    """Test exception chaining."""

    def test_cause_chain(self):
        """Test that exceptions can chain."""
        original = ValueError("Original error")
        
        try:
            raise RobotConnectionError("Failed to connect") from original
        except RobotConnectionError as e:
            assert e.__cause__ is original
            assert isinstance(e.__cause__, ValueError)

    def test_context_chain(self):
        """Test exception context."""
        try:
            try:
                raise ConnectionError("Network down")
            except ConnectionError:
                raise TransportError("Transport failed")
        except TransportError as e:
            assert e.__context__ is not None


class TestExceptionCatching:
    """Test exception catching behavior."""

    def test_catch_base_exception(self):
        """Test catching base catches all."""
        exceptions = [
            RobotConnectionError("test"),
            SafetyValidationError("test"),
            TransportError("test"),
        ]
        
        for exc in exceptions:
            try:
                raise exc
            except AgentROSBridgeError as e:
                assert str(e) is not None

    def test_catch_specific_exception(self):
        """Test catching specific exceptions."""
        try:
            raise RobotConnectionError("test", robot_id="bot1")
        except RobotConnectionError as e:
            assert e.robot_id == "bot1"
        except AgentROSBridgeError:
            pytest.fail("Should have caught specific exception first")


class TestExceptionAttributes:
    """Test exception attributes are preserved."""

    def test_all_exceptions_have_str(self):
        """Test all exceptions convert to string."""
        exceptions = [
            AgentROSBridgeError("base"),
            RobotConnectionError("robot"),
            SafetyValidationError("safety"),
            TransportError("transport"),
            ToolExecutionError("tool"),
            ConfigurationError("config"),
            ValidationError("validation"),
        ]
        
        for exc in exceptions:
            assert len(str(exc)) > 0
            assert isinstance(str(exc), str)

    def test_all_exceptions_have_details(self):
        """Test all exceptions have details dict."""
        exceptions = [
            AgentROSBridgeError("base"),
            RobotConnectionError("robot"),
            SafetyValidationError("safety"),
        ]
        
        for exc in exceptions:
            assert hasattr(exc, "details")
            assert isinstance(exc.details, dict)
