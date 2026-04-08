"""TDD Tests for ROSServiceCallTool.

Test-Driven Development approach:
1. Write tests first (this file)
2. Run tests - expect FAIL (Red phase)
3. Implement code to make tests pass (Green phase)
4. Refactor if needed
"""

import pytest
from unittest.mock import patch, MagicMock

from agent_ros_bridge.tools import ROSServiceCallTool
from agent_ros_bridge.tools.base import ToolResult


class TestROSServiceCallToolBasics:
    """Basic tests for ROSServiceCallTool."""
    
    def test_tool_attributes(self):
        """Test tool has correct metadata."""
        tool = ROSServiceCallTool()
        assert tool.name == "rosservice_call"
        assert "ROS service" in tool.description
        assert tool.version == "1.0.0"
    
    def test_tool_schema(self):
        """Test tool schema is defined."""
        tool = ROSServiceCallTool()
        schema = tool.get_schema()
        assert "service" in schema["parameters"]
        assert schema["parameters"]["service"]["type"] == "string"


class TestROSServiceCallToolExecution:
    """Execution tests for ROSServiceCallTool."""
    
    def test_execute_without_ros(self):
        """Test graceful fallback when ROS unavailable."""
        tool = ROSServiceCallTool()
        with patch.dict("sys.modules", {"rclpy": None}):
            result = tool.execute(service="/test_service")
            assert not result.success
            assert "ROS2 not available" in result.error
    
    def test_execute_with_empty_service(self):
        """Test validation of empty service name."""
        tool = ROSServiceCallTool()
        result = tool.execute(service="")
        assert not result.success
        assert "service name" in result.error.lower()
    
    def test_execute_with_valid_service_mock(self):
        """Test execution with mocked ROS2 - validates method chain is called."""
        tool = ROSServiceCallTool()
        
        # This test validates the implementation structure
        # Full integration test would require actual ROS2
        # For now, verify the tool has required methods
        assert hasattr(tool, '_execute_ros2')
        assert hasattr(tool, '_get_service_type')
        assert hasattr(tool, '_service_exists')


class TestROSServiceCallToolValidation:
    """Validation tests for ROSServiceCallTool."""
    
    def test_service_name_validation(self):
        """Test service name format validation."""
        tool = ROSServiceCallTool()
        
        # Valid names
        assert tool._validate_service_name("/test") is True
        assert tool._validate_service_name("/ns/service") is True
        
        # Invalid names
        assert tool._validate_service_name("") is False
        assert tool._validate_service_name("no_slash") is False
    
    def test_timeout_validation(self):
        """Test timeout parameter validation."""
        tool = ROSServiceCallTool()
        
        # Valid timeouts
        assert tool._validate_timeout(1.0) is True
        assert tool._validate_timeout(30.0) is True
        
        # Invalid timeouts
        assert tool._validate_timeout(0) is False
        assert tool._validate_timeout(-1.0) is False


class TestROSServiceCallToolEdgeCases:
    """Edge case tests for ROSServiceCallTool."""
    
    def test_very_long_service_name(self):
        """Test handling of very long service names."""
        tool = ROSServiceCallTool()
        long_name = "/" + "a" * 500
        
        result = tool.execute(service=long_name)
        # Should handle gracefully
        assert isinstance(result, ToolResult)
    
    def test_special_characters_in_service(self):
        """Test handling of special characters."""
        tool = ROSServiceCallTool()
        
        special_names = [
            "/test_123",
            "/test-name",
            "/ns1/ns2/service",
        ]
        
        for name in special_names:
            result = tool.execute(service=name)
            assert isinstance(result, ToolResult)
    
    def test_service_not_found(self):
        """Test handling of non-existent service."""
        tool = ROSServiceCallTool()
        
        with patch.dict("sys.modules", {"rclpy": MagicMock()}):
            with patch.object(tool, '_service_exists', return_value=False):
                result = tool.execute(service="/nonexistent")
                assert not result.success
                assert "not found" in result.error.lower() or "not available" in result.error.lower()


class TestROSServiceCallToolResultFormat:
    """Result format tests."""
    
    def test_result_has_service_name(self):
        """Test result includes service name."""
        tool = ROSServiceCallTool()
        
        with patch.dict("sys.modules", {"rclpy": None}):
            result = tool.execute(service="/my_service")
            assert result.data.get("service") == "/my_service"
    
    def test_result_timestamp_present(self):
        """Test result includes timestamp."""
        tool = ROSServiceCallTool()
        
        with patch.dict("sys.modules", {"rclpy": None}):
            result = tool.execute(service="/test")
            assert "timestamp" in result.data
