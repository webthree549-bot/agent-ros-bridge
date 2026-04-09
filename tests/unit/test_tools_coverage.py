"""Tools module coverage tests - comprehensive tool testing."""

import pytest
from unittest.mock import Mock, patch, MagicMock, AsyncMock
import json
import asyncio


class TestROSServiceCallTool:
    """Test ROS service call tool comprehensively."""

    def test_tool_initialization(self):
        """Test tool initialization with service name."""
        from agent_ros_bridge.tools.ros_service import ROSServiceCallTool

        tool = ROSServiceCallTool(
            service_name="/example_service",
            service_type="std_srvs/SetBool",
        )

        assert tool.service_name == "/example_service"
        assert tool.service_type == "std_srvs/SetBool"
        assert tool.description is not None

    def test_tool_properties(self):
        """Test tool name and description properties."""
        from agent_ros_bridge.tools.ros_service import ROSServiceCallTool

        tool = ROSServiceCallTool(
            service_name="/reset_world",
            service_type="std_srvs/Empty",
        )

        assert tool.name == "ros_service_reset_world"
        assert "ROS" in tool.description
        assert "reset_world" in tool.description

    def test_get_parameters_schema(self):
        """Test parameter schema generation."""
        from agent_ros_bridge.tools.ros_service import ROSServiceCallTool

        tool = ROSServiceCallTool(
            service_name="/set_bool",
            service_type="std_srvs/SetBool",
        )

        schema = tool.get_parameters_schema()

        assert "type" in schema
        assert schema["type"] == "object"
        assert "properties" in schema

    def test_execute_with_mock_service(self):
        """Test execution with mocked ROS service."""
        from agent_ros_bridge.tools.ros_service import ROSServiceCallTool

        mock_node = Mock()
        mock_client = Mock()
        mock_node.create_client.return_value = mock_client

        tool = ROSServiceCallTool(
            service_name="/test_service",
            service_type="std_srvs/SetBool",
            node=mock_node,
        )

        # Mock the service response
        mock_response = Mock()
        mock_response.success = True
        mock_response.message = "Success"

        mock_client.call.return_value = mock_response

        result = tool.execute(data={"data": True})

        assert result["success"] is True

    def test_execute_service_not_available(self):
        """Test execution when service is not available."""
        from agent_ros_bridge.tools.ros_service import ROSServiceCallTool

        mock_node = Mock()
        mock_client = Mock()
        mock_client.wait_for_service.return_value = False
        mock_node.create_client.return_value = mock_client

        tool = ROSServiceCallTool(
            service_name="/unavailable_service",
            service_type="std_srvs/Empty",
            node=mock_node,
        )

        result = tool.execute(data={})

        assert result["success"] is False
        assert "not available" in result.get("error", "").lower()


class TestROSPublisherTool:
    """Test ROS publisher tool."""

    def test_publisher_initialization(self):
        """Test publisher tool initialization."""
        from agent_ros_bridge.tools.ros_publisher import ROSPublisherTool

        tool = ROSPublisherTool(
            topic_name="/cmd_vel",
            message_type="geometry_msgs/Twist",
        )

        assert tool.topic_name == "/cmd_vel"
        assert tool.message_type == "geometry_msgs/Twist"

    def test_publish_message(self):
        """Test publishing a message."""
        from agent_ros_bridge.tools.ros_publisher import ROSPublisherTool

        mock_node = Mock()
        mock_publisher = Mock()
        mock_node.create_publisher.return_value = mock_publisher

        tool = ROSPublisherTool(
            topic_name="/test_topic",
            message_type="std_msgs/String",
            node=mock_node,
        )

        result = tool.execute(data={"data": "Hello"})

        assert result["success"] is True
        mock_publisher.publish.assert_called_once()


class TestROSSubscriberTool:
    """Test ROS subscriber tool."""

    def test_subscriber_initialization(self):
        """Test subscriber tool initialization."""
        from agent_ros_bridge.tools.ros_subscriber import ROSSubscriberTool

        tool = ROSSubscriberTool(
            topic_name="/odom",
            message_type="nav_msgs/Odometry",
        )

        assert tool.topic_name == "/odom"
        assert tool.message_type == "nav_msgs/Odometry"

    def test_get_latest_message(self):
        """Test getting latest message."""
        from agent_ros_bridge.tools.ros_subscriber import ROSSubscriberTool

        mock_node = Mock()
        tool = ROSSubscriberTool(
            topic_name="/test",
            message_type="std_msgs/String",
            node=mock_node,
        )

        # Set a cached message
        tool._latest_message = {"data": "test_message"}

        result = tool.execute(data={})

        assert result["success"] is True
        # The data field contains the full message dict
        assert result["data"] == {"data": "test_message"}


class TestToolRegistry:
    """Test tool registry."""

    def test_registry_initialization(self):
        """Test registry initialization."""
        from agent_ros_bridge.tools.registry import ToolRegistry

        registry = ToolRegistry()
        assert registry is not None
        assert len(registry.get_all_tools()) == 0

    def test_register_tool(self):
        """Test registering a tool."""
        from agent_ros_bridge.tools.registry import ToolRegistry
        from agent_ros_bridge.tools.base import BaseTool

        registry = ToolRegistry()
        mock_tool = Mock(spec=BaseTool)
        mock_tool.name = "test_tool"

        registry.register(mock_tool)

        assert len(registry.get_all_tools()) == 1
        assert registry.get_tool("test_tool") == mock_tool

    def test_unregister_tool(self):
        """Test unregistering a tool."""
        from agent_ros_bridge.tools.registry import ToolRegistry
        from agent_ros_bridge.tools.base import BaseTool

        registry = ToolRegistry()
        mock_tool = Mock(spec=BaseTool)
        mock_tool.name = "test_tool"

        registry.register(mock_tool)
        registry.unregister("test_tool")

        assert len(registry.get_all_tools()) == 0
        assert registry.get_tool("test_tool") is None


class TestBaseTool:
    """Test base tool class."""

    def test_concrete_tool(self):
        """Test creating a concrete tool."""
        from agent_ros_bridge.tools.base import BaseTool

        class TestTool(BaseTool):
            @property
            def name(self):
                return "test_tool"

            @property
            def description(self):
                return "A test tool"

            def get_parameters_schema(self):
                return {"type": "object", "properties": {}}

            def execute(self, **kwargs):
                return {"success": True}

        tool = TestTool()
        assert tool.name == "test_tool"
        assert tool.description == "A test tool"

        result = tool.execute()
        assert result["success"] is True


class TestToolUtils:
    """Test tool utility functions."""

    def test_sanitize_tool_name(self):
        """Test tool name sanitization."""
        from agent_ros_bridge.tools.utils import sanitize_tool_name

        assert sanitize_tool_name("Test Tool") == "test_tool"
        assert sanitize_tool_name("/ros/service") == "ros_service"
        assert sanitize_tool_name("tool-123") == "tool_123"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
