"""Tests for Agent ROS Bridge Tools."""

import pytest
from agent_ros_bridge_tools import (
    ROSTopicEchoTool,
    ROSServiceCallTool,
    ROSNodeListTool,
    ROSParamGetTool,
    ROSBagPlayTool,
    Tool,
    ToolResult,
    register_tool,
    get_tool,
    list_tools,
)


class TestToolRegistration:
    """Test tool registration system."""

    def test_list_tools(self):
        """Test listing registered tools."""
        tools = list_tools()
        assert "rostopic_echo" in tools
        assert "rosservice_call" in tools
        assert "rosnode_list" in tools
        assert "rosparam_get" in tools
        assert "rosbag_play" in tools

    def test_get_tool(self):
        """Test getting a tool by name."""
        tool_class = get_tool("rostopic_echo")
        assert tool_class is not None
        assert tool_class.name == "rostopic_echo"

    def test_get_nonexistent_tool(self):
        """Test getting a non-existent tool."""
        tool_class = get_tool("nonexistent")
        assert tool_class is None

    def test_register_custom_tool(self):
        """Test registering a custom tool."""

        @register_tool
        class TestTool(Tool):
            name = "test_tool"
            description = "A test tool"

            async def execute(self, value: str) -> ToolResult:
                return ToolResult.success_result(data={"value": value})

        assert "test_tool" in list_tools()
        tool_class = get_tool("test_tool")
        assert tool_class is not None


class TestToolResult:
    """Test ToolResult dataclass."""

    def test_success_result(self):
        """Test creating a success result."""
        result = ToolResult.success_result(data={"key": "value"})
        assert result.success is True
        assert result.data == {"key": "value"}
        assert result.error_message is None

    def test_error_result(self):
        """Test creating an error result."""
        result = ToolResult.error_result("Something went wrong")
        assert result.success is False
        assert result.error_message == "Something went wrong"
        assert result.data is None


class TestROSTopicEchoTool:
    """Test ROSTopicEchoTool."""

    def test_tool_properties(self):
        """Test tool properties."""
        tool = ROSTopicEchoTool()
        assert tool.name == "rostopic_echo"
        assert tool.description == "Echo messages from a ROS topic"
        assert tool.version == "1.0.0"

    def test_get_schema(self):
        """Test getting tool schema."""
        tool = ROSTopicEchoTool()
        schema = tool.get_schema()
        assert schema["name"] == "rostopic_echo"
        assert "parameters" in schema
        assert "topic_name" in schema["parameters"]


class TestROSServiceCallTool:
    """Test ROSServiceCallTool."""

    def test_tool_properties(self):
        """Test tool properties."""
        tool = ROSServiceCallTool()
        assert tool.name == "rosservice_call"
        assert tool.description == "Call a ROS service"


class TestROSNodeListTool:
    """Test ROSNodeListTool."""

    def test_tool_properties(self):
        """Test tool properties."""
        tool = ROSNodeListTool()
        assert tool.name == "rosnode_list"
        assert tool.description == "List running ROS nodes"


class TestROSParamGetTool:
    """Test ROSParamGetTool."""

    def test_tool_properties(self):
        """Test tool properties."""
        tool = ROSParamGetTool()
        assert tool.name == "rosparam_get"
        assert tool.description == "Get ROS parameters"


class TestROSBagPlayTool:
    """Test ROSBagPlayTool."""

    def test_tool_properties(self):
        """Test tool properties."""
        tool = ROSBagPlayTool()
        assert tool.name == "rosbag_play"
        assert tool.description == "Play recorded ROS bag files"

    def test_nonexistent_bag_file(self):
        """Test handling of non-existent bag file."""
        import asyncio

        async def run_test():
            tool = ROSBagPlayTool()
            result = await tool.execute("/nonexistent/path.bag")
            assert result.success is False
            assert "not found" in result.error_message

        asyncio.run(run_test())


class TestToolBase:
    """Test Tool base class."""

    def test_abstract_execute(self):
        """Test that Tool is abstract."""

        class ConcreteTool(Tool):
            name = "concrete"
            description = "A concrete tool"

            async def execute(self) -> ToolResult:
                return ToolResult.success_result()

        tool = ConcreteTool()
        assert tool.name == "concrete"
        assert tool.config == {}

    def test_tool_with_config(self):
        """Test tool with configuration."""

        class ConfiguredTool(Tool):
            name = "configured"
            description = "A configured tool"

            async def execute(self) -> ToolResult:
                return ToolResult.success_result(data=self.config)

        tool = ConfiguredTool(config={"key": "value"})
        assert tool.config == {"key": "value"}
