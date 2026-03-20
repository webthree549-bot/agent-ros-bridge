"""Tests for OpenClaw adapter."""

from unittest.mock import AsyncMock, MagicMock, Mock, patch
from pathlib import Path

import pytest

from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter, OpenClawTool


class TestOpenClawTool:
    """Test OpenClawTool dataclass."""

    def test_tool_creation(self):
        """Test creating a tool."""
        tool = OpenClawTool(
            name="test_tool",
            description="A test tool",
            parameters={"arg1": {"type": "string"}},
        )
        assert tool.name == "test_tool"
        assert tool.dangerous is False

    def test_tool_to_openclaw_format(self):
        """Test conversion to OpenClaw format."""
        tool = OpenClawTool(
            name="test_tool",
            description="A test tool",
            parameters={"arg1": {"type": "string"}},
        )
        result = tool.to_openclaw_format()
        assert result["name"] == "test_tool"
        assert result["dangerous"] is False
        assert "parameters" in result


class TestOpenClawAdapterInitialization:
    """Test adapter initialization."""

    def test_init_default(self):
        """Test default initialization."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        assert adapter.bridge is mock_bridge
        assert adapter.include_ros1 is False
        assert len(adapter._tools) > 0

    def test_init_with_ros1(self):
        """Test initialization with ROS1 support."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge, include_ros1=True)
        assert adapter.include_ros1 is True
        # Should have ROS1 tools registered
        assert "ros1_publish" in adapter._tools

    def test_tools_registered(self):
        """Test that default tools are registered."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        assert "ros2_publish" in adapter._tools
        assert "ros2_subscribe_once" in adapter._tools
        assert "ros2_list_topics" in adapter._tools
        assert "bridge_list_robots" in adapter._tools
        assert "safety_trigger_estop" in adapter._tools


class TestGetSkillPath:
    """Test getting skill path."""

    def test_get_skill_path(self):
        """Test getting skill path."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        # Skill path may be None if not found
        path = adapter.get_skill_path()
        # Just verify it doesn't crash
        assert path is None or isinstance(path, Path)


class TestGetTools:
    """Test getting tools."""

    def test_get_tools(self):
        """Test getting all tools."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        tools = adapter.get_tools()
        assert isinstance(tools, list)
        assert len(tools) > 0
        # Check format
        for tool in tools:
            assert "name" in tool
            assert "description" in tool
            assert "parameters" in tool

    def test_get_tool_by_name(self):
        """Test getting specific tool."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        tool = adapter.get_tool("ros2_publish")
        assert tool is not None
        assert tool.name == "ros2_publish"

    def test_get_tool_not_found(self):
        """Test getting non-existent tool."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        tool = adapter.get_tool("nonexistent")
        assert tool is None


class TestExecuteTool:
    """Test tool execution."""

    @pytest.mark.asyncio
    async def test_execute_tool_no_bridge(self):
        """Test execution with no bridge."""
        adapter = OpenClawAdapter(None)
        result = await adapter.execute_tool("ros2_publish", {"topic": "/test"})
        assert result["success"] is False
        assert "Bridge not available" in result["error"]

    @pytest.mark.asyncio
    async def test_execute_tool_unknown(self):
        """Test execution of unknown tool."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        result = await adapter.execute_tool("unknown_tool", {})
        assert result["success"] is False
        assert "Unknown tool" in result["error"]

    @pytest.mark.asyncio
    async def test_execute_ros2_publish_missing_params(self):
        """Test ros2_publish with missing params."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        result = await adapter.execute_tool("ros2_publish", {})
        assert result["success"] is False
        assert "Missing required" in result["error"]

    @pytest.mark.asyncio
    async def test_execute_ros2_list_topics_no_connector(self):
        """Test ros2_list_topics with no connector."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        result = await adapter.execute_tool("ros2_list_topics", {})
        assert result["success"] is False
        assert "ROS2 connector not available" in result["error"]

    @pytest.mark.asyncio
    async def test_execute_bridge_list_robots_no_fleets(self):
        """Test bridge_list_robots with no fleet support."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        result = await adapter.execute_tool("bridge_list_robots", {})
        assert result["success"] is False

    @pytest.mark.asyncio
    async def test_execute_safety_trigger_estop_missing_robot(self):
        """Test safety_trigger_estop with missing robot_id."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        result = await adapter.execute_tool("safety_trigger_estop", {})
        assert result["success"] is False
        assert "robot_id" in result["error"]

    @pytest.mark.asyncio
    async def test_execute_memory_set_missing_params(self):
        """Test memory_set with missing params."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        result = await adapter.execute_tool("memory_set", {})
        assert result["success"] is False
        assert "Missing required" in result["error"]


class TestToRosclawCompatibleFormat:
    """Test RosClaw compatibility."""

    def test_rosclaw_format(self):
        """Test conversion to RosClaw format."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        tools = adapter.to_rosclaw_compatible_format()
        assert isinstance(tools, list)
        for tool in tools:
            assert "type" in tool
            assert "function" in tool
            assert tool["type"] == "function"


class TestNaturalLanguageSupport:
    """Test natural language support."""

    def test_enable_natural_language(self):
        """Test enabling NL support."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        adapter.enable_natural_language()
        assert hasattr(adapter, "nl_interpreter")
        assert hasattr(adapter, "context_manager")
        assert hasattr(adapter, "contextual_interpreter")

    @pytest.mark.asyncio
    async def test_execute_nl_no_interpreter(self):
        """Test NL execution without interpreter."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        # Should auto-enable interpreter
        with patch.object(adapter, "execute_tool", return_value={"success": True}):
            result = await adapter.execute_nl("move forward")
            # Should have enabled NL support
            assert hasattr(adapter, "contextual_interpreter")

    def test_learn_location(self):
        """Test learning location."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        adapter.learn_location("session1", "kitchen", {"x": 10, "y": 5})
        assert hasattr(adapter, "context_manager")

    def test_get_conversation_history(self):
        """Test getting conversation history."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        history = adapter.get_conversation_history("session1", n=5)
        assert isinstance(history, list)


class TestPackageSkill:
    """Test skill packaging."""

    def test_package_skill_no_path(self):
        """Test packaging with no skill path."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        adapter._skill_path = None
        with pytest.raises(RuntimeError, match="Skill path not found"):
            adapter.package_skill(Path("/tmp"))
