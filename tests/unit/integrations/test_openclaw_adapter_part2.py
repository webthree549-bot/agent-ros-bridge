"""Additional tests for OpenClaw adapter - Part 2."""

import tempfile
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, Mock, call, patch

import pytest

from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter, OpenClawTool


class TestOpenClawAdapterToolFormats:
    """Test tool format conversions."""

    def test_all_default_tools_exist(self):
        """Test that all default tools are registered."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        
        expected_tools = [
            "ros2_publish",
            "ros2_subscribe_once",
            "ros2_service_call",
            "ros2_action_goal",
            "ros2_param_get",
            "ros2_param_set",
            "ros2_list_topics",
            "ros2_list_services",
            "ros2_camera_snapshot",
            "bridge_list_robots",
            "bridge_get_robot_status",
            "fleet_submit_task",
            "fleet_get_metrics",
            "safety_trigger_estop",
            "safety_release_estop",
            "memory_set",
            "memory_get",
        ]
        
        for tool_name in expected_tools:
            assert tool_name in adapter._tools, f"Tool {tool_name} not found"

    def test_tool_dangerous_flags(self):
        """Test that dangerous tools are marked correctly."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        
        # These should be marked dangerous
        dangerous_tools = ["ros2_action_goal", "ros2_param_set", "safety_trigger_estop", "safety_release_estop"]
        
        for tool_name in dangerous_tools:
            tool = adapter.get_tool(tool_name)
            assert tool.dangerous is True, f"Tool {tool_name} should be dangerous"

    def test_tool_to_openclaw_format_structure(self):
        """Test tool format structure."""
        tool = OpenClawTool(
            name="test_tool",
            description="A test tool",
            parameters={"arg1": {"type": "string", "description": "An argument"}},
            dangerous=False,
        )
        
        result = tool.to_openclaw_format()
        
        assert result["name"] == "test_tool"
        assert result["description"] == "A test tool"
        assert result["dangerous"] is False
        assert "parameters" in result
        assert result["parameters"]["type"] == "object"
        assert "properties" in result["parameters"]
        assert "required" in result["parameters"]


class TestOpenClawAdapterExecuteToolDetailed:
    """Detailed tool execution tests."""

    @pytest.mark.asyncio
    async def test_execute_ros2_param_get(self):
        """Test ros2_param_get execution."""
        mock_bridge = Mock()
        mock_ros2 = MagicMock()
        adapter = OpenClawAdapter(mock_bridge)
        adapter._get_ros2_connector = Mock(return_value=mock_ros2)
        
        result = await adapter.execute_tool("ros2_param_get", {
            "node_name": "/robot",
            "param_name": "max_speed",
        })
        
        # Should attempt execution
        assert "success" in result

    @pytest.mark.asyncio
    async def test_execute_ros2_param_set(self):
        """Test ros2_param_set execution."""
        mock_bridge = Mock()
        mock_ros2 = MagicMock()
        adapter = OpenClawAdapter(mock_bridge)
        adapter._get_ros2_connector = Mock(return_value=mock_ros2)
        
        result = await adapter.execute_tool("ros2_param_set", {
            "node_name": "/robot",
            "param_name": "max_speed",
            "value": 1.0,
        })
        
        assert "success" in result

    @pytest.mark.asyncio
    async def test_execute_ros2_list_services(self):
        """Test ros2_list_services execution."""
        mock_bridge = Mock()
        mock_ros2 = MagicMock()
        mock_ros2.list_services = AsyncMock(return_value=["/spawn", "/reset"])
        adapter = OpenClawAdapter(mock_bridge)
        adapter._get_ros2_connector = Mock(return_value=mock_ros2)
        
        result = await adapter.execute_tool("ros2_list_services", {})
        
        assert result["success"] is True

    @pytest.mark.asyncio
    async def test_execute_ros2_camera_snapshot(self):
        """Test ros2_camera_snapshot execution."""
        mock_bridge = Mock()
        mock_ros2 = MagicMock()
        adapter = OpenClawAdapter(mock_bridge)
        adapter._get_ros2_connector = Mock(return_value=mock_ros2)
        
        result = await adapter.execute_tool("ros2_camera_snapshot", {
            "camera_topic": "/camera/image_raw",
        })
        
        assert "success" in result

    @pytest.mark.asyncio
    async def test_execute_bridge_get_robot_status(self):
        """Test bridge_get_robot_status execution."""
        mock_bridge = Mock()
        mock_bridge.fleets = {}
        adapter = OpenClawAdapter(mock_bridge)
        
        result = await adapter.execute_tool("bridge_get_robot_status", {
            "robot_id": "robot1",
        })
        
        # Should fail since no fleets
        assert result["success"] is False

    @pytest.mark.asyncio
    async def test_execute_fleet_submit_task(self):
        """Test fleet_submit_task execution."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        
        result = await adapter.execute_tool("fleet_submit_task", {
            "task_type": "navigate",
            "target_location": "kitchen",
            "priority": 5,
        })
        
        assert "success" in result

    @pytest.mark.asyncio
    async def test_execute_memory_operations(self):
        """Test memory operations."""
        mock_bridge = Mock()
        mock_memory = MagicMock()
        mock_memory.set = AsyncMock()
        mock_memory.get = AsyncMock(return_value="test_value")
        mock_bridge.memory = mock_memory
        adapter = OpenClawAdapter(mock_bridge)
        
        # Test memory_set
        result = await adapter.execute_tool("memory_set", {
            "key": "test_key",
            "value": "test_value",
            "ttl": 3600,
        })
        
        assert result["success"] is True
        
        # Test memory_get
        result = await adapter.execute_tool("memory_get", {
            "key": "test_key",
        })
        
        assert result["success"] is True
        assert result["data"]["value"] == "test_value"


class TestOpenClawAdapterRosclawCompatibility:
    """Test RosClaw compatibility."""

    def test_rosclaw_format_structure(self):
        """Test RosClaw format structure."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        
        tools = adapter.to_rosclaw_compatible_format()
        
        assert isinstance(tools, list)
        assert len(tools) > 0
        
        for tool in tools:
            assert tool["type"] == "function"
            assert "function" in tool
            assert "name" in tool["function"]
            assert "description" in tool["function"]
            assert "parameters" in tool["function"]

    def test_rosclaw_format_matches_rosclaw_interface(self):
        """Test that format matches RosClaw interface."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        
        tools = adapter.to_rosclaw_compatible_format()
        
        # Should have standard ROS2 tools
        tool_names = [t["function"]["name"] for t in tools]
        assert "ros2_publish" in tool_names
        assert "ros2_subscribe_once" in tool_names


class TestOpenClawAdapterNaturalLanguageDetailed:
    """Detailed natural language tests."""

    @pytest.mark.asyncio
    async def test_execute_nl_navigation(self):
        """Test NL execution for navigation."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        adapter.enable_natural_language()
        
        # Mock interpreter
        adapter.contextual_interpreter = MagicMock()
        adapter.contextual_interpreter.interpret = Mock(return_value={
            "tool": "ros2_action_goal",
            "action_name": "/navigate_to_pose",
            "goal": {"pose": {"position": {"x": 5, "y": 3}}},
            "explanation": "Navigate to coordinates",
            "location_name": "kitchen",
        })
        
        with patch.object(adapter, "execute_tool", return_value={"success": True}):
            result = await adapter.execute_nl("go to kitchen", "session1")
            
            assert result["success"] is True
            assert result["command"] == "go to kitchen"
            assert "interpretation" in result

    @pytest.mark.asyncio
    async def test_execute_nl_unknown_command(self):
        """Test NL execution for unknown command."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        adapter.enable_natural_language()
        
        # Mock interpreter returning error
        adapter.contextual_interpreter = MagicMock()
        adapter.contextual_interpreter.interpret = Mock(return_value={
            "error": "Unknown command",
            "suggestion": "Try: move forward",
        })
        
        result = await adapter.execute_nl("do something weird", "session1")
        
        assert result["success"] is False
        assert "error" in result

    def test_learn_location_updates_context(self):
        """Test that learn_location updates context."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        adapter.enable_natural_language()
        
        adapter.learn_location("session1", "kitchen", {"x": 10, "y": 5})
        
        ctx = adapter.context_manager.get_context("session1")
        assert "kitchen" in ctx.known_locations
        assert ctx.known_locations["kitchen"] == {"x": 10, "y": 5}
        assert ctx.current_location == "kitchen"


class TestOpenClawAdapterSkillPackagingDetailed:
    """Detailed skill packaging tests."""

    def test_package_skill_creates_zip(self):
        """Test that package_skill creates zip file."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        
        with tempfile.TemporaryDirectory() as tmpdir:
            skill_dir = Path(tmpdir) / "agent-ros-bridge"
            skill_dir.mkdir()
            (skill_dir / "SKILL.md").write_text("# Skill Definition")
            (skill_dir / "tools.yaml").write_text("tools: []")
            subdir = skill_dir / "subdir"
            subdir.mkdir()
            (subdir / "file.txt").write_text("content")
            
            adapter._skill_path = skill_dir
            output_dir = Path(tmpdir)
            
            result = adapter.package_skill(output_dir)
            
            assert result.exists()
            assert result.suffix == ".skill"
            
            # Verify zip contents
            import zipfile
            with zipfile.ZipFile(result, 'r') as zf:
                names = zf.namelist()
                assert "SKILL.md" in names
                assert "tools.yaml" in names
                assert "subdir/file.txt" in names
