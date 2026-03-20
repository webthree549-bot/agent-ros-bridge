"""Additional tests for OpenClaw adapter."""

from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, Mock, patch

import pytest

from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter, OpenClawTool


class TestOpenClawAdapterROS1:
    """Test ROS1 support."""

    def test_init_with_ros1(self):
        """Test initialization with ROS1."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge, include_ros1=True)
        
        # Should have ROS1 tools
        assert "ros1_publish" in adapter._tools
        assert "ros1_subscribe_once" in adapter._tools
        assert "ros1_service_call" in adapter._tools

    def test_ros1_tool_formats(self):
        """Test ROS1 tool formats."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge, include_ros1=True)
        
        ros1_publish = adapter.get_tool("ros1_publish")
        assert ros1_publish is not None
        assert "topic" in ros1_publish.parameters


class TestOpenClawAdapterToolExecution:
    """Test tool execution scenarios."""

    @pytest.mark.asyncio
    async def test_execute_ros2_subscribe_once(self):
        """Test ros2_subscribe_once execution."""
        mock_bridge = Mock()
        mock_ros2 = MagicMock()
        mock_ros2.subscribe_once = AsyncMock(return_value={"data": "test"})
        adapter = OpenClawAdapter(mock_bridge)
        adapter._get_ros2_connector = Mock(return_value=mock_ros2)
        
        result = await adapter.execute_tool("ros2_subscribe_once", {"topic": "/test"})
        
        assert result["success"] is True
        assert "data" in result["data"]

    @pytest.mark.asyncio
    async def test_execute_ros2_service_call(self):
        """Test ros2_service_call execution."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        
        result = await adapter.execute_tool("ros2_service_call", {
            "service": "/spawn",
            "service_type": "turtlesim/srv/Spawn",
            "request": {"x": 1.0},
        })
        
        # Should fail since no connector
        assert result["success"] is False

    @pytest.mark.asyncio
    async def test_execute_safety_release_estop(self):
        """Test safety_release_estop execution."""
        mock_bridge = Mock()
        mock_safety = MagicMock()
        mock_bridge.safety_manager = mock_safety
        adapter = OpenClawAdapter(mock_bridge)
        
        result = await adapter.execute_tool("safety_release_estop", {"robot_id": "r1"})
        
        # Should work with safety manager
        assert "success" in result


class TestOpenClawAdapterNaturalLanguage:
    """Test natural language features."""

    def test_enable_natural_language(self):
        """Test enabling NL support."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        adapter.enable_natural_language()
        
        assert hasattr(adapter, "nl_interpreter")
        assert hasattr(adapter, "context_manager")
        assert hasattr(adapter, "contextual_interpreter")

    @pytest.mark.asyncio
    async def test_execute_nl_with_interpreter(self):
        """Test NL execution with interpreter."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        adapter.enable_natural_language()
        
        # Mock the interpreter
        adapter.contextual_interpreter = MagicMock()
        adapter.contextual_interpreter.interpret = Mock(return_value={
            "tool": "ros2_publish",
            "topic": "/cmd_vel",
            "explanation": "Move forward",
        })
        
        with patch.object(adapter, "execute_tool", return_value={"success": True}):
            result = await adapter.execute_nl("move forward", "session1")
            
            assert "success" in result
            assert "interpretation" in result

    def test_learn_location(self):
        """Test learning location."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        adapter.enable_natural_language()
        
        adapter.learn_location("session1", "kitchen", {"x": 10, "y": 5})
        
        # Should have learned the location
        ctx = adapter.context_manager.get_context("session1")
        assert "kitchen" in ctx.known_locations

    def test_get_conversation_history(self):
        """Test getting conversation history."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        adapter.enable_natural_language()
        
        # Add some history
        adapter.context_manager.log_interaction(
            "session1", "go", {"tool": "nav"}, {"success": True}
        )
        
        history = adapter.get_conversation_history("session1", n=5)
        assert isinstance(history, list)


class TestOpenClawAdapterHelpers:
    """Test helper methods."""

    def test_get_ros2_connector_found(self):
        """Test getting ROS2 connector when available."""
        mock_bridge = Mock()
        mock_connector = MagicMock()
        mock_connector.connector_type = "ros2"
        mock_bridge.transport_manager.connectors = [mock_connector]
        
        adapter = OpenClawAdapter(mock_bridge)
        connector = adapter._get_ros2_connector()
        
        assert connector is mock_connector

    def test_get_ros2_connector_direct(self):
        """Test getting ROS2 connector via direct access."""
        mock_bridge = Mock()
        mock_connector = MagicMock()
        mock_bridge.ros2_connector = mock_connector
        del mock_bridge.transport_manager
        
        adapter = OpenClawAdapter(mock_bridge)
        connector = adapter._get_ros2_connector()
        
        assert connector is mock_connector

    def test_get_ros1_connector_found(self):
        """Test getting ROS1 connector when available."""
        mock_bridge = Mock()
        mock_connector = MagicMock()
        mock_connector.connector_type = "ros1"
        mock_bridge.transport_manager.connectors = [mock_connector]
        
        adapter = OpenClawAdapter(mock_bridge)
        connector = adapter._get_ros1_connector()
        
        assert connector is mock_connector


class TestOpenClawAdapterSkillPackaging:
    """Test skill packaging."""

    def test_package_skill_success(self):
        """Test successful skill packaging."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        
        # Create a temp skill directory
        import os
        import tempfile
        with tempfile.TemporaryDirectory() as tmpdir:
            skill_dir = Path(tmpdir) / "agent-ros-bridge"
            skill_dir.mkdir()
            (skill_dir / "SKILL.md").write_text("# Skill")
            (skill_dir / "test.txt").write_text("test")
            
            adapter._skill_path = skill_dir
            output_dir = Path(tmpdir)
            
            result = adapter.package_skill(output_dir)
            
            assert result.exists()
            assert result.suffix == ".skill"

    def test_package_skill_no_path(self):
        """Test packaging with no skill path."""
        mock_bridge = Mock()
        adapter = OpenClawAdapter(mock_bridge)
        adapter._skill_path = None
        
        with pytest.raises(RuntimeError, match="Skill path not found"):
            adapter.package_skill(Path("/tmp"))
