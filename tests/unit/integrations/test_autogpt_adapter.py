"""Unit tests for integrations/autogpt_adapter.py.

Tests the AutoGPT adapter without external dependencies.
"""

import json
from unittest.mock import AsyncMock, Mock

import pytest

from agent_ros_bridge.integrations.autogpt_adapter import AutoGPTAdapter


class TestAutoGPTAdapter:
    """Test AutoGPTAdapter class."""

    @pytest.fixture
    def mock_bridge(self):
        """Create a mock bridge."""
        bridge = Mock()
        bridge.get_actions = Mock(return_value=["navigate", "move_arm", "get_status"])
        bridge.execute_action = AsyncMock(return_value={"status": "success"})
        return bridge

    def test_adapter_creation(self, mock_bridge):
        """Adapter can be created."""
        adapter = AutoGPTAdapter(mock_bridge)

        assert adapter.bridge == mock_bridge
        assert adapter.commands == {}

    def test_adapter_creation_without_bridge(self):
        """Adapter can be created without bridge."""
        adapter = AutoGPTAdapter(None)

        assert adapter.bridge is None

    def test_discover_commands(self, mock_bridge):
        """Commands can be discovered from bridge."""
        adapter = AutoGPTAdapter(mock_bridge)

        commands = adapter.discover_commands()

        assert len(commands) == 3
        assert "ros_navigate" in commands
        assert "ros_move_arm" in commands
        assert "ros_get_status" in commands

    def test_discover_commands_without_bridge(self):
        """Discover commands handles missing bridge."""
        adapter = AutoGPTAdapter(None)

        commands = adapter.discover_commands()

        assert commands == {}

    def test_discover_commands_without_get_actions(self):
        """Discover commands handles bridge without get_actions."""
        bridge = Mock()
        # No get_actions attribute
        del bridge.get_actions

        adapter = AutoGPTAdapter(bridge)
        commands = adapter.discover_commands()

        assert commands == {}

    def test_get_commands_format(self, mock_bridge):
        """Commands are returned in AutoGPT format."""
        adapter = AutoGPTAdapter(mock_bridge)
        adapter.discover_commands()

        commands = adapter.get_commands()

        assert isinstance(commands, list)
        assert len(commands) == 3

        # Check command structure
        for cmd in commands:
            assert "name" in cmd
            assert "description" in cmd
            assert "arguments" in cmd
            assert cmd["name"].startswith("ros_")

    def test_get_commands_caches_discovery(self, mock_bridge):
        """get_commands caches discovered commands."""
        adapter = AutoGPTAdapter(mock_bridge)

        # First call discovers
        commands1 = adapter.get_commands()
        # Second call uses cache
        commands2 = adapter.get_commands()

        assert commands1 == commands2
        mock_bridge.get_actions.assert_called_once()

    @pytest.mark.asyncio
    async def test_execute_command(self, mock_bridge):
        """Command can be executed."""
        adapter = AutoGPTAdapter(mock_bridge)

        result = await adapter.execute_command("ros_navigate", x=1.0, y=2.0)

        mock_bridge.execute_action.assert_called_once_with("navigate", {"x": 1.0, "y": 2.0})
        result_dict = json.loads(result)
        assert result_dict["status"] == "success"

    @pytest.mark.asyncio
    async def test_execute_command_without_bridge(self):
        """Execute command handles missing bridge."""
        adapter = AutoGPTAdapter(None)

        result = await adapter.execute_command("ros_navigate")

        result_dict = json.loads(result)
        assert "error" in result_dict
        assert "Bridge not available" in result_dict["error"]

    @pytest.mark.asyncio
    async def test_execute_command_without_execute_action(self):
        """Execute command handles bridge without execute_action."""
        bridge = Mock()
        bridge.get_actions = Mock(return_value=["navigate"])
        # Remove execute_action attribute if it exists
        if hasattr(bridge, "execute_action"):
            delattr(bridge, "execute_action")

        adapter = AutoGPTAdapter(bridge)

        result = await adapter.execute_command("ros_navigate")

        result_dict = json.loads(result)
        assert "error" in result_dict
        assert "doesn't support execute_action" in result_dict["error"]

    @pytest.mark.asyncio
    async def test_execute_command_exception(self, mock_bridge):
        """Execute command handles exceptions."""
        mock_bridge.execute_action = AsyncMock(side_effect=Exception("Connection failed"))
        adapter = AutoGPTAdapter(mock_bridge)

        result = await adapter.execute_command("ros_navigate")

        result_dict = json.loads(result)
        assert "error" in result_dict
        assert "Connection failed" in result_dict["error"]

    def test_to_autogpt_plugin_format(self, mock_bridge):
        """Adapter exports to AutoGPT plugin format."""
        adapter = AutoGPTAdapter(mock_bridge)
        adapter.discover_commands()

        plugin = adapter.to_autogpt_plugin_format()

        assert plugin["name"] == "agent_ros_bridge"
        assert plugin["version"] == "0.5.0"
        assert "Control ROS robot" in plugin["description"]
        assert "commands" in plugin
        assert len(plugin["commands"]) == 3

    def test_command_name_extraction(self, mock_bridge):
        """Command name is extracted correctly."""
        adapter = AutoGPTAdapter(mock_bridge)

        # Test various command names
        assert adapter.discover_commands()  # Populate commands

        # The action name should be extracted from ros_<action> format
        commands = adapter.get_commands()
        command_names = [cmd["name"] for cmd in commands]

        assert "ros_navigate" in command_names
        assert "ros_move_arm" in command_names
        assert "ros_get_status" in command_names


class TestAutoGPTAdapterEdgeCases:
    """Test AutoGPTAdapter edge cases."""

    @pytest.fixture
    def mock_bridge(self):
        """Create a mock bridge."""
        bridge = Mock()
        bridge.get_actions = Mock(return_value=[])
        bridge.execute_action = AsyncMock()
        return bridge

    def test_empty_actions_list(self, mock_bridge):
        """Handle empty actions list."""
        mock_bridge.get_actions.return_value = []
        adapter = AutoGPTAdapter(mock_bridge)

        commands = adapter.discover_commands()

        assert commands == {}

    def test_special_characters_in_action_names(self, mock_bridge):
        """Handle special characters in action names."""
        mock_bridge.get_actions.return_value = ["move_arm_v2", "navigate-to-point", "get.status"]
        adapter = AutoGPTAdapter(mock_bridge)

        commands = adapter.discover_commands()

        assert "ros_move_arm_v2" in commands
        assert "ros_navigate-to-point" in commands
        assert "ros_get.status" in commands

    @pytest.mark.asyncio
    async def test_execute_command_with_empty_params(self, mock_bridge):
        """Execute command with no parameters."""
        adapter = AutoGPTAdapter(mock_bridge)

        result = await adapter.execute_command("ros_navigate")

        mock_bridge.execute_action.assert_called_once_with("navigate", {})

    @pytest.mark.asyncio
    async def test_execute_command_with_many_params(self, mock_bridge):
        """Execute command with many parameters."""
        adapter = AutoGPTAdapter(mock_bridge)

        params = {
            "x": 1.0,
            "y": 2.0,
            "z": 0.0,
            "orientation": 1.57,
            "speed": 0.5,
        }
        result = await adapter.execute_command("ros_navigate", **params)

        mock_bridge.execute_action.assert_called_once_with("navigate", params)

    def test_command_description_generation(self, mock_bridge):
        """Command descriptions are generated correctly."""
        mock_bridge.get_actions.return_value = ["navigate", "emergency_stop"]
        adapter = AutoGPTAdapter(mock_bridge)
        adapter.discover_commands()

        commands = adapter.get_commands()

        navigate_cmd = next(cmd for cmd in commands if cmd["name"] == "ros_navigate")
        assert "navigate" in navigate_cmd["description"].lower()

    def test_command_params_field(self, mock_bridge):
        """Command params field is populated."""
        adapter = AutoGPTAdapter(mock_bridge)
        adapter.discover_commands()

        commands = adapter.get_commands()

        for cmd in commands:
            assert "arguments" in cmd
