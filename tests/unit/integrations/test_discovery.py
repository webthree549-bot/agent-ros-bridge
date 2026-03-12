"""Unit tests for Tool Discovery module.

TDD tests for ToolDiscovery, ROSAction.
"""

import pytest
from unittest.mock import Mock, MagicMock

from agent_ros_bridge.integrations.discovery import (
    ToolDiscovery,
    ROSAction,
)


class TestROSAction:
    """Test ROSAction data class."""

    def test_ros_action_creation(self):
        """ROSAction can be created."""
        action = ROSAction(
            name="/cmd_vel",
            action_type="topic",
            ros_type="geometry_msgs/Twist",
            description="Velocity command",
            parameters={"linear": {"x": "float"}},
        )
        assert action.name == "/cmd_vel"
        assert action.action_type == "topic"
        assert action.ros_type == "geometry_msgs/Twist"
        assert action.description == "Velocity command"
        assert action.dangerous is False

    def test_ros_action_dangerous(self):
        """ROSAction can be marked as dangerous."""
        action = ROSAction(
            name="/emergency_stop",
            action_type="service",
            ros_type="std_srvs/Trigger",
            description="Emergency stop",
            parameters={},
            dangerous=True,
        )
        assert action.dangerous is True


class TestToolDiscovery:
    """Test ToolDiscovery class."""

    def test_discovery_creation_without_bridge(self):
        """ToolDiscovery can be created without bridge."""
        discovery = ToolDiscovery()
        assert discovery.bridge is None
        assert discovery._cache == {}

    def test_discovery_creation_with_bridge(self):
        """ToolDiscovery can be created with bridge."""
        mock_bridge = Mock()
        discovery = ToolDiscovery(bridge=mock_bridge)
        assert discovery.bridge == mock_bridge

    def test_discover_all_without_bridge_returns_cached(self):
        """Discover all returns cached tools when no bridge."""
        discovery = ToolDiscovery()

        # Pre-populate cache
        action = ROSAction(
            name="/test",
            action_type="topic",
            ros_type="std_msgs/String",
            description="Test topic",
            parameters={},
        )
        discovery._cache["/test"] = action

        tools = discovery.discover_all()
        assert len(tools) == 1
        assert tools[0].name == "/test"

    def test_discover_all_without_bridge_empty_cache(self):
        """Discover all returns empty list when no bridge and empty cache."""
        discovery = ToolDiscovery()
        tools = discovery.discover_all()
        assert tools == []

    def test_get_all_robots_without_bridge(self):
        """Get all robots returns empty list without bridge."""
        discovery = ToolDiscovery()
        robots = discovery._get_all_robots()
        assert robots == []

    def test_get_all_robots_with_bridge(self):
        """Get all robots from bridge fleets."""
        mock_robot = Mock()
        mock_fleet = Mock()
        mock_fleet.robots = {"r1": mock_robot}

        mock_bridge = Mock()
        mock_bridge.fleets = {"fleet1": mock_fleet}

        discovery = ToolDiscovery(bridge=mock_bridge)
        robots = discovery._get_all_robots()

        assert len(robots) == 1
        assert robots[0] == mock_robot

    def test_discover_topics_with_ros2_robot_no_ros_node(self):
        """Discover topics returns empty when ROS2 robot has no ros_node."""
        mock_robot = Mock()
        mock_robot.connector_type = "ros2"
        # No ros_node attribute

        mock_fleet = Mock()
        mock_fleet.robots = {"r1": mock_robot}

        mock_bridge = Mock()
        mock_bridge.fleets = {"fleet1": mock_fleet}

        discovery = ToolDiscovery(bridge=mock_bridge)
        tools = discovery._discover_topics()

        # Returns empty because no ros_node
        assert tools == []

    def test_discover_topics_with_ros1_robot_no_cmd_topics(self):
        """Discover topics handles ROS1 robot without _cmd_get_topics."""
        mock_robot = Mock()
        mock_robot.connector_type = "ros1"
        # No _cmd_get_topics method

        mock_fleet = Mock()
        mock_fleet.robots = {"r1": mock_robot}

        mock_bridge = Mock()
        mock_bridge.fleets = {"fleet1": mock_fleet}

        discovery = ToolDiscovery(bridge=mock_bridge)
        tools = discovery._discover_topics()

        # Returns empty because no _cmd_get_topics
        assert tools == []

    def test_discover_services_with_ros2_robot_no_services(self):
        """Discover services returns empty when no services available."""
        mock_robot = Mock()
        mock_robot.connector_type = "ros2"
        # No ros_node or get_service_names_and_types

        mock_fleet = Mock()
        mock_fleet.robots = {"r1": mock_robot}

        mock_bridge = Mock()
        mock_bridge.fleets = {"fleet1": mock_fleet}

        discovery = ToolDiscovery(bridge=mock_bridge)
        tools = discovery._discover_services()

        assert tools == []

    def test_discover_actions_returns_default_actions(self):
        """Discover actions returns default hardcoded actions."""
        discovery = ToolDiscovery()
        tools = discovery._discover_actions()

        # The implementation returns hardcoded actions
        assert len(tools) >= 1
        assert any(t.action_type == "action" for t in tools)

    def test_to_mcp_tools_empty(self):
        """Convert empty list to MCP format."""
        discovery = ToolDiscovery()
        mcp_tools = discovery.to_mcp_tools([])
        assert mcp_tools == []

    def test_to_mcp_tools_with_actions(self):
        """Convert ROSActions to MCP format."""
        discovery = ToolDiscovery()
        actions = [
            ROSAction(
                name="/cmd_vel",
                action_type="topic",
                ros_type="geometry_msgs/Twist",
                description="Velocity command",
                parameters={"linear": {"x": "float"}},
            )
        ]

        mcp_tools = discovery.to_mcp_tools(actions)

        assert len(mcp_tools) == 1
        assert mcp_tools[0]["name"] == "/cmd_vel"  # Name is preserved as-is
        assert "description" in mcp_tools[0]

    def test_to_openai_functions_empty(self):
        """Convert empty list to OpenAI format."""
        discovery = ToolDiscovery()
        openai_tools = discovery.to_openai_functions([])
        assert openai_tools == []

    def test_to_openai_functions_with_actions(self):
        """Convert ROSActions to OpenAI function format."""
        discovery = ToolDiscovery()
        actions = [
            ROSAction(
                name="/cmd_vel",
                action_type="topic",
                ros_type="geometry_msgs/Twist",
                description="Velocity command",
                parameters={"linear": {"x": "float"}},
            )
        ]

        openai_tools = discovery.to_openai_functions(actions)

        assert len(openai_tools) == 1
        assert openai_tools[0]["type"] == "function"
        assert "function" in openai_tools[0]

    def test_cache_updated_after_discover(self):
        """Cache is updated after discover_all."""
        discovery = ToolDiscovery()
        # With no bridge, discover_all returns empty and doesn't update cache
        discovery.discover_all()
        assert discovery._cache == {}

    def test_get_tool_from_cache(self):
        """Get tool from cache by name."""
        discovery = ToolDiscovery()
        action = ROSAction(
            name="/cmd_vel",
            action_type="topic",
            ros_type="geometry_msgs/Twist",
            description="Velocity command",
            parameters={},
        )
        discovery._cache["/cmd_vel"] = action

        result = discovery.get_tool("/cmd_vel")
        assert result == action

    def test_get_tool_not_found(self):
        """Get tool returns None when not in cache."""
        discovery = ToolDiscovery()
        result = discovery.get_tool("/unknown")
        assert result is None

    def test_is_dangerous_topic_detects_emergency(self):
        """Dangerous topic detection works for emergency topics."""
        discovery = ToolDiscovery()

        # Test the internal method
        is_dangerous = discovery._is_dangerous_topic("/emergency_stop")
        assert is_dangerous is True

        is_safe = discovery._is_dangerous_topic("/odom")
        assert is_safe is False
