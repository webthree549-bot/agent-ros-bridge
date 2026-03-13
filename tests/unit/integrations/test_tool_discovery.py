"""Unit tests for integrations/discovery.py.

Tests the tool discovery system without external dependencies.
"""

import pytest
from unittest.mock import Mock, AsyncMock, patch, MagicMock

from agent_ros_bridge.integrations.discovery import ToolDiscovery, ROSAction


class TestROSAction:
    """Test ROSAction dataclass."""

    def test_action_creation(self):
        """ROSAction can be created."""
        action = ROSAction(
            name="navigate",
            action_type="action",
            ros_type="navigate_action/Goal",
            description="Navigate to a pose",
            parameters={"x": {"type": "float"}},
        )

        assert action.name == "navigate"
        assert action.action_type == "action"
        assert action.ros_type == "navigate_action/Goal"
        assert action.description == "Navigate to a pose"
        assert action.dangerous is False

    def test_action_with_dangerous_flag(self):
        """ROSAction can be marked as dangerous."""
        action = ROSAction(
            name="emergency_stop",
            action_type="service",
            ros_type="std_srvs/Trigger",
            description="Emergency stop",
            parameters={},
            dangerous=True,
        )

        assert action.dangerous is True


class TestToolDiscovery:
    """Test ToolDiscovery class."""

    @pytest.fixture
    def mock_bridge(self):
        """Create a mock bridge."""
        bridge = Mock()
        bridge.fleets = {}
        return bridge

    @pytest.fixture
    def discovery(self, mock_bridge):
        """Create a ToolDiscovery instance."""
        return ToolDiscovery(mock_bridge)

    def test_discovery_creation(self, mock_bridge):
        """Discovery can be created."""
        discovery = ToolDiscovery(mock_bridge)

        assert discovery.bridge == mock_bridge
        assert discovery._cache == {}

    def test_discovery_creation_without_bridge(self):
        """Discovery can be created without bridge."""
        discovery = ToolDiscovery(None)

        assert discovery.bridge is None

    def test_discover_all_with_bridge(self, discovery, mock_bridge):
        """All tools can be discovered with bridge."""
        # Mock _discover methods to return empty lists
        discovery._discover_topics = Mock(return_value=[])
        discovery._discover_services = Mock(return_value=[])
        discovery._discover_actions = Mock(return_value=[])

        tools = discovery.discover_all()

        discovery._discover_topics.assert_called_once()
        discovery._discover_services.assert_called_once()
        discovery._discover_actions.assert_called_once()

    def test_discover_all_without_bridge(self):
        """Discover all returns cached tools without bridge."""
        discovery = ToolDiscovery(None)
        # Pre-populate cache
        cached_action = ROSAction(
            name="cached_action",
            action_type="action",
            ros_type="test",
            description="Cached",
            parameters={},
        )
        discovery._cache["cached_action"] = cached_action

        tools = discovery.discover_all()

        assert len(tools) == 1
        assert tools[0].name == "cached_action"

    def test_discover_all_caches_results(self, discovery, mock_bridge):
        """Discover all caches discovered tools."""
        action = ROSAction(
            name="test_action",
            action_type="action",
            ros_type="test",
            description="Test",
            parameters={},
        )
        discovery._discover_topics = Mock(return_value=[])
        discovery._discover_services = Mock(return_value=[])
        discovery._discover_actions = Mock(return_value=[action])

        discovery.discover_all()

        assert "test_action" in discovery._cache

    def test_get_all_robots_empty(self, discovery):
        """Get all robots returns empty list when no fleets."""
        robots = discovery._get_all_robots()

        assert robots == []

    def test_get_all_robots_with_fleets(self, discovery, mock_bridge):
        """Get all robots from all fleets."""
        # Set up mock fleet with robots
        mock_fleet = Mock()
        mock_robot1 = Mock()
        mock_robot2 = Mock()
        mock_fleet.robots = {"r1": mock_robot1, "r2": mock_robot2}
        mock_bridge.fleets = {"fleet1": mock_fleet}

        robots = discovery._get_all_robots()

        assert len(robots) == 2
        assert mock_robot1 in robots
        assert mock_robot2 in robots

    def test_make_tool_name(self, discovery):
        """Tool name is created from topic name."""
        name = discovery._make_tool_name("/cmd_vel", "input")

        assert name == "write_cmd_vel"

    def test_make_tool_name_output(self, discovery):
        """Tool name reflects output direction."""
        name = discovery._make_tool_name("/odom", "output")

        assert name == "read_odom"

    def test_infer_topic_direction_command(self, discovery):
        """Command topics are identified as input."""
        direction = discovery._infer_topic_direction("/cmd_vel")

        assert direction == "input"

    def test_infer_topic_direction_telemetry(self, discovery):
        """Telemetry topics are identified as output."""
        direction = discovery._infer_topic_direction("/odom")

        assert direction == "output"

    def test_infer_topic_direction_unknown(self, discovery):
        """Unknown topics default to output."""
        direction = discovery._infer_topic_direction("/unknown_topic")

        assert direction == "output"

    def test_is_dangerous_topic(self, discovery):
        """Dangerous topics are identified."""
        assert discovery._is_dangerous_topic("/cmd_vel") is True
        assert discovery._is_dangerous_topic("/emergency_stop") is True
        assert discovery._is_dangerous_topic("/odom") is False

    def test_is_dangerous_service(self, discovery):
        """Dangerous services are identified."""
        assert discovery._is_dangerous_service("/estop") is True
        assert discovery._is_dangerous_service("/emergency") is True
        assert discovery._is_dangerous_service("/reset") is True
        assert discovery._is_dangerous_service("/get_status") is False

    def test_discover_actions_returns_common_actions(self, discovery):
        """Discover actions returns common ROS actions."""
        actions = discovery._discover_actions()

        action_names = [a.name for a in actions]
        assert "navigate_to_pose" in action_names
        assert "move_base" in action_names
        assert "pick_object" in action_names
        assert "place_object" in action_names

    def test_discover_actions_marks_dangerous(self, discovery):
        """Discover actions marks dangerous actions."""
        actions = discovery._discover_actions()

        pick_action = next(a for a in actions if a.name == "pick_object")
        place_action = next(a for a in actions if a.name == "place_object")

        assert pick_action.dangerous is True
        assert place_action.dangerous is True

    def test_infer_topic_parameters_twist_input(self, discovery):
        """Parameters inferred for Twist input messages."""
        params = discovery._infer_topic_parameters("geometry_msgs/Twist", "input")

        assert "linear" in params
        assert "angular" in params

    def test_infer_topic_parameters_twist_output(self, discovery):
        """No parameters for Twist output (reading)."""
        params = discovery._infer_topic_parameters("geometry_msgs/Twist", "output")

        assert params == {}

    def test_infer_topic_parameters_pose_input(self, discovery):
        """Parameters inferred for Pose input messages."""
        params = discovery._infer_topic_parameters("geometry_msgs/Pose", "input")

        assert "position" in params
        assert "orientation" in params

    def test_infer_topic_parameters_scan_input(self, discovery):
        """Parameters inferred for LaserScan input."""
        params = discovery._infer_topic_parameters("sensor_msgs/LaserScan", "input")

        assert "ranges" in params
        assert "angle_min" in params

    def test_infer_topic_parameters_odometry_input(self, discovery):
        """Parameters inferred for Odometry input."""
        params = discovery._infer_topic_parameters("nav_msgs/Odometry", "input")

        assert "pose" in params
        assert "twist" in params

    def test_infer_topic_parameters_unknown(self, discovery):
        """Parameters for unknown type."""
        params = discovery._infer_topic_parameters("unknown/Type", "input")

        assert "data" in params

    def test_get_tool(self, discovery):
        """Specific tool can be retrieved from cache."""
        # Pre-populate cache
        action = ROSAction(
            name="test_action",
            action_type="action",
            ros_type="test",
            description="Test",
            parameters={},
        )
        discovery._cache["test_action"] = action

        tool = discovery.get_tool("test_action")

        assert tool is not None
        assert tool.name == "test_action"

    def test_get_tool_not_found(self, discovery):
        """Getting non-existent tool returns None."""
        tool = discovery.get_tool("nonexistent")

        assert tool is None

    def test_invalidate_cache(self, discovery):
        """Cache can be invalidated."""
        discovery._cache["test"] = Mock()

        discovery.invalidate_cache()

        assert discovery._cache == {}

    def test_get_dangerous_tools(self, discovery):
        """Dangerous tools can be retrieved."""
        # Clear cache first
        discovery._cache.clear()
        
        # Clear cache to avoid interference from default actions
        discovery._cache.clear()

        # Pre-populate cache with dangerous and safe actions
        safe_action = ROSAction(
            name="safe_action",
            action_type="action",
            ros_type="test",
            description="Safe",
            parameters={},
            dangerous=False,
        )
        dangerous_action = ROSAction(
            name="dangerous_action",
            action_type="action",
            ros_type="test",
            description="Dangerous",
            parameters={},
            dangerous=True,
        )
        discovery._cache["safe_action"] = safe_action
        discovery._cache["dangerous_action"] = dangerous_action

        dangerous = discovery.get_dangerous_tools()

        # Should only return the dangerous action
        assert len(dangerous) == 1
        assert dangerous[0].name == "dangerous_action"


class TestToolDiscoveryROS2:
    """Test ToolDiscovery ROS2-specific functionality."""

    @pytest.fixture
    def mock_ros2_robot(self):
        """Create a mock ROS2 robot."""
        robot = Mock()
        robot.connector_type = "ros2"
        robot.name = "test_robot"
        robot.ros_node = Mock()
        # Mock returns list of tuples (topic_name, [types])
        robot.ros_node.get_topic_names_and_types = Mock(return_value=[
            ("/cmd_vel", ["geometry_msgs/msg/Twist"]),
            ("/odom", ["nav_msgs/msg/Odometry"]),
        ])
        robot.ros_node.get_service_names_and_types = Mock(return_value=[
            ("/reset", ["std_srvs/srv/Empty"]),
        ])
        return robot

    @pytest.fixture
    def discovery(self):
        """Create a ToolDiscovery instance."""
        bridge = Mock()
        bridge.fleets = {}
        return ToolDiscovery(bridge)

    def test_discover_ros2_topics(self, discovery, mock_ros2_robot):
        """ROS2 topics can be discovered."""
        tools = discovery._discover_ros2_topics(mock_ros2_robot)

        assert len(tools) == 2
        tool_names = [t.name for t in tools]
        assert "write_cmd_vel" in tool_names
        assert "read_odom" in tool_names

    def test_discover_ros2_topics_no_node(self, discovery):
        """Discover ROS2 topics handles missing node."""
        robot = Mock()
        robot.ros_node = None

        tools = discovery._discover_ros2_topics(robot)

        assert tools == []

    def test_discover_ros2_services(self, discovery, mock_ros2_robot):
        """ROS2 services can be discovered."""
        tools = discovery._discover_ros2_services(mock_ros2_robot)

        assert len(tools) == 1
        assert tools[0].name == "call_reset"
        assert tools[0].action_type == "service"

    def test_discover_ros2_services_no_node(self, discovery):
        """Discover ROS2 services handles missing node."""
        robot = Mock()
        robot.ros_node = None

        tools = discovery._discover_ros2_services(robot)

        assert tools == []


class TestToolDiscoveryROS1:
    """Test ToolDiscovery ROS1-specific functionality."""

    @pytest.fixture
    def mock_ros1_robot(self):
        """Create a mock ROS1 robot."""
        robot = Mock()
        robot.connector_type = "ros1"
        robot.name = "test_robot"
        robot._cmd_get_topics = Mock(return_value=[
            {"name": "/cmd_vel", "type": "geometry_msgs/Twist"},
            {"name": "/odom", "type": "nav_msgs/Odometry"},
        ])
        robot._cmd_get_services = Mock(return_value=[
            {"name": "/reset", "providers": ["/node1"]},
        ])
        return robot

    @pytest.fixture
    def discovery(self):
        """Create a ToolDiscovery instance."""
        bridge = Mock()
        bridge.fleets = {}
        return ToolDiscovery(bridge)

    def test_discover_ros1_topics(self, discovery, mock_ros1_robot):
        """ROS1 topics can be discovered."""
        tools = discovery._discover_ros1_topics(mock_ros1_robot)

        assert len(tools) == 2

    def test_discover_ros1_topics_error(self, discovery):
        """Discover ROS1 topics handles errors."""
        robot = Mock()
        robot._cmd_get_topics = Mock(side_effect=Exception("ROS error"))

        tools = discovery._discover_ros1_topics(robot)

        assert tools == []

    def test_discover_ros1_services(self, discovery, mock_ros1_robot):
        """ROS1 services can be discovered."""
        tools = discovery._discover_ros1_services(mock_ros1_robot)

        assert len(tools) == 1
        assert "reset" in tools[0].name

    def test_discover_ros1_services_error(self, discovery):
        """Discover ROS1 services handles errors."""
        robot = Mock()
        robot._cmd_get_services = Mock(side_effect=Exception("ROS error"))

        tools = discovery._discover_ros1_services(robot)

        assert tools == []


class TestToolDiscoveryExportFormats:
    """Test ToolDiscovery export formats."""

    @pytest.fixture
    def sample_tools(self):
        """Create sample tools for testing."""
        return [
            ROSAction(
                name="navigate",
                action_type="action",
                ros_type="navigate_action/Goal",
                description="Navigate to pose",
                parameters={"x": {"type": "float", "description": "X coordinate"}},
                dangerous=False,
            ),
            ROSAction(
                name="emergency_stop",
                action_type="service",
                ros_type="std_srvs/Trigger",
                description="Emergency stop",
                parameters={},
                dangerous=True,
            ),
        ]

    @pytest.fixture
    def discovery(self):
        """Create a ToolDiscovery instance."""
        return ToolDiscovery(None)

    def test_to_mcp_tools(self, discovery, sample_tools):
        """Tools can be converted to MCP format."""
        mcp_tools = discovery.to_mcp_tools(sample_tools)

        assert isinstance(mcp_tools, list)
        assert len(mcp_tools) == 2

        # Check MCP tool structure
        for tool in mcp_tools:
            assert "name" in tool
            assert "description" in tool
            assert "inputSchema" in tool

    def test_to_openai_functions(self, discovery, sample_tools):
        """Tools can be converted to OpenAI function format."""
        openai_tools = discovery.to_openai_functions(sample_tools)

        assert isinstance(openai_tools, list)
        assert len(openai_tools) == 2

        # Check OpenAI function structure - returns list of function dicts
        for tool in openai_tools:
            assert "type" in tool
            assert tool["type"] == "function"
            assert "function" in tool
            func = tool["function"]
            assert "name" in func
            assert "description" in func
            assert "parameters" in func

    def test_to_langchain_tools(self, discovery, sample_tools):
        """Tools can be converted to LangChain format."""
        langchain_tools = discovery.to_langchain_tools(sample_tools)

        assert isinstance(langchain_tools, list)
        assert len(langchain_tools) == 2

        for tool in langchain_tools:
            assert "name" in tool
            assert "description" in tool
            assert "args_schema" in tool

    def test_export_empty_tools(self, discovery):
        """Export handles empty tool list."""
        mcp_tools = discovery.to_mcp_tools([])
        openai_tools = discovery.to_openai_functions([])

        assert mcp_tools == []
        assert openai_tools == []

    def test_export_none_tools(self, discovery):
        """Export handles None tools by discovering."""
        discovery.discover_all = Mock(return_value=[])

        mcp_tools = discovery.to_mcp_tools(None)

        discovery.discover_all.assert_called_once()
