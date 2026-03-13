"""Unit tests for ROS1 Connector - TDD Style.

Tests written BEFORE implementation (following TDD Red-Green-Refactor).
"""

import asyncio
from unittest.mock import Mock, patch

import pytest

# Skip if ROS1 not available
pytestmark = [
    pytest.mark.unit,
    pytest.mark.skipif(
        not pytest.importorskip("rospy", reason="ROS1 not available"),
        reason="ROS1 (rospy) not installed",
    ),
]


class TestROS1MessageRegistry:
    """Test 1: Message type registry should exist with common types."""

    def test_message_registry_exists(self):
        """Red: Message registry should be importable."""
        from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
            MESSAGE_TYPE_REGISTRY

        assert MESSAGE_TYPE_REGISTRY is not None
        assert isinstance(MESSAGE_TYPE_REGISTRY, dict)

    def test_common_types_registered(self):
        """Red: Common ROS1 types should be pre-registered."""
        from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
            MESSAGE_TYPE_REGISTRY

        common_types = [
            "std_msgs/String",
            "geometry_msgs/Twist",
            "nav_msgs/Odometry",
            "sensor_msgs/LaserScan",
        ]

        for msg_type in common_types:
            assert msg_type in MESSAGE_TYPE_REGISTRY, f"{msg_type} should be registered"

    def test_registry_has_module_and_class(self):
        """Red: Each entry should have (module, class) tuple."""
        from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
            MESSAGE_TYPE_REGISTRY

        for msg_type, (module, class_name) in MESSAGE_TYPE_REGISTRY.items():
            assert isinstance(module, str), f"{msg_type}: module should be string"
            assert isinstance(class_name, str), f"{msg_type}: class_name should be string"
            assert ".msg" in module or ".srv" in module, f"{msg_type}: should be msg or srv module"


class TestGetMessageClass:
    """Test 2: get_message_class() should dynamically import messages."""

    def test_get_registered_message_class(self):
        """Red: Should return class for registered types."""
        from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
            get_message_class

        msg_class = get_message_class("std_msgs/String")
        assert msg_class is not None
        assert msg_class.__name__ == "String"

    def test_get_unknown_message_class_dynamically(self):
        """Red: Should attempt dynamic import for unknown types."""
        from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
            get_message_class

        # This might succeed or fail depending on ROS installation
        # but should not raise an exception
        msg_class = get_message_class("unknown_package/UnknownMsg")
        # Returns None for unknown types (graceful failure)
        assert msg_class is None or hasattr(msg_class, "__name__")

    def test_get_message_class_invalid_format(self):
        """Red: Should handle invalid message type format."""
        from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
            get_message_class

        result = get_message_class("invalid_format_no_slash")
        assert result is None


class TestROS1RobotConnection:
    """Test 3: ROS1Robot should connect and track state."""

    @pytest.fixture
    def mock_rospy(self):
        """Mock rospy for testing without ROS."""
        with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.ROS1_AVAILABLE", True):
            with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.rospy") as mock_ros:
                yield mock_ros

    @pytest.mark.asyncio
    async def test_robot_initialization(self):
        """Red: Robot should initialize with correct properties."""
        from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
            ROS1Robot

        robot = ROS1Robot("test_id", "test_robot")

        assert robot.robot_id == "test_id"
        assert robot.name == "test_robot"
        assert robot.connector_type == "ros1"
        assert robot.robot_type == "ros1"  # Compatibility alias
        assert not robot.connected

    @pytest.mark.asyncio
    async def test_robot_connect_success(self, mock_rospy):
        """Red: Robot should connect and set capabilities."""
        from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
            ROS1Robot

        robot = ROS1Robot("test_id", "test_robot")

        result = await robot.connect()

        assert result is True
        assert robot.connected
        assert "publish" in robot.capabilities
        assert "subscribe" in robot.capabilities
        assert "services" in robot.capabilities
        assert "actions" in robot.capabilities
        mock_rospy.init_node.assert_called_once()

    @pytest.mark.asyncio
    async def test_robot_connect_failure(self, mock_rospy):
        """Red: Robot should handle connection failure gracefully."""
        from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
            ROS1Robot

        mock_rospy.init_node.side_effect = Exception("ROS init failed")

        robot = ROS1Robot("test_id", "test_robot")
        result = await robot.connect()

        assert result is False
        assert not robot.connected

    @pytest.mark.asyncio
    async def test_robot_disconnect_cleanup(self, mock_rospy):
        """Red: Disconnect should clean up resources."""
        from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
            ROS1Robot

        robot = ROS1Robot("test_id", "test_robot")
        await robot.connect()

        # Add mock subscriber/publisher
        mock_sub = Mock()
        mock_pub = Mock()
        robot.subscribers["/test"] = mock_sub
        robot.publishers["/test"] = mock_pub

        await robot.disconnect()

        assert not robot.connected
        mock_sub.unregister.assert_called_once()
        mock_pub.unregister.assert_called_once()


class TestROS1RobotCommands:
    """Test 4: ROS1Robot commands should work with dynamic types."""

    @pytest.fixture
    def connected_robot(self):
        """Create a connected robot with mocked rospy."""
        with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.ROS1_AVAILABLE", True):
            with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.rospy") as mock_ros:
                from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
                    ROS1Robot

                robot = ROS1Robot("test_id", "test_robot")
                asyncio.run(robot.connect())

                # Reset mock to clear init_node call
                mock_ros.reset_mock()

                yield robot, mock_ros

    @pytest.mark.asyncio
    async def test_get_topics_command(self):
        """Red: Should return list of topics."""
        with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.ROS1_AVAILABLE", True):
            with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.rospy") as mock_ros:
                from agent_ros_bridge.gateway_v2.connectors.ros1_connector import (
                    Command, ROS1Robot)

                mock_ros.get_published_topics.return_value = [
                    ("/topic1", "std_msgs/String"),
                    ("/topic2", "geometry_msgs/Twist"),
                ]

                robot = ROS1Robot("test_id", "test_robot")
                await robot.connect()

                result = await robot.execute(Command(action="get_topics", parameters={}))

                assert len(result) == 2
                assert result[0]["name"] == "/topic1"
                assert result[0]["type"] == "std_msgs/String"

    @pytest.mark.asyncio
    async def test_publish_with_auto_type_detection(self):
        """Red: Publish should auto-detect message type if not provided."""
        with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.ROS1_AVAILABLE", True):
            with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.rospy") as mock_ros:
                from agent_ros_bridge.gateway_v2.connectors.ros1_connector import (
                    Command, ROS1Robot)

                # Mock topic introspection
                mock_ros.get_published_topics.return_value = [
                    ("/cmd_vel", "geometry_msgs/Twist"),
                ]

                robot = ROS1Robot("test_id", "test_robot")
                await robot.connect()

                # Mock Publisher
                mock_publisher = Mock()
                mock_ros.Publisher.return_value = mock_publisher

                # Publish without specifying type
                result = await robot.execute(
                    Command(
                        action="publish",
                        parameters={"topic": "/cmd_vel", "data": {"linear": {"x": 1.0}}},
                    )
                )

                assert result["status"] == "published"
                assert result["topic"] == "/cmd_vel"
                mock_ros.Publisher.assert_called_once()

    @pytest.mark.asyncio
    async def test_subscribe_async_generator(self):
        """Red: Subscribe should return async generator."""
        with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.ROS1_AVAILABLE", True):
            with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.rospy"):
                from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
                    ROS1Robot

                robot = ROS1Robot("test_id", "test_robot")
                await robot.connect()

                # Subscribe should return an async generator
                subscription = robot.subscribe("/test_topic", msg_type="std_msgs/String")

                # Check it's an async generator
                assert hasattr(subscription, "__aiter__")


class TestROS1ConnectorDiscovery:
    """Test 5: ROS1Connector discovery should find robots."""

    @pytest.mark.asyncio
    async def test_connector_initialization(self):
        """Red: Connector should initialize with proper config."""
        from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
            ROS1Connector

        connector = ROS1Connector(master_uri="http://localhost:11311")

        assert connector.connector_type == "ros1"
        assert connector.master_uri == "http://localhost:11311"

    @pytest.mark.asyncio
    async def test_discover_returns_endpoints(self):
        """Red: Discover should return list of RobotEndpoints."""
        with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.ROS1_AVAILABLE", True):
            with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.rospy"):
                from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
                    ROS1Connector

                # Mock system state
                mock_master = Mock()
                mock_master.getPublishedTopics.return_value = (
                    1,
                    "ok",
                    [
                        ("/namespace1/topic1", "std_msgs/String"),
                        ("/namespace1/topic2", "geometry_msgs/Twist"),
                        ("/namespace2/topic1", "std_msgs/String"),
                    ],
                )
                mock_master.getNodeNames.return_value = (1, "ok", ["/node1", "/node2"])

                with patch("rosgraph.Master", return_value=mock_master):
                    connector = ROS1Connector()
                    endpoints = await connector.discover()

                    assert len(endpoints) > 0
                    assert all(ep.connector_type == "ros1" for ep in endpoints)

    @pytest.mark.asyncio
    async def test_connect_creates_robot(self):
        """Red: Connect should create and return ROS1Robot."""
        with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.ROS1_AVAILABLE", True):
            with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.rospy"):
                from agent_ros_bridge.gateway_v2.connectors.ros1_connector import (
                    ROS1Connector, ROS1Robot)

                connector = ROS1Connector()
                robot = await connector.connect("ros1:///test_namespace")

                assert isinstance(robot, ROS1Robot)
                assert robot.robot_id == "test_namespace"


class TestROS1MessageConversion:
    """Test 6: Message conversion should work bidirectionally."""

    def test_ros_msg_to_dict_with_slots(self):
        """Red: Should convert ROS __slots__ messages to dict."""
        from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
            ROS1Robot

        # Create a mock ROS message with __slots__
        class MockMsg:
            __slots__ = ["data", "header"]

            def __init__(self):
                self.data = "test_data"
                self.header = Mock()
                self.header.seq = 1
                self.header.stamp = Mock()
                self.header.stamp.secs = 123

        robot = ROS1Robot("test", "test")
        result = robot._ros_msg_to_dict(MockMsg())

        assert result["_type"] == "MockMsg"
        assert result["data"] == "test_data"

    def test_dict_to_ros_msg_simple(self):
        """Red: Should convert dict to simple ROS message."""
        from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
            ROS1Robot

        # Create a mock ROS message
        class MockMsg:
            def __init__(self):
                self.data = None

        robot = ROS1Robot("test", "test")
        msg = MockMsg()

        robot._dict_to_ros_msg({"data": "hello"}, msg)

        assert msg.data == "hello"

    def test_dict_to_ros_msg_nested(self):
        """Red: Should handle nested message structures."""
        from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
            ROS1Robot

        class InnerMsg:
            def __init__(self):
                self.x = 0.0
                self.y = 0.0

        class OuterMsg:
            def __init__(self):
                self.linear = InnerMsg()
                self.angular = InnerMsg()

        robot = ROS1Robot("test", "test")
        msg = OuterMsg()

        data = {"linear": {"x": 1.0, "y": 0.0}, "angular": {"x": 0.0, "y": 0.0}}

        robot._dict_to_ros_msg(data, msg)

        assert msg.linear.x == 1.0
        assert msg.linear.y == 0.0


class TestROS1ToolDiscoveryIntegration:
    """Test 7: ROS1Robot should integrate with ToolDiscovery."""

    @pytest.mark.asyncio
    async def test_get_topic_message_type_introspection(self):
        """Red: Should introspect ROS to get topic type."""
        with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.ROS1_AVAILABLE", True):
            with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.rospy") as mock_ros:
                from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
                    ROS1Robot

                mock_ros.get_published_topics.return_value = [
                    ("/cmd_vel", "geometry_msgs/Twist"),
                    ("/odom", "nav_msgs/Odometry"),
                ]

                robot = ROS1Robot("test", "test")

                msg_type = robot._get_topic_message_type("/cmd_vel")

                assert msg_type == "geometry_msgs/Twist"

    @pytest.mark.asyncio
    async def test_discover_topics_returns_formatted_list(self):
        """Red: _cmd_get_topics should return formatted topic list."""
        with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.ROS1_AVAILABLE", True):
            with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.rospy") as mock_ros:
                from agent_ros_bridge.gateway_v2.connectors.ros1_connector import \
                    ROS1Robot

                mock_ros.get_published_topics.return_value = [
                    ("/topic1", "std_msgs/String"),
                    ("/topic2", "geometry_msgs/Twist"),
                ]

                robot = ROS1Robot("test", "test")

                topics = robot._cmd_get_topics()

                assert len(topics) == 2
                assert topics[0]["name"] == "/topic1"
                assert topics[0]["type"] == "std_msgs/String"


class TestROS1ErrorHandling:
    """Test 8: ROS1Robot should handle errors gracefully."""

    @pytest.mark.asyncio
    async def test_publish_missing_topic(self):
        """Red: Publish should fail gracefully with missing topic."""
        with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.ROS1_AVAILABLE", True):
            with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.rospy"):
                from agent_ros_bridge.gateway_v2.connectors.ros1_connector import (
                    Command, ROS1Robot)

                robot = ROS1Robot("test", "test")
                await robot.connect()

                result = await robot.execute(
                    Command(action="publish", parameters={"data": "test"})  # Missing topic
                )

                assert result["status"] == "error"
                assert "Topic is required" in result["message"]

    @pytest.mark.asyncio
    async def test_publish_unknown_message_type(self):
        """Red: Publish should handle unknown message types."""
        with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.ROS1_AVAILABLE", True):
            with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.rospy"):
                from agent_ros_bridge.gateway_v2.connectors.ros1_connector import (
                    Command, ROS1Robot)

                robot = ROS1Robot("test", "test")
                await robot.connect()

                # Mock no topics available for auto-detection
                with patch.object(robot, "_get_topic_message_type", return_value=None):
                    result = await robot.execute(
                        Command(action="publish", parameters={"topic": "/test", "data": {}})
                    )

                    assert result["status"] == "error"
                    assert "Could not determine message type" in result["message"]

    @pytest.mark.asyncio
    async def test_execute_unknown_command(self):
        """Red: Execute should raise on unknown commands."""
        with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.ROS1_AVAILABLE", True):
            with patch("agent_ros_bridge.gateway_v2.connectors.ros1_connector.rospy"):
                from agent_ros_bridge.gateway_v2.connectors.ros1_connector import (
                    Command, ROS1Robot)

                robot = ROS1Robot("test", "test")
                await robot.connect()

                with pytest.raises(ValueError, match="Unknown ROS1 command"):
                    await robot.execute(Command(action="unknown_command", parameters={}))
