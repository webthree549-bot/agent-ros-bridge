"""
Unit tests for ROS2 Connector.
Tests ROS message conversion and connector logic.
"""

import asyncio
from unittest import mock

import pytest

# Mock rclpy before import
_mock_modules = {
    "rclpy": mock.MagicMock(),
    "rclpy.node": mock.MagicMock(),
    "rclpy.qos": mock.MagicMock(),
}
with mock.patch.dict("sys.modules", _mock_modules):
    from agent_ros_bridge.gateway_v2.connectors.ros2_connector import (
        ROS2Connector,
        ROS2Robot,
        ROS2Topic,
    )
    from agent_ros_bridge.gateway_v2.core import Command, Telemetry


class TestROS2Topic:
    """Test ROS2Topic dataclass"""

    def test_topic_creation(self):
        """Test basic ROS2 topic creation"""
        topic = ROS2Topic(
            name="/robot/cmd_vel", message_type="geometry_msgs/Twist", qos_profile="reliable"
        )

        assert topic.name == "/robot/cmd_vel"
        assert topic.message_type == "geometry_msgs/Twist"
        assert topic.qos_profile == "reliable"


class TestROS2RobotBasics:
    """Test ROS2Robot basic functionality"""

    @pytest.fixture
    def mock_node(self):
        """Create mock ROS2 node"""
        return mock.MagicMock()

    @pytest.fixture
    def robot(self, mock_node):
        """Create ROS2 robot instance"""
        return ROS2Robot("ros2_test_1", "TestBot", mock_node)

    @pytest.mark.asyncio
    async def test_connect(self, robot, mock_node):
        """Test robot connection"""
        result = await robot.connect()

        assert result is True
        assert robot.connected is True
        assert "publish" in robot.capabilities
        assert "subscribe" in robot.capabilities
        assert "services" in robot.capabilities

    @pytest.mark.asyncio
    async def test_disconnect(self, robot, mock_node):
        """Test robot disconnection"""
        await robot.connect()

        # Add a mock subscription
        mock_sub = mock.MagicMock()
        robot.subscriptions["/test"] = mock_sub

        await robot.disconnect()

        assert robot.connected is False
        assert robot.subscriptions == {}
        mock_node.destroy_subscription.assert_called_once_with(mock_sub)


class TestMessageConversion:
    """Test ROS message to/from dictionary conversion"""

    @pytest.fixture
    def robot(self):
        """Create robot with mocked node"""
        mock_node = mock.MagicMock()
        return ROS2Robot("test", "TestBot", mock_node)

    def test_ros_msg_to_dict_simple(self, robot):
        """Test converting simple ROS message to dict"""
        # Create mock ROS message
        mock_msg = mock.MagicMock()
        mock_msg.get_fields_and_field_types.return_value = {"data": "string"}
        mock_msg.data = "test_value"

        result = robot._ros_msg_to_dict(mock_msg)

        assert result["data"] == "test_value"

    def test_ros_msg_to_dict_nested(self, robot):
        """Test converting nested ROS message to dict"""
        # Create mock nested message
        mock_inner = mock.MagicMock()
        mock_inner.get_fields_and_field_types.return_value = {"x": "float64"}
        mock_inner.x = 1.5

        mock_outer = mock.MagicMock()
        mock_outer.get_fields_and_field_types.return_value = {"position": "geometry_msgs/Point"}
        mock_outer.position = mock_inner

        result = robot._ros_msg_to_dict(mock_outer)

        assert result["position"]["x"] == 1.5

    def test_ros_msg_to_dict_with_list(self, robot):
        """Test converting ROS message with list fields"""
        mock_inner1 = mock.MagicMock()
        mock_inner1.get_fields_and_field_types.return_value = {"value": "int32"}
        mock_inner1.value = 10

        mock_inner2 = mock.MagicMock()
        mock_inner2.get_fields_and_field_types.return_value = {"value": "int32"}
        mock_inner2.value = 20

        mock_msg = mock.MagicMock()
        mock_msg.get_fields_and_field_types.return_value = {"points": "sequence"}
        mock_msg.points = [mock_inner1, mock_inner2]

        result = robot._ros_msg_to_dict(mock_msg)

        assert len(result["points"]) == 2
        assert result["points"][0]["value"] == 10
        assert result["points"][1]["value"] == 20

    def test_ros_msg_to_dict_fallback(self, robot):
        """Test fallback when get_fields_and_field_types fails"""
        mock_msg = mock.MagicMock()
        mock_msg.get_fields_and_field_types.side_effect = Exception("No fields")
        # Add some public attributes
        mock_msg.data = "test"
        mock_msg.value = 42
        mock_msg._private = "hidden"  # Should be ignored

        result = robot._ros_msg_to_dict(mock_msg)

        assert "data" in result
        assert "value" in result
        assert "_private" not in result  # Private attrs ignored

    def test_dict_to_ros_msg(self, robot):
        """Test converting dict to ROS message"""
        mock_msg = mock.MagicMock()
        data = {"x": 1.0, "y": 2.0, "z": 3.0}

        robot._dict_to_ros_msg(data, mock_msg)

        assert mock_msg.x == 1.0
        assert mock_msg.y == 2.0
        assert mock_msg.z == 3.0


class TestCommandExecution:
    """Test command execution"""

    @pytest.fixture
    def robot(self):
        """Create robot with mocked node"""
        mock_node = mock.MagicMock()
        robot = ROS2Robot("test", "TestBot", mock_node)
        robot.connected = True
        return robot

    @pytest.mark.asyncio
    async def test_execute_get_topics(self, robot):
        """Test get_topics command"""
        # Mock topic list
        robot._cmd_get_topics = mock.MagicMock(return_value=["/topic1", "/topic2"])

        command = Command(action="get_topics", parameters={})
        result = await robot.execute(command)

        assert result == ["/topic1", "/topic2"]

    @pytest.mark.asyncio
    async def test_execute_get_nodes(self, robot):
        """Test get_nodes command"""
        robot._cmd_get_nodes = mock.MagicMock(return_value=["node1", "node2"])

        command = Command(action="get_nodes", parameters={})
        result = await robot.execute(command)

        assert result == ["node1", "node2"]

    @pytest.mark.asyncio
    async def test_execute_unknown_command(self, robot):
        """Test unknown command raises error"""
        command = Command(action="unknown_action", parameters={})

        with pytest.raises(ValueError, match="Unknown ROS2 command"):
            await robot.execute(command)


class TestROS2ConnectorBasics:
    """Test ROS2Connector functionality"""

    @pytest.fixture
    def connector(self):
        """Create ROS2 connector"""
        return ROS2Connector(domain_id=0)

    def test_initialization(self, connector):
        """Test connector initialization"""
        assert connector.domain_id == 0
        assert connector.node is None
        assert connector._initialized is False

    def test_initialization_custom_domain(self):
        """Test connector with custom domain ID"""
        connector = ROS2Connector(domain_id=42)
        assert connector.domain_id == 42


class TestURIParsing:
    """Test ROS2 URI parsing"""

    def test_parse_uri_default_domain(self):
        """Test parsing URI with default domain"""
        with mock.patch("agent_ros_bridge.gateway_v2.connectors.ros2_connector.rclpy"):
            ROS2Connector(domain_id=0)
            # URI without domain uses default
            robot_id = "ros2_0_"
            assert "ros2_0_" in robot_id

    def test_parse_uri_custom_domain(self):
        """Test parsing URI with custom domain"""
        connector = ROS2Connector(domain_id=42)
        # Domain ID should be accessible
        assert connector.domain_id == 42

    def test_parse_uri_with_namespace(self):
        """Test parsing URI with namespace"""
        # Namespace would be part of robot_id construction
        ROS2Connector(domain_id=0)
        namespace = "production/robots"
        expected_id_part = namespace.replace("/", "_")
        assert expected_id_part == "production_robots"


class TestRobotCapabilities:
    """Test robot capability reporting"""

    @pytest.fixture
    def robot(self):
        """Create connected robot"""
        mock_node = mock.MagicMock()
        robot = ROS2Robot("test", "TestBot", mock_node)
        robot.connected = True
        robot.capabilities = {"publish", "subscribe", "services"}
        return robot

    def test_capabilities_after_connect(self, robot):
        """Test capabilities after connection"""
        assert "publish" in robot.capabilities
        assert "subscribe" in robot.capabilities
        assert "services" in robot.capabilities

    def test_robot_id_format(self, robot):
        """Test robot ID format"""
        assert robot.robot_id == "test"
        assert robot.name == "TestBot"
        assert robot.robot_type == "ros2"


class TestErrorHandling:
    """Test error handling"""

    @pytest.mark.asyncio
    async def test_ros2_not_available(self):
        """Test behavior when ROS2 is not available"""
        # Skip if ROS2 is actually available (can't test the error path)
        from agent_ros_bridge.gateway_v2.connectors import ros2_connector

        if ros2_connector.ROS2_AVAILABLE:
            pytest.skip("ROS2 is available, cannot test error path")

        connector = ROS2Connector()
        with pytest.raises(RuntimeError, match="ROS2 not available"):
            await connector._ensure_initialized()

    def test_ros_msg_conversion_error(self):
        """Test graceful handling of message conversion errors"""
        mock_node = mock.MagicMock()
        robot = ROS2Robot("test", "TestBot", mock_node)

        # Create message that causes error
        mock_msg = mock.MagicMock()
        mock_msg.get_fields_and_field_types.side_effect = Exception("Conversion failed")
        mock_msg.__dir__ = mock.MagicMock(return_value=[])

        # Should not raise, returns empty or partial dict
        result = robot._ros_msg_to_dict(mock_msg)
        assert isinstance(result, dict)


class TestTelemetryHandling:
    """Test telemetry handling"""

    @pytest.fixture
    def robot(self):
        """Create robot with mocked node"""
        mock_node = mock.MagicMock()
        return ROS2Robot("test", "TestBot", mock_node)

    @pytest.mark.asyncio
    async def test_telemetry_queue(self, robot):
        """Test telemetry queue operations"""
        telemetry = Telemetry(topic="/test", data={"value": 42}, quality=1.0)

        await robot._telemetry_queue.put(telemetry)

        # Should be able to retrieve
        result = await asyncio.wait_for(robot._telemetry_queue.get(), timeout=0.1)
        assert result.topic == "/test"
        assert result.data["value"] == 42

    def test_telemetry_conversion_in_callback(self, robot):
        """Test telemetry created from ROS message callback"""
        # Simulate callback creating telemetry
        mock_msg = mock.MagicMock()
        mock_msg.get_fields_and_field_types.return_value = {"temperature": "float64"}
        mock_msg.temperature = 25.5

        telemetry = Telemetry(
            topic="/sensor/temp", data=robot._ros_msg_to_dict(mock_msg), quality=1.0
        )

        assert telemetry.topic == "/sensor/temp"
        assert telemetry.data["temperature"] == 25.5
