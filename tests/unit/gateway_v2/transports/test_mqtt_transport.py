"""Tests for MQTT transport module."""

import asyncio
import json
from datetime import UTC, datetime
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from agent_ros_bridge.gateway_v2.transports.mqtt_transport import MQTT_AVAILABLE, MQTTTransport


class TestMQTTTransportInit:
    """Test MQTTTransport initialization."""

    def test_default_init(self):
        """Test initialization with default config."""
        transport = MQTTTransport({})
        assert transport.broker_host == "localhost"
        assert transport.broker_port == 1883
        assert transport.username is None
        assert transport.password is None
        assert transport.tls_enabled is False
        assert transport.command_topic == "robots/commands"
        assert transport.telemetry_topic == "robots/telemetry"
        assert transport.event_topic == "robots/events"

    def test_custom_init(self):
        """Test initialization with custom config."""
        config = {
            "host": "mqtt.example.com",
            "port": 8883,
            "username": "user",
            "password": "pass",
            "client_id": "test_client",
            "tls": True,
            "tls_ca": "/path/to/ca.crt",
            "command_topic": "custom/commands",
        }
        transport = MQTTTransport(config)
        assert transport.broker_host == "mqtt.example.com"
        assert transport.broker_port == 8883
        assert transport.username == "user"
        assert transport.password == "pass"
        assert transport.client_id == "test_client"
        assert transport.tls_enabled is True
        assert transport.tls_ca == "/path/to/ca.crt"
        assert transport.command_topic == "custom/commands"

    def test_name_property(self):
        """Test transport name property."""
        transport = MQTTTransport({})
        assert transport.name == "mqtt"


class TestMQTTTransportCallbacks:
    """Test MQTT callback methods."""

    @pytest.fixture
    def transport(self):
        """Create transport fixture."""
        return MQTTTransport({})

    def test_on_connect_success(self, transport, caplog):
        """Test _on_connect with success."""
        with caplog.at_level("INFO"):
            transport._on_connect(None, None, None, 0)
            assert transport._connected is True
            assert "Connected to MQTT broker" in caplog.text

    def test_on_connect_failure(self, transport, caplog):
        """Test _on_connect with failure."""
        with caplog.at_level("ERROR"):
            transport._on_connect(None, None, None, 1)
            assert transport._connected is False
            assert "MQTT connection failed" in caplog.text

    def test_on_disconnect_normal(self, transport):
        """Test _on_disconnect with normal disconnect."""
        transport._connected = True
        transport._on_disconnect(None, None, 0)
        assert transport._connected is False

    def test_on_disconnect_unexpected(self, transport, caplog):
        """Test _on_disconnect with unexpected disconnect."""
        transport._connected = True
        with caplog.at_level("WARNING"):
            transport._on_disconnect(None, None, 1)
            assert transport._connected is False
            assert "Unexpected MQTT disconnection" in caplog.text


class TestMQTTToMessageConversion:
    """Test MQTT to Message conversion."""

    @pytest.fixture
    def transport(self):
        """Create transport fixture."""
        return MQTTTransport({})

    def test_mqtt_to_message_command(self, transport):
        """Test converting MQTT payload with command."""
        payload = {
            "id": "msg123",
            "timestamp": "2024-01-15T10:30:00",
            "command": {
                "action": "move",
                "parameters": {"x": 1.0, "y": 2.0},
                "timeout_ms": 10000,
                "priority": 3,
            },
        }
        message = transport._mqtt_to_message(payload, "robots/commands")
        assert message.header.message_id == "msg123"
        assert message.command is not None
        assert message.command.action == "move"
        assert message.command.parameters == {"x": 1.0, "y": 2.0}

    def test_mqtt_to_message_telemetry(self, transport):
        """Test converting MQTT payload with telemetry."""
        payload = {
            "id": "msg456",
            "timestamp": "2024-01-15T10:30:00",
            "telemetry": {
                "topic": "sensor/temp",
                "data": {"temperature": 25.5},
                "quality": 0.95,
            },
        }
        message = transport._mqtt_to_message(payload, "robots/telemetry")
        assert message.telemetry is not None
        assert message.telemetry.topic == "sensor/temp"
        assert message.telemetry.data == {"temperature": 25.5}
        assert message.telemetry.quality == 0.95

    def test_mqtt_to_message_event(self, transport):
        """Test converting MQTT payload with event."""
        payload = {
            "id": "msg789",
            "timestamp": "2024-01-15T10:30:00",
            "event": {
                "type": "obstacle_detected",
                "severity": "warning",
                "data": {"distance": 0.5},
            },
        }
        message = transport._mqtt_to_message(payload, "robots/events")
        assert message.event is not None
        assert message.event.event_type == "obstacle_detected"
        assert message.event.severity == "warning"

    def test_mqtt_to_message_no_timestamp(self, transport):
        """Test converting MQTT payload without timestamp."""
        payload = {"id": "msg000"}
        message = transport._mqtt_to_message(payload, "test/topic")
        assert message.header.message_id == "msg000"
        assert isinstance(message.header.timestamp, datetime)

    def test_mqtt_to_message_with_metadata(self, transport):
        """Test converting MQTT payload with metadata."""
        payload = {
            "id": "msg123",
            "timestamp": "2024-01-15T10:30:00",
            "metadata": {"robot_id": "r1", "zone": "kitchen"},
        }
        message = transport._mqtt_to_message(payload, "test/topic")
        assert message.metadata == {"robot_id": "r1", "zone": "kitchen"}


class TestMessageToMQTTConversion:
    """Test Message to MQTT conversion."""

    @pytest.fixture
    def transport(self):
        """Create transport fixture."""
        return MQTTTransport({})

    def test_message_to_mqtt_command(self, transport):
        """Test converting Message with command."""
        from agent_ros_bridge.gateway_v2.core import Command, Header, Message

        message = Message(
            header=Header(
                message_id="msg123",
                timestamp=datetime(2024, 1, 15, 10, 30, 0, tzinfo=UTC),
                source="test",
                target="robot",
            ),
            command=Command(
                action="move",
                parameters={"x": 1.0},
                timeout_ms=5000,
                priority=5,
            ),
        )
        payload = transport._message_to_mqtt(message)
        assert payload["id"] == "msg123"
        assert "command" in payload
        assert payload["command"]["action"] == "move"

    def test_message_to_mqtt_telemetry(self, transport):
        """Test converting Message with telemetry."""
        from agent_ros_bridge.gateway_v2.core import Header, Message, Telemetry

        message = Message(
            header=Header(
                message_id="msg456",
                timestamp=datetime(2024, 1, 15, 10, 30, 0, tzinfo=UTC),
                source="robot",
                target="bridge",
            ),
            telemetry=Telemetry(
                topic="sensor/battery",
                data={"level": 85},
                quality=0.95,
            ),
        )
        payload = transport._message_to_mqtt(message)
        assert "telemetry" in payload
        assert payload["telemetry"]["topic"] == "sensor/battery"

    def test_message_to_mqtt_event(self, transport):
        """Test converting Message with event."""
        from agent_ros_bridge.gateway_v2.core import Event, Header, Message

        message = Message(
            header=Header(
                message_id="msg789",
                timestamp=datetime(2024, 1, 15, 10, 30, 0, tzinfo=UTC),
                source="robot",
                target="bridge",
            ),
            event=Event(
                event_type="error",
                severity="error",
                data={"code": 500},
            ),
        )
        payload = transport._message_to_mqtt(message)
        assert "event" in payload
        assert payload["event"]["type"] == "error"


class TestMQTTMessageProcessing:
    """Test MQTT message processing."""

    @pytest.fixture
    def transport(self):
        """Create transport fixture."""
        t = MQTTTransport({})
        t.running = True
        return t

    @pytest.mark.asyncio
    async def test_process_messages_timeout(self, transport):
        """Test message processing with timeout."""
        transport.running = True

        # Run for a short time then stop
        async def stop_after_delay():
            await asyncio.sleep(0.15)
            transport.running = False

        asyncio.create_task(stop_after_delay())
        await transport._process_messages()
        # Should complete without error

    @pytest.mark.asyncio
    async def test_process_messages_with_handler(self, transport):
        """Test message processing with message handler."""
        from agent_ros_bridge.gateway_v2.core import Header, Identity, Message

        transport.running = True
        transport.message_handler = AsyncMock(return_value=None)

        # Add a message to the queue
        message = Message(
            header=Header(
                message_id="test",
                timestamp=datetime.now(UTC),
                source="test",
                target="bridge",
            ),
        )
        identity = Identity(id="test_id", name="test", roles=["test"])
        await transport._message_queue.put((message, identity))

        # Process one message then stop
        async def stop_after_delay():
            await asyncio.sleep(0.1)
            transport.running = False

        asyncio.create_task(stop_after_delay())
        await transport._process_messages()

        transport.message_handler.assert_called_once()


class TestMQTTSend:
    """Test MQTT send functionality."""

    @pytest.fixture
    def transport(self):
        """Create transport fixture."""
        t = MQTTTransport({})
        t._connected = True
        t.client = MagicMock()
        return t

    @pytest.mark.asyncio
    async def test_send_not_connected(self, transport):
        """Test send when not connected."""
        transport._connected = False
        from agent_ros_bridge.gateway_v2.core import Header, Message

        message = Message(
            header=Header(
                message_id="test",
                timestamp=datetime.now(UTC),
                source="test",
                target="bridge",
            ),
        )
        result = await transport.send(message, "recipient")
        assert result is False

    @pytest.mark.skipif(not MQTT_AVAILABLE, reason="paho-mqtt not installed")
    @pytest.mark.asyncio
    async def test_send_command(self, transport):
        """Test sending command message."""
        from agent_ros_bridge.gateway_v2.core import Command, Header, Message

        message = Message(
            header=Header(
                message_id="cmd123",
                timestamp=datetime.now(UTC),
                source="bridge",
                target="robot1",
            ),
            command=Command(action="move", parameters={}),
        )

        # Mock successful publish
        mock_result = MagicMock()
        mock_result.rc = 0
        transport.client.publish.return_value = mock_result

        result = await transport.send(message, "robot1")
        assert result is True

    @pytest.mark.skipif(not MQTT_AVAILABLE, reason="paho-mqtt not installed")
    @pytest.mark.asyncio
    async def test_send_telemetry(self, transport):
        """Test sending telemetry message."""
        from agent_ros_bridge.gateway_v2.core import Header, Message, Telemetry

        message = Message(
            header=Header(
                message_id="tel456",
                timestamp=datetime.now(UTC),
                source="robot1",
                target="bridge",
            ),
            telemetry=Telemetry(topic="battery", data={"level": 80}),
        )

        mock_result = MagicMock()
        mock_result.rc = 0
        transport.client.publish.return_value = mock_result

        result = await transport.send(message, "robot1")
        assert result is True

    @pytest.mark.skipif(not MQTT_AVAILABLE, reason="paho-mqtt not installed")
    @pytest.mark.asyncio
    async def test_send_publish_failure(self, transport):
        """Test send when publish fails."""
        from agent_ros_bridge.gateway_v2.core import Header, Message

        message = Message(
            header=Header(
                message_id="test",
                timestamp=datetime.now(UTC),
                source="test",
                target="bridge",
            ),
        )

        mock_result = MagicMock()
        mock_result.rc = 1  # Error code
        transport.client.publish.return_value = mock_result

        result = await transport.send(message, "recipient")
        assert result is False


class TestMQTTBroadcast:
    """Test MQTT broadcast functionality."""

    @pytest.fixture
    def transport(self):
        """Create transport fixture."""
        t = MQTTTransport({})
        t._connected = True
        t.client = MagicMock()
        t._identities = {"client1": MagicMock(), "client2": MagicMock()}
        return t

    @pytest.mark.skipif(not MQTT_AVAILABLE, reason="paho-mqtt not installed")
    @pytest.mark.asyncio
    async def test_broadcast_to_all(self, transport):
        """Test broadcasting to all clients."""
        from agent_ros_bridge.gateway_v2.core import Header, Message

        message = Message(
            header=Header(
                message_id="broadcast",
                timestamp=datetime.now(UTC),
                source="bridge",
                target="all",
            ),
        )

        mock_result = MagicMock()
        mock_result.rc = 0
        transport.client.publish.return_value = mock_result

        sent = await transport.broadcast(message)
        assert len(sent) == 2
        assert "client1" in sent
        assert "client2" in sent


class TestMQTTOnMessage:
    """Test MQTT on_message callback."""

    @pytest.fixture
    def transport(self):
        """Create transport fixture."""
        t = MQTTTransport({})
        t._loop = MagicMock()
        t._loop.is_running.return_value = True
        return t

    def test_on_message_valid_json(self, transport):
        """Test on_message with valid JSON."""
        mock_msg = MagicMock()
        mock_msg.topic = "robots/commands"
        mock_msg.payload = json.dumps({
            "id": "msg123",
            "timestamp": "2024-01-15T10:30:00",
            "command": {"action": "move"},
        }).encode()

        transport._on_message(None, None, mock_msg)

        # Identity should be created
        assert "mqtt_robots_commands" in transport._identities
        # Message should be queued
        transport._loop.call_soon_threadsafe.assert_called_once()

    def test_on_message_invalid_json(self, transport, caplog):
        """Test on_message with invalid JSON."""
        mock_msg = MagicMock()
        mock_msg.topic = "test/topic"
        mock_msg.payload = b"not valid json"

        with caplog.at_level("WARNING"):
            transport._on_message(None, None, mock_msg)
            assert "Invalid JSON" in caplog.text

    def test_on_message_no_loop(self, transport):
        """Test on_message when loop is not running."""
        transport._loop = None
        mock_msg = MagicMock()
        mock_msg.topic = "test/topic"
        mock_msg.payload = json.dumps({"id": "test"}).encode()

        # Should not raise
        transport._on_message(None, None, mock_msg)


class TestMQTTAvailable:
    """Test MQTT availability check."""

    def test_mqtt_available_constant(self):
        """Test MQTT_AVAILABLE constant."""
        # Should be True if paho-mqtt is installed
        assert isinstance(MQTT_AVAILABLE, bool)
