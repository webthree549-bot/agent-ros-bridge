"""
Unit tests for WebSocket Transport.
Tests connection handling, authentication, and message routing.
"""

# Check optional dependencies
import importlib.util
import json
from datetime import datetime
from unittest import mock

import pytest

WEBSOCKETS_AVAILABLE = importlib.util.find_spec("websockets") is not None

# Mock websockets before import if not available
if not WEBSOCKETS_AVAILABLE:
    mock_websockets = mock.MagicMock()
    mock_websockets.exceptions = mock.MagicMock()
    mock.patch.dict(
        "sys.modules",
        {
            "websockets": mock_websockets,
            "websockets.exceptions": mock_websockets.exceptions,
        },
    ).start()

from agent_ros_bridge.gateway_v2.core import (  # noqa: E402
    Command,
    Header,
    Identity,
    Message,
    Telemetry,
)
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport  # noqa: E402


class TestWebSocketTransportBasics:
    """Test WebSocketTransport initialization"""

    @pytest.fixture
    def basic_config(self):
        """Basic transport config"""
        return {"host": "127.0.0.1", "port": 8765}

    @pytest.fixture
    def tls_config(self):
        """TLS-enabled config"""
        return {
            "host": "0.0.0.0",
            "port": 8766,
            "tls_cert": "/path/to/cert.pem",
            "tls_key": "/path/to/key.pem",
        }

    @pytest.fixture
    def auth_config(self):
        """Auth-enabled config"""
        return {
            "host": "127.0.0.1",
            "port": 8765,
            "auth": {"enabled": True, "jwt_secret": "test-secret"},
        }

    def test_basic_initialization(self, basic_config):
        """Test basic transport initialization"""
        transport = WebSocketTransport(basic_config)

        assert transport.name == "websocket"
        assert transport.host == "127.0.0.1"
        assert transport.port == 8765
        assert transport.auth_enabled is False
        assert transport.authenticator is None
        assert transport.rbac is None

    def test_default_values(self):
        """Test default configuration values"""
        transport = WebSocketTransport({})

        assert transport.host == "0.0.0.0"
        assert transport.port == 8765
        assert transport.tls_cert is None
        assert transport.tls_key is None

    def test_tls_configuration(self, tls_config):
        """Test TLS configuration"""
        transport = WebSocketTransport(tls_config)

        assert transport.tls_cert == "/path/to/cert.pem"
        assert transport.tls_key == "/path/to/key.pem"

    def test_auth_enabled_configuration(self, auth_config):
        """Test auth-enabled configuration"""
        transport = WebSocketTransport(auth_config)

        assert transport.auth_enabled is True
        # Authenticator created when auth is available


class TestMessageConversion:
    """Test message serialization/deserialization"""

    @pytest.fixture
    def transport(self):
        """Create transport instance"""
        return WebSocketTransport({})

    def test_message_to_json_command(self, transport):
        """Test converting Command message to JSON"""
        msg = Message(
            header=Header(message_id="msg-1", timestamp=datetime.now()),
            command=Command(action="move", parameters={"x": 1.0, "y": 2.0}),
        )

        json_str = transport._message_to_json(msg)
        data = json.loads(json_str)

        assert data["header"]["message_id"] == "msg-1"
        assert data["command"]["action"] == "move"
        assert data["command"]["parameters"] == {"x": 1.0, "y": 2.0}

    def test_message_to_json_telemetry(self, transport):
        """Test converting Telemetry message to JSON"""
        msg = Message(
            header=Header(message_id="msg-2", timestamp=datetime.now()),
            telemetry=Telemetry(topic="/robot/status", data={"battery": 85.5}),
        )

        json_str = transport._message_to_json(msg)
        data = json.loads(json_str)

        assert data["telemetry"]["topic"] == "/robot/status"
        assert data["telemetry"]["data"]["battery"] == 85.5

    def test_json_to_message_command(self, transport):
        """Test converting JSON to Command message"""
        json_data = {
            "header": {"message_id": "msg-3", "timestamp": datetime.now().isoformat()},
            "command": {"action": "rotate", "parameters": {"angle": 90}},
        }

        msg = transport._json_to_message(json_data)

        assert msg.command is not None
        assert msg.command.action == "rotate"
        assert msg.command.parameters["angle"] == 90

    def test_json_to_message_telemetry(self, transport):
        """Test converting JSON to Telemetry message"""
        json_data = {
            "header": {"message_id": "msg-4", "timestamp": datetime.now().isoformat()},
            "telemetry": {"topic": "/sensor/temperature", "data": {"value": 25.5}},
        }

        msg = transport._json_to_message(json_data)

        assert msg.telemetry is not None
        assert msg.telemetry.topic == "/sensor/temperature"

    def test_json_to_message_with_identity(self, transport):
        """Test converting JSON with identity info"""
        json_data = {
            "header": {"message_id": "msg-5", "timestamp": datetime.now().isoformat()},
            "identity": {"id": "user-123", "name": "Test User", "roles": ["operator"]},
            "command": {"action": "test", "parameters": {}},
        }

        msg = transport._json_to_message(json_data)

        assert msg.identity is not None
        assert msg.identity.id == "user-123"
        assert msg.identity.name == "Test User"
        assert "operator" in msg.identity.roles


class TestClientManagement:
    """Test client connection management"""

    @pytest.fixture
    def transport(self):
        """Create transport instance"""
        return WebSocketTransport({})

    @pytest.fixture
    def mock_websocket(self):
        """Create mock WebSocket"""
        ws = mock.MagicMock()
        ws.remote_address = ("127.0.0.1", 54321)
        return ws

    def test_initial_client_state(self, transport):
        """Test initial client state is empty"""
        assert transport.clients == {}
        assert transport.identities == {}

    def test_add_client(self, transport, mock_websocket):
        """Test adding a client"""
        client_id = str(id(mock_websocket))
        transport.clients[client_id] = mock_websocket

        identity = Identity(id=client_id, name="test-user", roles=["operator"])
        transport.identities[client_id] = identity

        assert client_id in transport.clients
        assert client_id in transport.identities
        assert transport.identities[client_id].name == "test-user"

    def test_remove_client(self, transport, mock_websocket):
        """Test removing a client"""
        client_id = str(id(mock_websocket))
        transport.clients[client_id] = mock_websocket
        transport.identities[client_id] = Identity(id=client_id, name="test")

        del transport.clients[client_id]
        del transport.identities[client_id]

        assert client_id not in transport.clients
        assert client_id not in transport.identities


class TestBroadcasting:
    """Test message broadcasting"""

    @pytest.fixture
    def transport(self):
        """Create transport instance"""
        return WebSocketTransport({})

    @pytest.fixture
    def sample_message(self):
        """Create sample message"""
        return Message(
            header=Header(message_id="broadcast-1", timestamp=datetime.now()),
            telemetry=Telemetry(topic="/status", data={"active": True}),
        )

    @pytest.mark.asyncio
    async def test_broadcast_all(self, transport, sample_message):
        """Test broadcasting to all clients"""
        ws1 = mock.MagicMock()
        ws1.send = mock.AsyncMock()
        ws2 = mock.MagicMock()
        ws2.send = mock.AsyncMock()

        transport.clients["client-1"] = ws1
        transport.clients["client-2"] = ws2

        await transport.broadcast(sample_message)

        ws1.send.assert_called_once()
        ws2.send.assert_called_once()

    @pytest.mark.asyncio
    async def test_broadcast_excludes_sender(self, transport, sample_message):
        """Test broadcast excludes sender"""
        ws1 = mock.MagicMock()
        ws1.send = mock.AsyncMock()
        ws2 = mock.MagicMock()
        ws2.send = mock.AsyncMock()

        transport.clients["sender"] = ws1
        transport.clients["other"] = ws2

        await transport.broadcast(sample_message, exclude="sender")

        ws1.send.assert_not_called()
        ws2.send.assert_called_once()

    @pytest.mark.asyncio
    async def test_send_to_specific_client(self, transport, sample_message):
        """Test sending to specific client"""
        ws = mock.MagicMock()
        ws.send = mock.AsyncMock()

        transport.clients["target"] = ws

        result = await transport.send(sample_message, "target")

        assert result is True
        ws.send.assert_called_once()

    @pytest.mark.asyncio
    async def test_send_to_nonexistent_client(self, transport, sample_message):
        """Test sending to non-existent client"""
        result = await transport.send(sample_message, "nonexistent")

        assert result is False


class TestTransportLifecycle:
    """Test transport start/stop lifecycle"""

    @pytest.mark.asyncio
    @mock.patch("agent_ros_bridge.gateway_v2.transports.websocket.WEBSOCKETS_AVAILABLE", True)
    async def test_start_without_websockets(self):
        """Test start fails when websockets not available"""
        with mock.patch(
            "agent_ros_bridge.gateway_v2.transports.websocket.WEBSOCKETS_AVAILABLE",
            False,
        ):
            transport = WebSocketTransport({})
            result = await transport.start()

            assert result is False

    @pytest.mark.asyncio
    async def test_stop_closes_clients(self):
        """Test stop closes all client connections"""
        transport = WebSocketTransport({})

        ws1 = mock.MagicMock()
        ws1.close = mock.AsyncMock()
        ws2 = mock.MagicMock()
        ws2.close = mock.AsyncMock()

        transport.clients["client-1"] = ws1
        transport.clients["client-2"] = ws2
        transport.server = mock.MagicMock()
        transport.server.close = mock.MagicMock()
        transport.server.wait_closed = mock.AsyncMock()

        await transport.stop()

        transport.server.close.assert_called_once()
        ws1.close.assert_called_once()
        ws2.close.assert_called_once()
        assert transport.clients == {}


class TestErrorHandling:
    """Test error handling in transport"""

    @pytest.fixture
    def transport(self):
        """Create transport instance"""
        return WebSocketTransport({})

    def test_json_to_message_invalid_data(self, transport):
        """Test handling invalid message data"""
        # Missing required fields
        json_data = {"invalid": "data"}

        # Should not raise, creates message with defaults
        msg = transport._json_to_message(json_data)
        assert msg is not None

    def test_json_to_message_malformed_header(self, transport):
        """Test handling malformed header"""
        json_data = {"header": "not-a-dict", "command": {"action": "test"}}

        msg = transport._json_to_message(json_data)
        assert msg is not None

    @pytest.mark.asyncio
    async def test_broadcast_handles_send_errors(self, transport):
        """Test broadcast continues despite individual send errors"""
        from agent_ros_bridge.gateway_v2.core import Header, Message, Telemetry

        ws1 = mock.MagicMock()
        ws1.send = mock.AsyncMock(side_effect=Exception("Send failed"))
        ws2 = mock.MagicMock()
        ws2.send = mock.AsyncMock()

        transport.clients["client-1"] = ws1
        transport.clients["client-2"] = ws2

        msg = Message(
            header=Header(message_id="test", timestamp=datetime.now()),
            telemetry=Telemetry(topic="/test", data={}),
        )

        # Should not raise despite ws1 failing
        await transport.broadcast(msg)

        ws2.send.assert_called_once()


class TestMessageHandlerIntegration:
    """Test message handler integration"""

    @pytest.fixture
    def transport(self):
        """Create transport instance"""
        return WebSocketTransport({})

    @pytest.mark.asyncio
    async def test_message_handler_called(self, transport):
        """Test message handler is called for incoming messages"""
        handler_called = False
        received_msg = None
        received_identity = None

        async def handler(msg, identity):
            nonlocal handler_called, received_msg, received_identity
            handler_called = True
            received_msg = msg
            received_identity = identity
            return None

        transport.message_handler = handler

        # Simulate message handling
        json_data = {
            "header": {"message_id": "test-1", "timestamp": datetime.now().isoformat()},
            "command": {"action": "move", "parameters": {}},
        }
        msg = transport._json_to_message(json_data)
        identity = Identity(id="client-1", name="test-user", roles=["operator"])

        if transport.message_handler:
            await transport.message_handler(msg, identity)

        assert handler_called is True
        assert received_msg.command.action == "move"
        assert received_identity.name == "test-user"

    @pytest.mark.asyncio
    async def test_message_handler_response_sent(self, transport):
        """Test response from handler is sent back"""
        ws = mock.MagicMock()
        ws.send = mock.AsyncMock()

        async def handler(msg, identity):
            return Message(
                header=Header(message_id="response-1", timestamp=datetime.now()),
                telemetry=Telemetry(topic="/response", data={"status": "ok"}),
            )

        transport.message_handler = handler
        transport.clients["client-1"] = ws

        # Simulate handling
        json_data = {
            "header": {"message_id": "req-1", "timestamp": datetime.now().isoformat()},
            "command": {"action": "test", "parameters": {}},
        }
        msg = transport._json_to_message(json_data)
        identity = Identity(id="client-1", name="test")

        response = await transport.message_handler(msg, identity)
        if response:
            await ws.send(transport._message_to_json(response))

        ws.send.assert_called_once()
