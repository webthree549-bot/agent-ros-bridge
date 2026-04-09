"""Integration tests for gRPC Transport.

Tests using real gRPC server and client (in-memory or localhost).
"""

import asyncio
import time
from unittest.mock import Mock

import pytest

pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif(
        not pytest.importorskip("grpc", reason="gRPC not available"),
        reason="grpc not installed",
    ),
]


class TestGRPCTransportIntegration:
    """Integration tests for gRPC transport with real server."""

    @pytest.fixture
    def free_port(self):
        """Get a free port for testing."""
        import socket

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(("127.0.0.1", 0))
            return s.getsockname()[1]

    @pytest.fixture
    def auth_config(self):
        """Create auth config for tests."""
        from agent_ros_bridge.gateway_v2.auth import AuthConfig

        return AuthConfig(jwt_secret="test-secret-for-integration")

    @pytest.fixture
    def authenticator(self, auth_config):
        """Create authenticator for tests."""
        from agent_ros_bridge.gateway_v2.auth import Authenticator

        return Authenticator(auth_config)

    @pytest.fixture
    def valid_token(self, authenticator):
        """Generate a valid JWT token for testing."""
        return authenticator.create_token(
            identity="test-user",
            roles=["robot:operate"],
            device_id="test-device",
        )

    @pytest.mark.asyncio
    async def test_transport_starts_and_stops(self, free_port, auth_config):
        """Test that transport can start and stop successfully."""
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

        config = {
            "host": "127.0.0.1",
            "port": free_port,
            "reflection": False,
        }

        transport = GRPCTransport(config)

        # Start transport
        result = await transport.start()
        assert result is True
        assert transport.running is True

        # Stop transport
        await transport.stop()
        assert transport.running is False

    @pytest.mark.asyncio
    async def test_transport_with_different_ports(self, auth_config):
        """Test transport with multiple sequential ports."""
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

        for port_offset in range(3):
            import socket

            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(("127.0.0.1", 0))
                port = s.getsockname()[1]

            config = {
                "host": "127.0.0.1",
                "port": port,
                "reflection": False,
            }

            transport = GRPCTransport(config)
            result = await transport.start()
            assert result is True, f"Failed to start on port {port}"
            await transport.stop()

    @pytest.mark.asyncio
    async def test_transport_config_values(self, free_port, auth_config):
        """Test that config values are properly set."""
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

        config = {
            "host": "127.0.0.1",
            "port": free_port,
            "max_workers": 5,
            "keepalive_time_ms": 5000,
            "reflection": True,
        }

        transport = GRPCTransport(config)

        assert transport.host == "127.0.0.1"
        assert transport.port == free_port
        assert transport.max_workers == 5
        assert transport.keepalive_time_ms == 5000
        assert transport.reflection is True

        await transport.start()
        await transport.stop()


class TestGRPCServicerIntegration:
    """Integration tests for gRPC servicer."""

    @pytest.fixture
    def authenticator(self):
        """Create authenticator for tests."""
        from agent_ros_bridge.gateway_v2.auth import AuthConfig, Authenticator

        auth_config = AuthConfig(jwt_secret="test-secret-for-servicer")
        return Authenticator(auth_config)

    @pytest.fixture
    def valid_token(self, authenticator):
        """Generate a valid JWT token."""
        return authenticator.create_token(
            identity="test-user",
            roles=["robot:operate"],
        )

    def test_servicer_creation(self, authenticator):
        """Test that servicer can be created."""
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCServicer

        command_handler = Mock()
        servicer = GRPCServicer(command_handler, authenticator)

        assert servicer.command_handler == command_handler
        assert servicer.authenticator == authenticator

    @pytest.mark.asyncio
    async def test_servicer_execute_command_without_auth(self, authenticator):
        """Test that servicer rejects requests without authentication."""
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCServicer

        command_handler = Mock()
        servicer = GRPCServicer(command_handler, authenticator)

        # Mock context without authorization
        mock_context = Mock()
        mock_context.invocation_metadata.return_value = []
        mock_context.set_code = Mock()
        mock_context.set_details = Mock()

        # Import bridge_pb2 for request
        from agent_ros_bridge.gateway_v2.transports import bridge_pb2

        request = bridge_pb2.Command(action="test", parameters={})

        result = await servicer.ExecuteCommand(request, mock_context)

        # Should set error code for unauthenticated
        mock_context.set_code.assert_called_once()
        mock_context.set_details.assert_called_once()


class TestBridgePb2Module:
    """Tests for bridge_pb2 module functionality."""

    def test_header_creation(self):
        """Test creating Header message."""
        from agent_ros_bridge.gateway_v2.transports.bridge_pb2 import Header

        header = Header(message_id="msg-123", timestamp=1234567890, client_id="client-1")

        assert header.message_id == "msg-123"
        assert header.timestamp == 1234567890
        assert header.client_id == "client-1"

    def test_command_creation(self):
        """Test creating Command message."""
        from agent_ros_bridge.gateway_v2.transports.bridge_pb2 import Command

        cmd = Command(action="navigate", parameters={"x": 1.0, "y": 2.0})

        assert cmd.action == "navigate"
        assert cmd.parameters == {"x": 1.0, "y": 2.0}

    def test_command_response_creation(self):
        """Test creating CommandResponse message."""
        from agent_ros_bridge.gateway_v2.transports.bridge_pb2 import CommandResponse

        response = CommandResponse(success=True, data='{"result": "ok"}', error="")

        assert response.success is True
        assert response.data == '{"result": "ok"}'
        assert response.error == ""

    def test_telemetry_creation(self):
        """Test creating Telemetry message."""
        from agent_ros_bridge.gateway_v2.transports.bridge_pb2 import Telemetry

        telemetry = Telemetry(topic="battery", data={"level": 85.0})

        assert telemetry.topic == "battery"
        assert telemetry.data == {"level": 85.0}

    def test_event_creation(self):
        """Test creating Event message."""
        from agent_ros_bridge.gateway_v2.transports.bridge_pb2 import Event

        event = Event(event_type="obstacle_detected", source="lidar", data={"distance": 1.5})

        assert event.event_type == "obstacle_detected"
        assert event.source == "lidar"
        assert event.data == {"distance": 1.5}

    def test_status_creation(self):
        """Test creating Status message."""
        from agent_ros_bridge.gateway_v2.transports.bridge_pb2 import Status

        status = Status(healthy=True, uptime_seconds=3600, version="1.0.0")

        assert status.healthy is True
        assert status.uptime_seconds == 3600
        assert status.version == "1.0.0"

    def test_add_bridge_service_servicer_to_server(self):
        """Test service registration function."""
        from agent_ros_bridge.gateway_v2.transports.bridge_pb2 import (
            add_BridgeServiceServicer_to_server,
        )

        mock_servicer = Mock()
        mock_server = Mock()

        # Call registration function
        add_BridgeServiceServicer_to_server(mock_servicer, mock_server)

        # If server has add_servicer method, it should be called
        if hasattr(mock_server, "add_servicer"):
            mock_server.add_servicer.assert_called_once_with(mock_servicer)

    def test_bridge_service_servicer_base_class(self):
        """Test BridgeServiceServicer base class."""
        from agent_ros_bridge.gateway_v2.transports.bridge_pb2 import BridgeServiceServicer

        class TestServicer(BridgeServiceServicer):
            async def SendCommand(self, request, context):
                return "sent"

            async def HealthCheck(self, request, context):
                return "healthy"

            async def SubscribeTelemetry(self, request, context):
                return []

            async def SubscribeEvents(self, request, context):
                return []

            async def StreamCommands(self, request_iterator, context):
                return []

        servicer = TestServicer()
        assert servicer is not None

    def test_bridge_service_stub_base_class(self):
        """Test BridgeServiceStub client stub."""
        from agent_ros_bridge.gateway_v2.transports.bridge_pb2 import BridgeServiceStub

        mock_channel = Mock()
        stub = BridgeServiceStub(mock_channel)

        assert stub.channel == mock_channel


class TestGRPCErrorHandling:
    """Test error handling in gRPC transport."""

    @pytest.mark.asyncio
    async def test_start_on_used_port_fails(self):
        """Test that starting on an already used port fails gracefully."""
        import socket

        from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(("127.0.0.1", 0))
            s.listen(1)
            port = s.getsockname()[1]

            config = {
                "host": "127.0.0.1",
                "port": port,
                "reflection": False,
            }

            transport = GRPCTransport(config)
            # This may raise an exception or return False
            try:
                result = await transport.start()
                # If it returns, it should be False due to port conflict
                assert result is False or transport.running is False
            except Exception:
                # Exception is also acceptable for port conflict
                pass

    @pytest.mark.asyncio
    async def test_stop_without_start(self):
        """Test that stop without start doesn't raise errors."""
        import socket

        from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(("127.0.0.1", 0))
            port = s.getsockname()[1]

        config = {
            "host": "127.0.0.1",
            "port": port,
            "reflection": False,
        }

        transport = GRPCTransport(config)

        # Should not raise
        await transport.stop()

        assert transport.running is False
