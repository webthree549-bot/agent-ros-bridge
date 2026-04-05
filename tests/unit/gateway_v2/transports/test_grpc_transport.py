"""Tests for gRPC transport module."""

import logging
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from agent_ros_bridge.gateway_v2.transports.grpc_transport import (
    GRPC_AVAILABLE,
    GRPCClient,
    GRPCServicer,
    GRPCTransport,
)


class TestGRPCAvailable:
    """Test gRPC availability constant."""

    def test_grpc_available_is_bool(self):
        """Test GRPC_AVAILABLE is a boolean."""
        assert isinstance(GRPC_AVAILABLE, bool)


class TestGRPCTransportInit:
    """Test GRPCTransport initialization."""

    def test_default_init(self):
        """Test initialization with default config."""
        transport = GRPCTransport()
        assert transport.name == "grpc"
        assert transport.host == "0.0.0.0"
        assert transport.port == 50051
        assert transport.max_workers == 10
        assert transport.reflection is True
        assert transport.keepalive_time_ms == 10000
        assert transport.tls_cert is None
        assert transport.tls_key is None
        assert transport.ca_cert is None

    def test_custom_init(self):
        """Test initialization with custom config."""
        config = {
            "host": "127.0.0.1",
            "port": 50052,
            "max_workers": 20,
            "reflection": False,
            "keepalive_time_ms": 5000,
            "tls_cert": "/path/to/cert.pem",
            "tls_key": "/path/to/key.pem",
            "ca_cert": "/path/to/ca.pem",
            "jwt_secret": "my-secret",
        }
        transport = GRPCTransport(config)
        assert transport.host == "127.0.0.1"
        assert transport.port == 50052
        assert transport.max_workers == 20
        assert transport.reflection is False
        assert transport.keepalive_time_ms == 5000
        assert transport.tls_cert == "/path/to/cert.pem"
        assert transport.tls_key == "/path/to/key.pem"
        assert transport.ca_cert == "/path/to/ca.pem"

    def test_transport_type(self):
        """Test transport_type class attribute."""
        assert GRPCTransport.transport_type == "grpc"


class TestGRPCTransportStart:
    """Test GRPCTransport start method."""

    @pytest.fixture
    def transport(self):
        """Create transport fixture."""
        return GRPCTransport()

    @pytest.mark.skipif(GRPC_AVAILABLE, reason="Test only when gRPC not available")
    @pytest.mark.asyncio
    async def test_start_grpc_not_available(self, transport, caplog):
        """Test start when gRPC not available."""
        with caplog.at_level("ERROR"):
            result = await transport.start()
            assert result is False
            assert "gRPC not available" in caplog.text

    @pytest.mark.skipif(not GRPC_AVAILABLE, reason="gRPC not installed")
    @pytest.mark.asyncio
    async def test_start_success(self, transport, caplog):
        """Test successful start."""
        with caplog.at_level("INFO"):
            # Mock the server
            mock_server = AsyncMock()
            mock_server.add_insecure_port = MagicMock()
            mock_server.start = AsyncMock()

            with patch("grpc.aio.server", return_value=mock_server):
                result = await transport.start()
                assert result is True
                assert transport.running is True
                assert "gRPC server started" in caplog.text


class TestGRPCTransportStop:
    """Test GRPCTransport stop method."""

    @pytest.fixture
    def transport(self):
        """Create transport fixture."""
        t = GRPCTransport()
        t.server = AsyncMock()
        t.running = True
        return t

    @pytest.mark.asyncio
    async def test_stop_with_server(self, transport, caplog):
        """Test stop with server running."""
        with caplog.at_level("INFO"):
            await transport.stop()
            transport.server.stop.assert_called_once_with(grace_period=5)
            assert transport.running is False
            assert "gRPC server stopped" in caplog.text

    @pytest.mark.asyncio
    async def test_stop_without_server(self, transport):
        """Test stop without server."""
        transport.server = None
        await transport.stop()
        assert transport.running is False


class TestGRPCTransportSend:
    """Test GRPCTransport send method."""

    @pytest.fixture
    def transport(self):
        """Create transport fixture."""
        return GRPCTransport()

    @pytest.mark.asyncio
    async def test_send_not_implemented(self, transport, caplog):
        """Test send returns False (not implemented)."""
        with caplog.at_level("WARNING"):
            result = await transport.send({"test": "message"}, "recipient")
            assert result is False
            assert "gRPC send not implemented" in caplog.text


class TestGRPCTransportBroadcast:
    """Test GRPCTransport broadcast method."""

    @pytest.fixture
    def transport(self):
        """Create transport fixture."""
        return GRPCTransport()

    @pytest.mark.asyncio
    async def test_broadcast_not_implemented(self, transport, caplog):
        """Test broadcast returns empty list (not implemented)."""
        with caplog.at_level("WARNING"):
            result = await transport.broadcast({"test": "message"})
            assert result == []
            assert "gRPC broadcast not implemented" in caplog.text


class TestGRPCTransportHandleCommand:
    """Test GRPCTransport handle_command method."""

    @pytest.fixture
    def transport(self):
        """Create transport fixture."""
        return GRPCTransport()

    @pytest.mark.asyncio
    async def test_handle_command(self, transport, caplog):
        """Test handle_command."""
        with caplog.at_level("DEBUG"):
            result = await transport.handle_command({"action": "move", "parameters": {"x": 1.0}})
            assert result["success"] is True
            assert "data" in result


class TestGRPCServicer:
    """Test GRPCServicer class."""

    @pytest.fixture
    def servicer(self):
        """Create servicer fixture."""
        command_handler = AsyncMock()
        authenticator = MagicMock()
        authenticator.verify_token.return_value = {"sub": "user123"}
        return GRPCServicer(command_handler, authenticator)

    @pytest.mark.skipif(not GRPC_AVAILABLE, reason="gRPC not installed")
    @pytest.mark.asyncio
    async def test_execute_command_missing_token(self, servicer):
        """Test ExecuteCommand with missing token."""
        mock_context = MagicMock()
        mock_context.invocation_metadata.return_value = []

        await servicer.ExecuteCommand(None, mock_context)

        mock_context.set_code.assert_called_once()
        mock_context.set_details.assert_called_with("Missing authentication token")

    @pytest.mark.skipif(not GRPC_AVAILABLE, reason="gRPC not installed")
    @pytest.mark.asyncio
    async def test_execute_command_invalid_token(self, servicer):
        """Test ExecuteCommand with invalid token."""
        mock_context = MagicMock()
        mock_context.invocation_metadata.return_value = [("authorization", "Bearer invalid")]
        servicer.authenticator.verify_token.side_effect = Exception("Invalid token")

        await servicer.ExecuteCommand(None, mock_context)

        mock_context.set_code.assert_called_once()
        assert "Invalid token" in mock_context.set_details.call_args[0][0]

    @pytest.mark.skipif(not GRPC_AVAILABLE, reason="gRPC not installed")
    @pytest.mark.asyncio
    async def test_execute_command_success(self, servicer):
        """Test ExecuteCommand with valid token."""
        mock_request = MagicMock()
        mock_request.action = "move"
        mock_request.parameters = {"x": 1.0}

        mock_context = MagicMock()
        mock_context.invocation_metadata.return_value = [("authorization", "Bearer valid_token")]

        await servicer.ExecuteCommand(mock_request, mock_context)

        servicer.command_handler.assert_called_once()


class TestGRPCClient:
    """Test GRPCClient class."""

    def test_default_init(self):
        """Test default initialization."""
        client = GRPCClient()
        assert client.host == "localhost"
        assert client.port == 50051
        assert client.token == ""
        assert client.channel is None

    def test_custom_init(self):
        """Test custom initialization."""
        client = GRPCClient(host="example.com", port=50052, token="my_token")
        assert client.host == "example.com"
        assert client.port == 50052
        assert client.token == "my_token"

    @pytest.mark.skipif(not GRPC_AVAILABLE, reason="gRPC not installed")
    @pytest.mark.asyncio
    async def test_connect(self, caplog):
        """Test connect method."""
        client = GRPCClient(host="localhost", port=50051)

        mock_channel = AsyncMock()
        with patch("grpc.aio.insecure_channel", return_value=mock_channel):
            with caplog.at_level("INFO"):
                await client.connect()
                assert client.channel is mock_channel
                assert "Connected to gRPC server" in caplog.text

    @pytest.mark.asyncio
    async def test_disconnect(self, caplog):
        """Test disconnect method."""
        client = GRPCClient()
        client.channel = AsyncMock()

        with caplog.at_level("INFO"):
            await client.disconnect()
            client.channel.close.assert_called_once()
            assert "Disconnected from gRPC server" in caplog.text

    @pytest.mark.asyncio
    async def test_disconnect_no_channel(self):
        """Test disconnect without channel."""
        client = GRPCClient()
        client.channel = None
        # Should not raise
        await client.disconnect()

    @pytest.mark.asyncio
    async def test_execute_command_not_connected(self):
        """Test execute_command when not connected."""
        client = GRPCClient()
        client.channel = None

        with pytest.raises(RuntimeError, match="Not connected to server"):
            await client.execute_command("move", {})

    @pytest.mark.asyncio
    async def test_execute_command_placeholder(self):
        """Test execute_command placeholder implementation."""
        client = GRPCClient()
        client.channel = MagicMock()  # Not None to pass the check

        result = await client.execute_command("move", {"x": 1.0})
        assert result["success"] is True
        assert "data" in result
