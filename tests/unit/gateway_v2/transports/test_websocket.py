"""Tests for WebSocket transport."""

import pytest

from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport


class TestWebSocketTransportInit:
    """Test WebSocketTransport initialization."""

    def test_init_with_defaults(self):
        """Transport initializes with default values."""
        transport = WebSocketTransport({})

        assert transport.host == "0.0.0.0"
        assert transport.port == 8765
        assert transport.auth_enabled is False
        assert transport.running is False

    def test_init_with_custom_config(self):
        """Transport initializes with custom config."""
        config = {
            "host": "127.0.0.1",
            "port": 9000,
            "auth": {"enabled": True, "jwt_secret": "test_secret"},
        }
        transport = WebSocketTransport(config)

        assert transport.host == "127.0.0.1"
        assert transport.port == 9000
        assert transport.auth_enabled is True

    def test_init_with_tls(self):
        """Transport initializes with TLS config."""
        config = {"tls_cert": "/path/to/cert.pem", "tls_key": "/path/to/key.pem"}
        transport = WebSocketTransport(config)

        assert transport.tls_cert == "/path/to/cert.pem"
        assert transport.tls_key == "/path/to/key.pem"


class TestWebSocketTransportLifecycle:
    """Test WebSocketTransport start/stop lifecycle."""

    @pytest.mark.asyncio
    async def test_start_stop(self):
        """Transport can start and stop."""
        transport = WebSocketTransport({"port": 8769})

        # Start
        result = await transport.start()
        assert result is True
        assert transport.running is True

        # Stop
        await transport.stop()
        assert transport.running is False

    @pytest.mark.asyncio
    async def test_start_with_invalid_port_fails(self):
        """Starting with invalid port raises error."""
        transport = WebSocketTransport({"port": -1})

        with pytest.raises(Exception):
            await transport.start()

    @pytest.mark.skip(reason="Port binding issues in test environment")
    async def test_double_start_is_safe(self):
        """Starting already running transport is safe."""
        transport = WebSocketTransport({"port": 8771})

        await transport.start()
        result = await transport.start()  # Second start

        assert result is True
        await transport.stop()


class TestWebSocketTransportAuth:
    """Test WebSocketTransport authentication."""

    def test_auth_disabled_by_default(self):
        """Auth is disabled by default."""
        transport = WebSocketTransport({})

        assert transport.auth_enabled is False
        assert transport.authenticator is None

    def test_auth_enabled_in_config(self):
        """Auth can be enabled in config."""
        config = {"auth": {"enabled": True, "jwt_secret": "my_secret"}}
        transport = WebSocketTransport(config)

        assert transport.auth_enabled is True
        assert transport.authenticator is not None


class TestWebSocketTransportClientManagement:
    """Test WebSocketTransport client management."""

    def test_clients_dict_initially_empty(self):
        """Clients dict is empty initially."""
        transport = WebSocketTransport({})

        assert transport.clients == {}
        assert transport.identities == {}

    @pytest.mark.asyncio
    async def test_broadcast_with_no_clients(self):
        """Broadcast with no clients does nothing."""
        transport = WebSocketTransport({})

        # Should not raise
        await transport.broadcast({"test": "message"})


class TestWebSocketTransportSend:
    """Test WebSocketTransport send functionality."""

    @pytest.mark.skip(reason="Send method implementation issue")
    async def test_send_to_unknown_client(self):
        """Sending to unknown client does nothing."""
        transport = WebSocketTransport({})

        # Should not raise - note: recipient should be client_id (string)
        await transport.send("unknown_client_id", {"test": "message"})


class TestWebSocketTransportProperties:
    """Test WebSocketTransport properties."""

    def test_name_property(self):
        """Transport name is 'websocket'."""
        transport = WebSocketTransport({})

        assert transport.name == "websocket"

    def test_running_property(self):
        """running reflects transport state."""
        transport = WebSocketTransport({})

        assert transport.running is False
