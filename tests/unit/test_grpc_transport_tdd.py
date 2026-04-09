"""Unit tests for gRPC Transport - TDD Style.

Tests written BEFORE implementation (following TDD Red-Green-Refactor).
"""

from unittest.mock import AsyncMock, Mock, patch

import pytest

pytestmark = [
    pytest.mark.unit,
    pytest.mark.skipif(
        not pytest.importorskip("grpc", reason="gRPC not available"), reason="grpc not installed"
    ),
]


class TestGRPCTransportInitialization:
    """Test 1: GRPCTransport should initialize with proper config."""

    def test_transport_initialization_defaults(self):
        """Red: Should initialize with default values."""
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

        transport = GRPCTransport({})

        assert transport.name == "grpc"
        assert transport.host == "0.0.0.0"
        assert transport.port == 50051
        assert transport.reflection is True
        assert transport.max_workers == 10
        assert transport.keepalive_time_ms == 10000
        assert not transport.running

    def test_transport_initialization_custom_config(self):
        """Red: Should accept custom configuration."""
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

        config = {
            "host": "127.0.0.1",
            "port": 50052,
            "reflection": False,
            "max_workers": 20,
            "keepalive_time_ms": 5000,
            "tls_cert": "/path/to/cert",
            "tls_key": "/path/to/key",
            "ca_cert": "/path/to/ca",
        }

        transport = GRPCTransport(config)

        assert transport.host == "127.0.0.1"
        assert transport.port == 50052
        assert transport.reflection is False
        assert transport.max_workers == 20
        assert transport.tls_cert == "/path/to/cert"
        assert transport.tls_key == "/path/to/key"
        assert transport.ca_cert == "/path/to/ca"


class TestGRPCServerLifecycle:
    """Test 2: gRPC server should start and stop properly."""

    @pytest.mark.asyncio
    @pytest.mark.skip(reason="TDD test - needs implementation update for aio import")
    async def test_start_creates_server(self):
        """Red: Start should create and start gRPC server."""
        with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.grpc") as mock_grpc:
            from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

            mock_server = AsyncMock()
            mock_grpc.aio.server.return_value = mock_server

            transport = GRPCTransport({"port": 50051})
            result = await transport.start()

            assert result is True
            assert transport.running
            mock_grpc.aio.server.assert_called_once()
            mock_server.start.assert_called_once()

    @pytest.mark.asyncio
    @pytest.mark.skip(reason="TDD test - needs implementation update for aio import")
    async def test_stop_gracefully_shutdowns(self):
        """Red: Stop should gracefully shutdown server."""
        with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.grpc") as mock_grpc:
            from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

            mock_server = AsyncMock()
            mock_grpc.aio.server.return_value = mock_server

            transport = GRPCTransport({"port": 50051})
            await transport.start()
            await transport.stop()

            assert not transport.running
            mock_server.stop.assert_called_once()

    @pytest.mark.asyncio
    async def test_start_without_grpc_returns_false(self):
        """Red: Should return False if gRPC not available."""
        with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.GRPC_AVAILABLE", False):
            from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

            transport = GRPCTransport({})
            result = await transport.start()

            assert result is False


@pytest.mark.skip(reason="TDD tests - implementation pending")
class TestGRPCTLSConfiguration:
    """Test 3: gRPC should support TLS and mTLS."""

    @pytest.mark.asyncio
    async def test_tls_enabled_with_cert_and_key(self):
        """Red: Should enable TLS when cert and key provided."""
        with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.grpc") as mock_grpc:
            from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

            mock_server = AsyncMock()
            mock_grpc.aio.server.return_value = mock_server

            with patch("builtins.open", mock_open(read_data=b"cert_data")):
                transport = GRPCTransport(
                    {
                        "tls_cert": "/path/to/cert.pem",
                        "tls_key": "/path/to/key.pem",
                    }
                )
                await transport.start()

                mock_grpc.ssl_server_credentials.assert_called_once()
                mock_server.add_secure_port.assert_called_once()

    @pytest.mark.asyncio
    async def test_mtls_enabled_with_ca_cert(self):
        """Red: Should enable mTLS when CA cert provided."""
        with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.grpc") as mock_grpc:
            from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

            mock_server = AsyncMock()
            mock_grpc.aio.server.return_value = mock_server

            with patch("builtins.open", mock_open(read_data=b"cert_data")):
                transport = GRPCTransport(
                    {
                        "tls_cert": "/path/to/cert.pem",
                        "tls_key": "/path/to/key.pem",
                        "ca_cert": "/path/to/ca.pem",
                    }
                )
                await transport.start()

                # Should create credentials with client auth required
                call_args = mock_grpc.ssl_server_credentials.call_args
                assert call_args[1]["require_client_auth"] is True

    @pytest.mark.asyncio
    async def test_insecure_fallback_on_tls_error(self):
        """Red: Should fall back to insecure if TLS fails."""
        with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.grpc") as mock_grpc:
            from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

            mock_server = AsyncMock()
            mock_grpc.aio.server.return_value = mock_server

            with patch("builtins.open", side_effect=FileNotFoundError("Cert not found")):
                transport = GRPCTransport(
                    {
                        "tls_cert": "/invalid/path",
                        "tls_key": "/invalid/path",
                    }
                )
                result = await transport.start()

                assert result is True
                mock_server.add_insecure_port.assert_called_once()


def mock_open(read_data=b""):
    """Helper to mock open() for file reads."""
    from unittest.mock import mock_open as _mock_open

    return _mock_open(read_data=read_data)


class TestGRPCServiceRegistration:
    """Test 4: gRPC service should be registered correctly."""

    @pytest.mark.asyncio
    @pytest.mark.skip(reason="TDD test - bridge_pb2 not yet implemented")
    async def test_bridge_service_registered(self):
        """Red: BridgeService should be registered to server."""
        with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.grpc") as mock_grpc:
            with patch(
                "agent_ros_bridge.gateway_v2.transports.grpc_transport.bridge_pb2"
            ) as mock_pb2:
                from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

                mock_server = AsyncMock()
                mock_grpc.aio.server.return_value = mock_server

                transport = GRPCTransport({})
                await transport.start()

                mock_pb2.add_BridgeServiceServicer_to_server.assert_called_once()

    @pytest.mark.asyncio
    async def test_reflection_enabled_when_configured(self):
        """Red: Reflection should be enabled when configured."""
        with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.grpc") as mock_grpc:
            with patch("grpc_reflection.v1alpha.reflection") as mock_reflection:
                from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

                mock_server = AsyncMock()
                mock_grpc.aio.server.return_value = mock_server

                transport = GRPCTransport({"reflection": True})
                await transport.start()

                mock_reflection.enable_server_reflection.assert_called_once()


@pytest.mark.skip(reason="TDD tests - implementation pending")
class TestGRPCClientManagement:
    """Test 5: gRPC should track connected clients."""

    @pytest.mark.asyncio
    async def test_get_connected_clients_empty(self):
        """Red: Should return empty list when no clients."""
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

        transport = GRPCTransport({})
        clients = transport.get_connected_clients()

        assert clients == []

    @pytest.mark.asyncio
    async def test_get_stats_returns_current_state(self):
        """Red: Should return transport statistics."""
        with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.grpc") as mock_grpc:
            from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

            mock_server = AsyncMock()
            mock_grpc.aio.server.return_value = mock_server

            transport = GRPCTransport(
                {
                    "host": "0.0.0.0",
                    "port": 50051,
                }
            )
            await transport.start()

            stats = transport.get_stats()

            assert stats["running"] is True
            assert stats["host"] == "0.0.0.0"
            assert stats["port"] == 50051
            assert stats["tls_enabled"] is False
            assert "connected_clients" in stats


@pytest.mark.skip(reason="TDD tests - implementation pending")
class TestGRPCMessageConversion:
    """Test 6: gRPC should convert messages bidirectionally."""

    def test_proto_to_message_command(self):
        """Red: Should convert protobuf to Command."""
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import BridgeServiceServicer

        servicer = BridgeServiceServicer(Mock())

        # Create mock proto message
        mock_proto = Mock()
        mock_proto.header.message_id = "test-id"
        mock_proto.command.action = "test_action"
        mock_proto.command.parameters.fields = {"key": "value"}
        mock_proto.command.timeout_ms = 10000
        mock_proto.command.priority = 3

        message = servicer._proto_to_message(mock_proto)

        assert message.header.message_id == "test-id"
        assert message.command.action == "test_action"
        assert message.command.parameters == {"key": "value"}
        assert message.command.timeout_ms == 10000
        assert message.command.priority == 3

    def test_message_to_proto_response(self):
        """Red: Should convert Message to CommandResponse protobuf."""
        from agent_ros_bridge.gateway_v2.core import Event, Message, Telemetry
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import BridgeServiceServicer

        servicer = BridgeServiceServicer(Mock())

        message = Message(
            telemetry=Telemetry(topic="test", data={"status": "ok"}),
            event=Event(event_type="test", severity="info", data={}),
        )

        response = servicer._message_to_proto_response(message)

        assert response.success is True
        assert "status" in str(response.result)

    def test_struct_to_dict_conversion(self):
        """Red: Should convert protobuf Struct to dict."""
        from google.protobuf import struct_pb2

        from agent_ros_bridge.gateway_v2.transports.grpc_transport import BridgeServiceServicer

        servicer = BridgeServiceServicer(Mock())

        struct = struct_pb2.Struct()
        struct["key1"] = "value1"
        struct["key2"] = 123
        struct["nested"]["field"] = "nested_value"

        result = servicer._struct_to_dict(struct)

        assert result["key1"] == "value1"
        assert result["key2"] == 123
        assert result["nested"]["field"] == "nested_value"


class TestGRPCIdentityExtraction:
    """Test 7: Should extract identity from gRPC metadata."""

    def test_extract_identity_from_metadata(self):
        """Red: Should extract user identity from request metadata."""
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import BridgeServiceServicer

        servicer = BridgeServiceServicer(Mock())

        mock_context = Mock()
        mock_context.invocation_metadata.return_value = [
            ("x-user-id", "user-123"),
            ("x-user-name", "Test User"),
            ("x-roles", "operator,admin"),
            ("authorization", "Bearer token123"),
        ]
        mock_context.peer.return_value = "ipv4:127.0.0.1:12345"

        identity = servicer._extract_identity(mock_context)

        assert identity.id == "user-123"
        assert identity.name == "Test User"
        assert "operator" in identity.roles
        assert "admin" in identity.roles
        assert identity.metadata["transport"] == "grpc"

    def test_extract_anonymous_identity_no_auth(self):
        """Red: Should create anonymous identity without auth."""
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import BridgeServiceServicer

        servicer = BridgeServiceServicer(Mock())

        mock_context = Mock()
        mock_context.invocation_metadata.return_value = []
        mock_context.peer.return_value = "ipv4:127.0.0.1:12345"

        identity = servicer._extract_identity(mock_context)

        assert identity.id is not None  # Should generate UUID
        assert identity.name.startswith("anonymous_")
        assert "anonymous" in identity.roles


@pytest.mark.skip(reason="TDD tests - implementation pending")
class TestGRPCServicerRPCs:
    """Test 8: gRPC servicer should implement all RPCs."""

    @pytest.mark.asyncio
    @pytest.mark.skip(reason="TDD test - bridge_pb2 not yet implemented")
    async def test_send_command_rpc(self):
        """Red: Should handle SendCommand RPC."""
        with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.bridge_pb2"):
            from agent_ros_bridge.gateway_v2.transports.grpc_transport import BridgeServiceServicer

            mock_transport = Mock()
            mock_transport.message_handler = AsyncMock(
                return_value=Mock(telemetry=Mock(data={"result": "success"}), event=None)
            )

            servicer = BridgeServiceServicer(mock_transport)

            mock_request = Mock()
            mock_request.header.message_id = "msg-1"
            mock_request.command.action = "test_action"
            mock_request.command.parameters.fields = {}

            mock_context = Mock()
            mock_context.invocation_metadata.return_value = []
            mock_context.peer.return_value = "ipv4:127.0.0.1:12345"

            response = await servicer.SendCommand(mock_request, mock_context)

            assert response.success is True
            mock_transport.message_handler.assert_called_once()

    @pytest.mark.asyncio
    async def test_health_check_rpc(self):
        """Red: Should handle HealthCheck RPC."""
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import BridgeServiceServicer

        servicer = BridgeServiceServicer(Mock())

        mock_context = Mock()

        response = await servicer.HealthCheck(Mock(), mock_context)

        assert response.success is True
        assert "healthy" in str(response.result).lower()

    @pytest.mark.asyncio
    async def test_subscribe_telemetry_stream(self):
        """Red: Should stream telemetry via SubscribeTelemetry."""
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import BridgeServiceServicer

        mock_transport = Mock()
        servicer = BridgeServiceServicer(mock_transport)

        mock_request = Mock()
        mock_request.topics = ["/odom", "/scan"]
        mock_request.robot_id = "robot-1"

        mock_context = Mock()
        mock_context.done.return_value = True  # Stop after first iteration

        responses = []
        async for response in servicer.SubscribeTelemetry(mock_request, mock_context):
            responses.append(response)
            break  # Only collect one for test

        assert len(responses) > 0


@pytest.mark.skip(reason="TDD tests - implementation pending")
class TestGRPCClientHelper:
    """Test 9: GRPCClient helper should work for client connections."""

    @pytest.mark.asyncio
    async def test_client_connect(self):
        """Red: GRPCClient should connect to server."""
        with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.grpc") as mock_grpc:
            from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCClient

            mock_channel = AsyncMock()
            mock_grpc.aio.insecure_channel.return_value = mock_channel

            client = GRPCClient(target="localhost:50051")
            await client.connect()

            mock_grpc.aio.insecure_channel.assert_called_once_with("localhost:50051")
            assert client.channel is not None
            assert client.stub is not None

    @pytest.mark.asyncio
    @pytest.mark.skip(reason="TDD test - bridge_pb2 not yet implemented")
    async def test_client_send_command(self):
        """Red: GRPCClient should send commands."""
        with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.grpc"):
            with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.bridge_pb2"):
                from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCClient

                mock_stub = AsyncMock()
                mock_response = Mock()
                mock_response.success = True
                mock_response.result.fields = {"status": "ok"}
                mock_response.error = ""
                mock_stub.SendCommand.return_value = mock_response

                client = GRPCClient(target="localhost:50051")
                client.stub = mock_stub

                result = await client.send_command("discover", {})

                assert result["success"] is True
                assert result["result"]["status"] == "ok"

    @pytest.mark.asyncio
    @pytest.mark.skip(reason="TDD test - bridge_pb2 not yet implemented")
    async def test_client_health_check(self):
        """Red: GRPCClient should check health."""
        with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.grpc"):
            with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.bridge_pb2"):
                from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCClient

                mock_stub = AsyncMock()
                mock_response = Mock()
                mock_response.success = True
                mock_response.result.fields = {"status": "healthy"}
                mock_stub.HealthCheck.return_value = mock_response

                client = GRPCClient(target="localhost:50051")
                client.stub = mock_stub

                result = await client.health_check()

                assert result["success"] is True
                mock_stub.HealthCheck.assert_called_once()


class TestGRPCBroadcast:
    """Test 10: gRPC should support broadcasting to clients."""

    @pytest.mark.asyncio
    async def test_broadcast_to_all_clients(self):
        """Red: Should broadcast message to all connected clients."""
        with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.grpc") as mock_grpc:
            from agent_ros_bridge.gateway_v2.core import Message, Telemetry
            from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

            mock_server = AsyncMock()
            mock_grpc.aio.server.return_value = mock_server

            transport = GRPCTransport({})
            await transport.start()

            # Mock service with clients
            mock_client = Mock()
            mock_client.client_id = "client-1"
            transport.service = Mock()
            transport.service.clients = {"client-1": mock_client}
            transport.service._queue_telemetry = AsyncMock()

            message = Message(telemetry=Telemetry(topic="broadcast", data={"alert": "test"}))
            recipients = await transport.broadcast(message)

            assert "client-1" in recipients


class TestGRPCErrorHandling:
    """Test 11: gRPC should handle errors gracefully."""

    @pytest.mark.asyncio
    async def test_send_to_disconnected_client(self):
        """Red: Should handle send to disconnected client."""
        from agent_ros_bridge.gateway_v2.core import Message
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

        transport = GRPCTransport({})
        transport.service = None  # No service = no clients

        result = await transport.send(Message(), "nonexistent-client")

        assert result is False

    @pytest.mark.asyncio
    @pytest.mark.skip(reason="TDD test - bridge_pb2 not yet implemented")
    async def test_send_command_with_no_handler(self):
        """Red: Should return error when no message handler registered."""
        with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.bridge_pb2"):
            from agent_ros_bridge.gateway_v2.transports.grpc_transport import BridgeServiceServicer

            mock_transport = Mock()
            mock_transport.message_handler = None

            servicer = BridgeServiceServicer(mock_transport)

            mock_request = Mock()
            mock_request.header.message_id = "msg-1"
            mock_request.command.action = "test"

            mock_context = Mock()
            mock_context.invocation_metadata.return_value = []
            mock_context.peer.return_value = "ipv4:127.0.0.1:12345"

            response = await servicer.SendCommand(mock_request, mock_context)

            assert response.success is False
            assert "No handler registered" in response.error


class TestGRPCJWTAuthentication:
    """Test 12: gRPC should support JWT authentication."""

    def test_transport_auth_config_defaults(self):
        """Red: Should have auth disabled by default."""
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

        transport = GRPCTransport({})

        assert transport.auth_enabled is False
        assert transport.jwt_secret is None

    def test_transport_auth_config_enabled(self):
        """Red: Should accept auth configuration."""
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

        config = {
            "auth": {
                "enabled": True,
                "jwt_secret": "test-secret-key",
            }
        }

        transport = GRPCTransport(config)

        assert transport.auth_enabled is True
        assert transport.jwt_secret == "test-secret-key"

    @pytest.mark.asyncio
    async def test_extract_identity_with_valid_jwt(self):
        """Red: Should extract identity from valid JWT token."""
        with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.AUTH_AVAILABLE", True):
            from agent_ros_bridge.gateway_v2.transports.grpc_transport import (
                BridgeServiceServicer,
                GRPCTransport,
            )

            # Create transport with auth enabled
            transport = GRPCTransport(
                {
                    "auth": {
                        "enabled": True,
                        "jwt_secret": "test-secret-key-for-jwt-signing",
                    }
                }
            )

            servicer = BridgeServiceServicer(transport)

            # Create a valid JWT token
            import jwt

            token = jwt.encode(
                {"sub": "user-123", "roles": ["admin", "user"]},
                "test-secret-key-for-jwt-signing",
                algorithm="HS256",
            )

            mock_context = Mock()
            mock_context.invocation_metadata.return_value = [
                ("authorization", f"Bearer {token}"),
            ]
            mock_context.peer.return_value = "ipv4:127.0.0.1:12345"

            identity = servicer._extract_identity(mock_context)

            assert identity.id == "user-123"
            assert identity.roles == ["admin", "user"]
            assert identity.metadata["auth_status"] == "authenticated"

    @pytest.mark.asyncio
    async def test_extract_identity_with_invalid_jwt(self):
        """Red: Should reject invalid JWT token."""
        with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.AUTH_AVAILABLE", True):
            from grpc import StatusCode

            from agent_ros_bridge.gateway_v2.transports.grpc_transport import (
                BridgeServiceServicer,
                GRPCTransport,
            )

            # Create transport with auth enabled
            transport = GRPCTransport(
                {
                    "auth": {
                        "enabled": True,
                        "jwt_secret": "test-secret-key-for-jwt-signing",
                    }
                }
            )

            servicer = BridgeServiceServicer(transport)

            # Create an invalid JWT token (wrong secret)
            import jwt

            token = jwt.encode(
                {"sub": "user-123", "roles": ["admin"]},
                "wrong-secret-key",
                algorithm="HS256",
            )

            mock_context = Mock()
            mock_context.invocation_metadata.return_value = [
                ("authorization", f"Bearer {token}"),
            ]
            mock_context.peer.return_value = "ipv4:127.0.0.1:12345"

            # Should raise exception and set gRPC error code
            with pytest.raises(Exception, match="Authentication failed"):
                servicer._extract_identity(mock_context)

            mock_context.set_code.assert_called_once_with(StatusCode.UNAUTHENTICATED)

    @pytest.mark.asyncio
    async def test_extract_identity_without_token_when_auth_required(self):
        """Red: Should reject requests without token when auth is enabled."""
        with patch("agent_ros_bridge.gateway_v2.transports.grpc_transport.AUTH_AVAILABLE", True):
            from grpc import StatusCode

            from agent_ros_bridge.gateway_v2.transports.grpc_transport import (
                BridgeServiceServicer,
                GRPCTransport,
            )

            # Create transport with auth enabled
            transport = GRPCTransport(
                {
                    "auth": {
                        "enabled": True,
                        "jwt_secret": "test-secret-key",
                    }
                }
            )

            servicer = BridgeServiceServicer(transport)

            mock_context = Mock()
            mock_context.invocation_metadata.return_value = []  # No auth header
            mock_context.peer.return_value = "ipv4:127.0.0.1:12345"

            # Should raise exception and set gRPC error code
            with pytest.raises(Exception, match="Authentication failed"):
                servicer._extract_identity(mock_context)

            mock_context.set_code.assert_called_once_with(StatusCode.UNAUTHENTICATED)

    @pytest.mark.asyncio
    async def test_extract_identity_anonymous_when_auth_disabled(self):
        """Red: Should allow anonymous access when auth is disabled."""
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import (
            BridgeServiceServicer,
            GRPCTransport,
        )

        # Create transport without auth
        transport = GRPCTransport({})

        servicer = BridgeServiceServicer(transport)

        mock_context = Mock()
        mock_context.invocation_metadata.return_value = []  # No auth header
        mock_context.peer.return_value = "ipv4:127.0.0.1:12345"

        identity = servicer._extract_identity(mock_context)

        assert identity.roles == ["anonymous"]
        assert identity.metadata["auth_status"] == "anonymous"

    def test_get_stats_includes_auth_status(self):
        """Red: Should include auth status in stats."""
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

        transport = GRPCTransport({"auth": {"enabled": True, "jwt_secret": "secret"}})

        stats = transport.get_stats()

        assert "auth_enabled" in stats
        assert stats["auth_enabled"] is True
