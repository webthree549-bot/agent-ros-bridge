"""gRPC transport for Agent ROS Bridge.

Provides high-performance, type-safe communication for robot control.
"""

import logging
from collections.abc import Callable
from concurrent import futures
from typing import Any

try:
    import grpc
    from grpc import aio
    GRPC_AVAILABLE = True
except ImportError:
    GRPC_AVAILABLE = False
    grpc = None  # type: ignore
    aio = None  # type: ignore

from ..auth import AuthConfig, Authenticator
from ..core import Transport

# Generated protobuf imports (would be generated from .proto files)
# from . import bridge_pb2
# from . import bridge_pb2_grpc

logger = logging.getLogger(__name__)


class GRPCServicer:
    """gRPC servicer for bridge commands."""

    def __init__(self, command_handler: Callable, authenticator: Authenticator):
        self.command_handler = command_handler
        self.authenticator = authenticator

    async def ExecuteCommand(self, request, context):
        """Execute a robot command via gRPC."""
        # Authenticate
        metadata = dict(context.invocation_metadata())
        token = metadata.get("authorization", "").replace("Bearer ", "")

        if not token:
            context.set_code(grpc.StatusCode.UNAUTHENTICATED)
            context.set_details("Missing authentication token")
            return  # bridge_pb2.CommandResponse()

        try:
            payload = self.authenticator.verify_token(token)
        except Exception as e:
            context.set_code(grpc.StatusCode.UNAUTHENTICATED)
            context.set_details(f"Invalid token: {e}")
            return  # bridge_pb2.CommandResponse()

        # Execute command
        try:
            await self.command_handler(
                {
                    "action": request.action,
                    "parameters": dict(request.parameters),
                    "user_id": payload.get("sub"),
                }
            )

            # return bridge_pb2.CommandResponse(
            #     success=result.get('success', False),
            #     data=str(result.get('data', {})),
            #     error=result.get('error', '')
            # )
        except Exception as e:
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(e))
            return  # bridge_pb2.CommandResponse()


class GRPCTransport(Transport):
    """gRPC transport implementation."""

    transport_type = "grpc"

    def __init__(self, config: dict[str, Any] | None = None):
        self.name = "grpc"
        self.config = config or {}
        self.host = self.config.get("host", "0.0.0.0")
        self.port = self.config.get("port", 50051)
        self.max_workers = self.config.get("max_workers", 10)
        self.reflection = self.config.get("reflection", True)
        self.keepalive_time_ms = self.config.get("keepalive_time_ms", 10000)
        self.tls_cert = self.config.get("tls_cert")
        self.tls_key = self.config.get("tls_key")
        self.ca_cert = self.config.get("ca_cert")
        self.server: aio.Server | None = None
        self.running = False
        auth_config = AuthConfig(
            jwt_secret=self.config.get("jwt_secret", "test-secret")
        )
        self.authenticator = Authenticator(auth_config)

    async def start(self) -> bool:
        """Start gRPC server."""
        if not GRPC_AVAILABLE:
            logger.error("gRPC not available")
            return False

        logger.info(f"Starting gRPC server on {self.host}:{self.port}")

        # Create server
        self.server = aio.server(
            futures.ThreadPoolExecutor(max_workers=self.max_workers),
            options=[
                ("grpc.max_send_message_length", 50 * 1024 * 1024),
                ("grpc.max_receive_message_length", 50 * 1024 * 1024),
            ],
        )

        # Add servicer
        # bridge_pb2_grpc.add_BridgeServicer_to_server(
        #     GRPCServicer(self.handle_command, self.authenticator),
        #     self.server
        # )

        # Bind port
        listen_addr = f"{self.host}:{self.port}"
        self.server.add_insecure_port(listen_addr)

        # Start server
        await self.server.start()
        logger.info(f"gRPC server started on {listen_addr}")
        self.running = True
        return True

    async def stop(self) -> None:
        """Stop gRPC server."""
        if self.server:
            logger.info("Stopping gRPC server...")
            await self.server.stop(grace_period=5)
            logger.info("gRPC server stopped")
        self.running = False

    async def send(self, message: Any, recipient: str) -> bool:
        """Send message to specific recipient (not implemented for gRPC)."""
        logger.warning("gRPC send not implemented")
        return False

    async def broadcast(self, message: Any) -> list[str]:
        """Broadcast to all connected clients (not implemented for gRPC)."""
        logger.warning("gRPC broadcast not implemented")
        return []

    async def handle_command(self, command: dict[str, Any]) -> dict[str, Any]:
        """Handle incoming command."""
        # This would be connected to the bridge's command processor
        logger.debug(f"Received gRPC command: {command}")
        return {"success": True, "data": {}}


class GRPCClient:
    """gRPC client for connecting to bridge."""

    def __init__(self, host: str = "localhost", port: int = 50051, token: str = ""):
        self.host = host
        self.port = port
        self.token = token
        self.channel: aio.Channel | None = None
        # self.stub: Optional[bridge_pb2_grpc.BridgeStub] = None

    async def connect(self) -> None:
        """Connect to gRPC server."""
        target = f"{self.host}:{self.port}"

        # Create channel (insecure for development, use ssl_channel_credentials() for TLS)
        self.channel = aio.insecure_channel(target)

        # self.stub = bridge_pb2_grpc.BridgeStub(self.channel)
        logger.info(f"Connected to gRPC server at {target}")

    async def disconnect(self) -> None:
        """Disconnect from server."""
        if self.channel:
            await self.channel.close()
            logger.info("Disconnected from gRPC server")

    async def execute_command(self, action: str, parameters: dict[str, Any]) -> dict[str, Any]:
        """Execute command via gRPC."""
        if not self.channel:
            raise RuntimeError("Not connected to server")

        # metadata = [("authorization", f"Bearer {self.token}")]

        # request = bridge_pb2.CommandRequest(
        #     action=action,
        #     parameters=parameters
        # )

        # response = await self.stub.ExecuteCommand(request, metadata=metadata)

        # return {
        #     'success': response.success,
        #     'data': response.data,
        #     'error': response.error
        # }

        # Placeholder until protobuf is generated
        return {"success": True, "data": {}}
