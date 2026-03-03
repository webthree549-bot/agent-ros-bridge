#!/usr/bin/env python3
"""gRPC Transport for Agent ROS Bridge.

High-performance gRPC transport for microservices, cloud deployments,
and strongly-typed integrations.

Features:
- Bidirectional streaming for real-time commands and telemetry
- Telemetry subscription management
- Client connection tracking
- Authentication via metadata
- Broadcast support via streaming
- Health checking
"""

import asyncio
import logging
import time
import uuid
from concurrent import futures
from dataclasses import dataclass
from typing import Any, AsyncIterator, Callable, Dict, List, Optional, Set

from google.protobuf import struct_pb2
from google.protobuf.empty_pb2 import Empty

try:
    import grpc
    from grpc import StatusCode
    from grpc.aio import Server, ServicerContext

    from agent_ros_bridge.proto import bridge_pb2

    GRPC_AVAILABLE = True
except ImportError as e:
    GRPC_AVAILABLE = False
    logging.debug(f"gRPC not available: {e}")
    # Define placeholders for type hints
    Server = Any
    ServicerContext = Any
    StatusCode = Any

import contextlib

from agent_ros_bridge.gateway_v2.core import (
    Command,
    Event,
    Header,
    Identity,
    Message,
    Telemetry,
    Transport,
)

logger = logging.getLogger("transport.grpc")


@dataclass
class GRPCClient:
    """Represents a connected gRPC client."""

    client_id: str
    peer: str
    identity: Identity
    context: Any  # ServicerContext
    subscriptions: Set[str]
    connected_at: float
    last_activity: float


class BridgeServiceServicer(bridge_pb2.BridgeServiceServicer if GRPC_AVAILABLE else object):
    """Bridge service implementation with full streaming support."""

    def __init__(self, transport: "GRPCTransport"):
        """Initialize servicer with transport reference.

        Args:
            transport: Parent gRPC transport instance for message handling.
        """
        self.transport = transport
        self.clients: Dict[str, GRPCClient] = {}
        self._telemetry_queues: Dict[str, asyncio.Queue] = {}  # client_id -> queue
        self._shutdown_event = asyncio.Event()

    async def SendCommand(
        self, request: bridge_pb2.Message, context: ServicerContext
    ) -> bridge_pb2.CommandResponse:
        """Handle incoming command (unary-unary)."""
        start_time = time.time()
        client_id = str(uuid.uuid4())

        try:
            # Extract identity from metadata
            identity = self._extract_identity(context)
            client_id = identity.id

            # Track client
            self._register_client(client_id, context, identity)

            # Convert and handle message
            message = self._proto_to_message(request)

            if self.transport.message_handler:
                response = await self.transport.message_handler(message, identity)
                if response:
                    duration_ms = (time.time() - start_time) * 1000
                    logger.debug(f"Command processed in {duration_ms:.2f}ms")
                    return self._message_to_proto_response(response)

            return bridge_pb2.CommandResponse(success=False, error="No handler registered")

        except Exception as e:
            logger.error(f"Error handling command: {e}")
            return bridge_pb2.CommandResponse(success=False, error=str(e))

    async def StreamTelemetry(
        self, request_iterator: AsyncIterator[bridge_pb2.Message], context: ServicerContext
    ) -> AsyncIterator[bridge_pb2.Message]:
        """Bidirectional streaming for real-time commands and telemetry.

        Clients can:
        1. Send commands via the request stream
        2. Receive telemetry via the response stream
        """
        client_id = str(uuid.uuid4())
        identity = self._extract_identity(context)
        client_id = identity.id

        # Register client
        self._register_client(client_id, context, identity)
        self._telemetry_queues[client_id] = asyncio.Queue(maxsize=100)

        logger.info(f"Client {client_id} started bidirectional stream")

        # Start telemetry sender task
        sender_task = asyncio.create_task(self._stream_telemetry_to_client(client_id, context))

        try:
            async for request in request_iterator:
                # Process incoming command
                try:
                    message = self._proto_to_message(request)

                    if self.transport.message_handler:
                        response = await self.transport.message_handler(message, identity)
                        if response:
                            # Send response back through telemetry queue
                            proto_response = self._message_to_proto(response)
                            await self._queue_telemetry(client_id, proto_response)

                except Exception as e:
                    logger.error(f"Error processing stream message: {e}")
                    await self._queue_telemetry(
                        client_id,
                        bridge_pb2.Message(
                            event=bridge_pb2.Event(
                                type="error",
                                severity="error",
                                data=self._dict_to_struct({"error": str(e)}),
                            )
                        ),
                    )

                # Check if context is still active
                if context.done():
                    break

        except Exception as e:
            logger.error(f"Stream error for {client_id}: {e}")

        finally:
            # Cleanup
            sender_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await sender_task

            self._unregister_client(client_id)
            logger.info(f"Client {client_id} ended bidirectional stream")

    async def SubscribeTelemetry(
        self, request: bridge_pb2.SubscriptionRequest, context: ServicerContext
    ) -> AsyncIterator[bridge_pb2.Telemetry]:
        """Server streaming for telemetry subscription."""
        client_id = str(uuid.uuid4())
        identity = self._extract_identity(context)
        client_id = identity.id

        topics = list(request.topics)
        robot_id = request.robot_id

        logger.info(f"Client {client_id} subscribing to topics: {topics}, robot: {robot_id}")

        # Register client with subscriptions
        self._register_client(client_id, context, identity)
        self.clients[client_id].subscriptions.update(topics)

        # Create telemetry queue for this client
        self._telemetry_queues[client_id] = asyncio.Queue(maxsize=1000)

        try:
            # Subscribe to robot telemetry if specified
            if robot_id and self.transport.message_handler:
                await self._subscribe_to_robot(robot_id, topics, client_id)

            # Stream telemetry to client
            while not context.done():
                try:
                    # Wait for telemetry with timeout
                    telemetry = await asyncio.wait_for(
                        self._telemetry_queues[client_id].get(), timeout=1.0
                    )
                    yield telemetry
                except asyncio.TimeoutError:
                    # Send heartbeat to keep connection alive
                    yield bridge_pb2.Telemetry(
                        topic="_heartbeat",
                        data=self._dict_to_struct({"timestamp": time.time()}),
                        quality=1.0,
                    )

        except Exception as e:
            logger.error(f"Subscription error for {client_id}: {e}")

        finally:
            self._unregister_client(client_id)
            logger.info(f"Client {client_id} unsubscribed")

    async def HealthCheck(
        self, _request: Empty, context: ServicerContext
    ) -> bridge_pb2.CommandResponse:
        """Health check endpoint."""
        return bridge_pb2.CommandResponse(
            success=True,
            result=self._dict_to_struct(
                {
                    "status": "healthy",
                    "transport": "grpc",
                    "clients": len(self.clients),
                    "timestamp": time.time(),
                }
            ),
        )

    async def _stream_telemetry_to_client(self, client_id: str, context: ServicerContext):
        """Background task to stream telemetry to a client."""
        queue = self._telemetry_queues.get(client_id)
        if not queue:
            return

        try:
            while not context.done():
                try:
                    message = await asyncio.wait_for(queue.get(), timeout=5.0)
                    yield message
                except asyncio.TimeoutError:
                    continue
        except asyncio.CancelledError:
            logger.debug(f"Telemetry stream cancelled for {client_id}")
        except Exception as e:
            logger.error(f"Telemetry stream error for {client_id}: {e}")

    async def _subscribe_to_robot(self, robot_id: str, topics: List[str], client_id: str):
        """Subscribe to robot telemetry and route to client."""
        # This would integrate with the bridge's robot fleet
        # For now, create a command to subscribe
        if not self.transport.message_handler:
            return

        identity = self.clients[client_id].identity

        for topic in topics:
            message = Message(
                header=Header(),
                command=Command(
                    action="robot.subscribe", parameters={"robot_id": robot_id, "topic": topic}
                ),
            )

            try:
                response = await self.transport.message_handler(message, identity)
                if response and response.telemetry:
                    # Route telemetry to client's queue
                    await self._queue_telemetry(
                        client_id, self._telemetry_to_proto(response.telemetry)
                    )
            except Exception as e:
                logger.error(f"Failed to subscribe to {topic}: {e}")

    async def _queue_telemetry(self, client_id: str, telemetry: bridge_pb2.Telemetry):
        """Queue telemetry for a client."""
        queue = self._telemetry_queues.get(client_id)
        if not queue:
            return

        try:
            queue.put_nowait(telemetry)
        except asyncio.QueueFull:
            # Drop oldest message
            try:
                queue.get_nowait()
                queue.put_nowait(telemetry)
            except asyncio.QueueEmpty:
                pass

    def _register_client(self, client_id: str, context: ServicerContext, identity: Identity):
        """Register a connected client."""
        client = GRPCClient(
            client_id=client_id,
            peer=str(context.peer()),
            identity=identity,
            context=context,
            subscriptions=set(),
            connected_at=time.time(),
            last_activity=time.time(),
        )
        self.clients[client_id] = client
        self.transport._on_client_connect(client_id, identity)

    def _unregister_client(self, client_id: str):
        """Unregister a disconnected client."""
        if client_id in self.clients:
            del self.clients[client_id]
        if client_id in self._telemetry_queues:
            del self._telemetry_queues[client_id]
        self.transport._on_client_disconnect(client_id)

    def _extract_identity(self, context: ServicerContext) -> Identity:
        """Extract identity from gRPC metadata."""
        metadata = dict(context.invocation_metadata() or [])

        # Extract JWT token from metadata
        auth_header = metadata.get("authorization", "")
        if auth_header.startswith("Bearer "):
            auth_header[7:]
            # TODO: Validate JWT and extract claims
            user_id = metadata.get("x-user-id", str(uuid.uuid4()))
            user_name = metadata.get("x-user-name", f"user_{user_id[:8]}")
            roles_str = metadata.get("x-roles", "")
            roles = roles_str.split(",") if roles_str else ["authenticated"]
        else:
            user_id = metadata.get("x-user-id", str(uuid.uuid4()))
            user_name = metadata.get("x-user-name", f"anonymous_{user_id[:8]}")
            roles = ["anonymous"]

        return Identity(
            id=user_id,
            name=user_name,
            roles=roles,
            metadata={
                "transport": "grpc",
                "peer": str(context.peer()),
                "auth_header": auth_header[:20] + "..." if auth_header else None,
            },
        )

    def _proto_to_message(self, proto: bridge_pb2.Message) -> Message:
        """Convert protobuf to Message."""
        # Header
        header = Header()
        if proto.header and proto.header.message_id:
            header.message_id = proto.header.message_id
            header.source = proto.header.source
            header.target = proto.header.target
            header.correlation_id = proto.header.correlation_id

        # Command
        command = None
        if proto.command and proto.command.action:
            params = {}
            if proto.command.parameters:
                params = self._struct_to_dict(proto.command.parameters)
            command = Command(
                action=proto.command.action,
                parameters=params,
                timeout_ms=proto.command.timeout_ms or 5000,
                priority=proto.command.priority or 5,
            )

        # Telemetry
        telemetry = None
        if proto.telemetry and proto.telemetry.topic:
            telemetry = Telemetry(
                topic=proto.telemetry.topic,
                data=self._struct_to_dict(proto.telemetry.data) if proto.telemetry.data else {},
                quality=proto.telemetry.quality,
            )

        # Event
        event = None
        if proto.event and proto.event.type:
            event = Event(
                event_type=proto.event.type,
                severity=proto.event.severity,
                data=self._struct_to_dict(proto.event.data) if proto.event.data else {},
            )

        # Metadata
        metadata = self._struct_to_dict(proto.metadata) if proto.metadata else {}

        return Message(
            header=header, command=command, telemetry=telemetry, event=event, metadata=metadata
        )

    def _message_to_proto(self, message: Message) -> bridge_pb2.Message:
        """Convert Message to protobuf."""
        proto = bridge_pb2.Message()

        # Header
        if message.header:
            proto.header.message_id = message.header.message_id
            proto.header.source = message.header.source
            proto.header.target = message.header.target
            proto.header.correlation_id = message.header.correlation_id or ""

        # Command
        if message.command:
            proto.command.action = message.command.action
            proto.command.parameters.update(message.command.parameters)
            proto.command.timeout_ms = message.command.timeout_ms
            proto.command.priority = message.command.priority

        # Telemetry
        if message.telemetry:
            proto.telemetry.topic = message.telemetry.topic
            proto.telemetry.data.update(message.telemetry.data)
            proto.telemetry.quality = message.telemetry.quality

        # Event
        if message.event:
            proto.event.type = message.event.event_type
            proto.event.severity = message.event.severity
            proto.event.data.update(message.event.data)

        # Metadata
        if message.metadata:
            proto.metadata.update(message.metadata)

        return proto

    def _message_to_proto_response(self, message: Message) -> bridge_pb2.CommandResponse:
        """Convert Message to CommandResponse protobuf."""
        result_struct = struct_pb2.Struct()

        if message.telemetry and message.telemetry.data and isinstance(
            message.telemetry.data, dict
        ):
            result_struct.update(message.telemetry.data)

        error = ""
        if message.event and message.event.severity == "error":
            error = message.event.data.get("error", "Unknown error")

        return bridge_pb2.CommandResponse(success=error == "", result=result_struct, error=error)

    def _telemetry_to_proto(self, telemetry: Telemetry) -> bridge_pb2.Telemetry:
        """Convert Telemetry to protobuf."""
        return bridge_pb2.Telemetry(
            topic=telemetry.topic,
            data=self._dict_to_struct(telemetry.data) if telemetry.data else struct_pb2.Struct(),
            quality=telemetry.quality,
        )

    def _struct_to_dict(self, struct: struct_pb2.Struct) -> Dict[str, Any]:
        """Convert protobuf Struct to dict."""
        return dict(struct.fields) if struct else {}

    def _dict_to_struct(self, data: Dict[str, Any]) -> struct_pb2.Struct:
        """Convert dict to protobuf Struct."""
        struct = struct_pb2.Struct()
        if data:
            struct.update(data)
        return struct


class GRPCTransport(Transport):
    """gRPC transport implementation with full streaming support."""

    def __init__(self, config: Dict[str, Any]):
        """Initialize gRPC transport with configuration.

        Args:
            config: gRPC server configuration including host, port, TLS settings,
                reflection, and connection management options.
        """
        super().__init__("grpc", config)
        self.host = config.get("host", "0.0.0.0")
        self.port = config.get("port", 50051)
        self.tls_cert = config.get("tls_cert")
        self.tls_key = config.get("tls_key")
        self.ca_cert = config.get("ca_cert")  # For mTLS
        self.reflection = config.get("reflection", True)
        self.max_workers = config.get("max_workers", 10)
        self.keepalive_time_ms = config.get("keepalive_time_ms", 10000)

        self.server: Optional[Server] = None
        self.service: Optional[BridgeServiceServicer] = None
        self._clients: Dict[str, GRPCClient] = {}
        self._client_handlers: Dict[str, Callable] = {}
        self._shutdown_event = asyncio.Event()

    async def start(self) -> bool:
        """Start gRPC server with full configuration."""
        if not GRPC_AVAILABLE:
            logger.error("grpc library not installed. Run: pip install grpcio grpcio-tools")
            return False

        self.service = BridgeServiceServicer(self)

        # Server options
        options = [
            ("grpc.max_send_message_length", 50 * 1024 * 1024),  # 50MB
            ("grpc.max_receive_message_length", 50 * 1024 * 1024),  # 50MB
            ("grpc.keepalive_time_ms", self.keepalive_time_ms),
            ("grpc.keepalive_timeout_ms", 20000),
            ("grpc.http2.max_pings_without_data", 0),
            ("grpc.http2.min_time_between_pings_ms", 10000),
        ]

        # Create gRPC server
        self.server = grpc.aio.server(
            futures.ThreadPoolExecutor(max_workers=self.max_workers), options=options
        )

        # Register service
        bridge_pb2.add_BridgeServiceServicer_to_server(self.service, self.server)
        logger.info("BridgeService registered to gRPC server")

        # Configure TLS/mTLS
        if self.tls_cert and self.tls_key:
            try:
                with open(self.tls_cert, "rb") as f:
                    certificate_chain = f.read()
                with open(self.tls_key, "rb") as f:
                    private_key = f.read()

                if self.ca_cert:
                    # mTLS
                    with open(self.ca_cert, "rb") as f:
                        ca_cert = f.read()
                    credentials = grpc.ssl_server_credentials(
                        ((private_key, certificate_chain),),
                        root_certificates=ca_cert,
                        require_client_auth=True,
                    )
                    logger.info("gRPC mTLS enabled")
                else:
                    # TLS only
                    credentials = grpc.ssl_server_credentials(((private_key, certificate_chain),))
                    logger.info("gRPC TLS enabled")

                self.server.add_secure_port(f"{self.host}:{self.port}", credentials)
            except Exception as e:
                logger.error(f"Failed to load TLS certificates: {e}")
                logger.warning(f"Falling back to insecure port {self.host}:{self.port}")
                self.server.add_insecure_port(f"{self.host}:{self.port}")
        else:
            self.server.add_insecure_port(f"{self.host}:{self.port}")
            logger.info(f"gRPC starting on {self.host}:{self.port} (insecure)")

        # Add reflection for discovery
        if self.reflection:
            try:
                from grpc_reflection.v1alpha import reflection

                SERVICE_NAMES = (
                    bridge_pb2.DESCRIPTOR.services_by_name["BridgeService"].full_name,
                    reflection.SERVICE_NAME,
                )
                reflection.enable_server_reflection(SERVICE_NAMES, self.server)
                logger.info("gRPC reflection enabled")
            except ImportError:
                logger.warning(
                    "grpc-reflection not available, install with: pip install grpcio-reflection"
                )
            except Exception as e:
                logger.warning(f"Failed to enable reflection: {e}")

        await self.server.start()
        self.running = True
        logger.info(f"✓ gRPC transport started on {self.host}:{self.port}")
        return True

    async def stop(self) -> None:
        """Stop gRPC server gracefully."""
        logger.info("Stopping gRPC transport...")

        # Signal shutdown
        self._shutdown_event.set()

        # Disconnect all clients
        for client_id in list(self._clients.keys()):
            self._on_client_disconnect(client_id)

        # Stop server
        if self.server:
            await self.server.stop(grace_period=5)

        self.running = False
        logger.info("✓ gRPC transport stopped")

    async def send(self, message: Message, recipient: str) -> bool:
        """Send message to specific client via their telemetry stream."""
        if not self.service or recipient not in self.service.clients:
            logger.warning(f"Client {recipient} not connected")
            return False

        try:
            proto_telemetry = self.service._telemetry_to_proto(
                Telemetry(
                    topic=message.command.action if message.command else "message",
                    data=message.telemetry.data if message.telemetry else {},
                    quality=1.0,
                )
            )
            await self.service._queue_telemetry(recipient, proto_telemetry)
            return True
        except Exception as e:
            logger.error(f"Failed to send to {recipient}: {e}")
            return False

    async def broadcast(self, message: Message) -> List[str]:
        """Broadcast message to all connected clients."""
        if not self.service:
            return []

        sent_to = []
        self.service._message_to_proto(message)

        # Convert to telemetry format for streaming clients
        telemetry = bridge_pb2.Telemetry(
            topic="broadcast",
            data=self.service._dict_to_struct(
                {
                    "message_id": message.header.message_id,
                    "data": message.telemetry.data if message.telemetry else {},
                }
            ),
            quality=1.0,
        )

        for client_id in list(self.service.clients.keys()):
            try:
                await self.service._queue_telemetry(client_id, telemetry)
                sent_to.append(client_id)
            except Exception as e:
                logger.error(f"Failed to broadcast to {client_id}: {e}")

        return sent_to

    def _on_client_connect(self, client_id: str, identity: Identity):
        """Handle client connection."""
        logger.info(f"Client connected: {client_id} ({identity.name})")

    def _on_client_disconnect(self, client_id: str):
        """Handle client disconnection."""
        logger.info(f"Client disconnected: {client_id}")

    def get_connected_clients(self) -> List[Dict[str, Any]]:
        """Get list of connected clients."""
        if not self.service:
            return []

        return [
            {
                "id": c.client_id,
                "name": c.identity.name,
                "peer": c.peer,
                "subscriptions": list(c.subscriptions),
                "connected_at": c.connected_at,
                "last_activity": c.last_activity,
            }
            for c in self.service.clients.values()
        ]

    def get_stats(self) -> Dict[str, Any]:
        """Get transport statistics."""
        return {
            "running": self.running,
            "host": self.host,
            "port": self.port,
            "tls_enabled": bool(self.tls_cert),
            "mtls_enabled": bool(self.tls_cert and self.ca_cert),
            "connected_clients": len(self.service.clients) if self.service else 0,
            "clients": self.get_connected_clients(),
        }


# Client-side helper for testing
class GRPCClientHelper:
    """Helper class for gRPC client connections."""

    def __init__(self, target: str = "localhost:50051", tls: bool = False):
        """Initialize gRPC client helper.

        Args:
            target: gRPC server address (e.g., "localhost:50051").
            tls: Whether to use TLS encryption.
        """
        self.target = target
        self.tls = tls
        self.channel = None
        self.stub = None

    async def connect(self):
        """Connect to gRPC server."""
        if self.tls:
            credentials = grpc.ssl_channel_credentials()
            self.channel = grpc.aio.secure_channel(self.target, credentials)
        else:
            self.channel = grpc.aio.insecure_channel(self.target)

        self.stub = bridge_pb2.BridgeServiceStub(self.channel)
        logger.info(f"Connected to gRPC server at {self.target}")

    async def send_command(self, action: str, parameters: Dict[str, Any] = None) -> Dict[str, Any]:
        """Send a command to the server."""
        if not self.stub:
            raise RuntimeError("Not connected")

        message = bridge_pb2.Message(
            header=bridge_pb2.Header(message_id=str(uuid.uuid4())),
            command=bridge_pb2.Command(
                action=action, parameters=self._dict_to_struct(parameters or {})
            ),
        )

        response = await self.stub.SendCommand(message)
        return {
            "success": response.success,
            "result": self._struct_to_dict(response.result),
            "error": response.error,
        }

    async def health_check(self) -> Dict[str, Any]:
        """Check server health."""
        if not self.stub:
            raise RuntimeError("Not connected")

        response = await self.stub.HealthCheck(Empty())
        return {"success": response.success, "result": self._struct_to_dict(response.result)}

    async def subscribe_telemetry(
        self, topics: List[str], robot_id: str = ""
    ) -> AsyncIterator[Telemetry]:
        """Subscribe to telemetry stream."""
        if not self.stub:
            raise RuntimeError("Not connected")

        request = bridge_pb2.SubscriptionRequest(topics=topics, robot_id=robot_id)

        async for proto_telemetry in self.stub.SubscribeTelemetry(request):
            yield Telemetry(
                topic=proto_telemetry.topic,
                data=self._struct_to_dict(proto_telemetry.data),
                quality=proto_telemetry.quality,
            )

    def _dict_to_struct(self, data: Dict[str, Any]) -> struct_pb2.Struct:
        """Convert dict to protobuf Struct."""
        struct = struct_pb2.Struct()
        if data:
            struct.update(data)
        return struct

    def _struct_to_dict(self, struct: struct_pb2.Struct) -> Dict[str, Any]:
        """Convert protobuf Struct to dict."""
        return dict(struct.fields) if struct else {}

    async def close(self):
        """Close the connection."""
        if self.channel:
            await self.channel.close()


# Example usage
async def example_server():
    """Example gRPC server with full features."""
    from agent_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector
    from agent_ros_bridge.gateway_v2.core import Bridge

    # Create bridge
    gateway = Bridge()

    # Register ROS2 connector
    ros2 = ROS2Connector()
    gateway.connector_registry.register(ros2)

    # Create gRPC transport with TLS
    grpc_transport = GRPCTransport(
        {
            "host": "0.0.0.0",
            "port": 50051,
            "reflection": True,
            # "tls_cert": "/path/to/server.crt",
            # "tls_key": "/path/to/server.key",
            # "ca_cert": "/path/to/ca.crt",  # For mTLS
        }
    )

    gateway.transport_manager.register(grpc_transport)

    # Start gateway
    async with gateway.run():
        logger.info("=" * 60)
        logger.info("gRPC Server running on port 50051")
        logger.info("Features: bidirectional streaming, telemetry subscription")
        logger.info("=" * 60)

        # Print stats periodically
        while True:
            await asyncio.sleep(30)
            stats = grpc_transport.get_stats()
            logger.info(f"Connected clients: {stats['connected_clients']}")


if __name__ == "__main__":
    asyncio.run(example_server())
