#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gRPC Transport for Agent ROS Bridge

High-performance gRPC transport for microservices, cloud deployments,
and strongly-typed integrations.
"""

import asyncio
import logging
from typing import Dict, Any, Optional
from concurrent import futures
from datetime import datetime
from google.protobuf import struct_pb2

try:
    import grpc
    from google.protobuf import empty_pb2
    from agent_ros_bridge.proto import bridge_pb2
    GRPC_AVAILABLE = True
except ImportError as e:
    GRPC_AVAILABLE = False
    logging.debug(f"gRPC not available: {e}")

from agent_ros_bridge.gateway_v2.core import (
    Transport, Message, Identity, Header, Command, Telemetry, Event
)

logger = logging.getLogger("transport.grpc")


class BridgeServiceServicer(bridge_pb2.BridgeServiceServicer if GRPC_AVAILABLE else object):
    """Bridge service implementation"""
    
    def __init__(self, message_handler, transport):
        self.message_handler = message_handler
        self.transport = transport
    
    async def SendCommand(self, request, context):
        """Handle incoming command"""
        try:
            identity = Identity(
                id=str(context.peer()),
                name=f"grpc_{context.peer()}",
                roles=["authenticated"],
                metadata={"transport": "grpc", "method": "SendCommand"}
            )
            
            message = self._proto_to_message(request)
            
            if self.message_handler:
                response = await self.message_handler(message, identity)
                if response:
                    return self._message_to_proto_response(response)
            
            return bridge_pb2.CommandResponse(
                success=False,
                error="No handler registered"
            )
        except Exception as e:
            logger.error(f"Error handling command: {e}")
            return bridge_pb2.CommandResponse(
                success=False,
                error=str(e)
            )
    
    async def StreamTelemetry(self, request_iterator, context):
        """Bidirectional streaming for telemetry"""
        identity = Identity(
            id=str(context.peer()),
            name=f"grpc_{context.peer()}",
            roles=["authenticated"],
            metadata={"transport": "grpc", "method": "StreamTelemetry"}
        )
        
        async for request in request_iterator:
            try:
                message = self._proto_to_message(request)
                
                if self.message_handler:
                    response = await self.message_handler(message, identity)
                    if response:
                        yield self._message_to_proto_response(response)
            except Exception as e:
                logger.error(f"Error in telemetry stream: {e}")
                break
    
    async def SubscribeTelemetry(self, request, context):
        """Subscribe to telemetry stream"""
        # This would typically connect to a telemetry publisher
        # For now, yield a placeholder
        yield bridge_pb2.Telemetry(
            topic="status",
            data=struct_pb2.Struct(),
            quality=1.0
        )
    
    async def HealthCheck(self, request, context):
        """Health check endpoint"""
        return bridge_pb2.CommandResponse(
            success=True,
            result=struct_pb2.Struct()
        )
    
    def _proto_to_message(self, proto) -> Message:
        """Convert protobuf to Message"""
        header = Header()
        if proto.header and proto.header.message_id:
            header.message_id = proto.header.message_id
            header.source = proto.header.source
            header.target = proto.header.target
        
        command = None
        if proto.command and proto.command.action:
            params = {}
            if proto.command.parameters:
                params = dict(proto.command.parameters.fields)
            command = Command(
                action=proto.command.action,
                parameters=params,
                timeout_ms=proto.command.timeout_ms,
                priority=proto.command.priority
            )
        
        return Message(header=header, command=command)
    
    def _message_to_proto_response(self, message: Message) -> bridge_pb2.CommandResponse:
        """Convert Message to CommandResponse protobuf"""
        result_struct = struct_pb2.Struct()
        
        if message.telemetry and message.telemetry.data:
            if isinstance(message.telemetry.data, dict):
                result_struct.update(message.telemetry.data)
        
        return bridge_pb2.CommandResponse(
            success=True,
            result=result_struct,
            error=""
        )


class GRPCTransport(Transport):
    """gRPC transport implementation"""
    
    def __init__(self, config: Dict[str, Any]):
        super().__init__("grpc", config)
        self.host = config.get("host", "0.0.0.0")
        self.port = config.get("port", 50051)
        self.tls_cert = config.get("tls_cert")
        self.tls_key = config.get("tls_key")
        self.reflection = config.get("reflection", True)
        self.server = None
        self.service = None
    
    async def start(self) -> bool:
        """Start gRPC server"""
        if not GRPC_AVAILABLE:
            logger.error("grpc library not installed. Run: pip install grpcio grpcio-tools")
            return False
        
        self.service = BridgeServiceServicer(self.message_handler, self)
        
        # Create gRPC server
        self.server = grpc.aio.server(futures.ThreadPoolExecutor(max_workers=10))
        
        # Register service to server
        bridge_pb2.add_BridgeServiceServicer_to_server(self.service, self.server)
        logger.info("BridgeService registered to gRPC server")
        
        # Bind to port
        if self.tls_cert and self.tls_key:
            try:
                with open(self.tls_cert, 'rb') as f:
                    certificate_chain = f.read()
                with open(self.tls_key, 'rb') as f:
                    private_key = f.read()
                
                credentials = grpc.ssl_server_credentials(
                    ((private_key, certificate_chain),)
                )
                self.server.add_secure_port(f"{self.host}:{self.port}", credentials)
                logger.info(f"gRPC TLS enabled on {self.host}:{self.port}")
            except Exception as e:
                logger.error(f"Failed to load TLS certificates: {e}")
                logger.info(f"Falling back to insecure port {self.host}:{self.port}")
                self.server.add_insecure_port(f"{self.host}:{self.port}")
        else:
            self.server.add_insecure_port(f"{self.host}:{self.port}")
            logger.info(f"gRPC starting on {self.host}:{self.port}")
        
        # Add reflection for discovery
        if self.reflection:
            try:
                from grpc_reflection.v1alpha import reflection
                SERVICE_NAMES = (
                    bridge_pb2.SERVICE_NAME,
                    reflection.SERVICE_NAME,
                )
                reflection.enable_server_reflection(SERVICE_NAMES, self.server)
                logger.info("gRPC reflection enabled")
            except ImportError:
                logger.warning("grpc-reflection not available, install with: pip install grpcio-reflection")
        
        await self.server.start()
        self.running = True
        logger.info(f"gRPC transport started successfully")
        return True
    
    async def stop(self) -> None:
        """Stop gRPC server"""
        if self.server:
            await self.server.stop(grace_period=5)
        
        self.running = False
        logger.info("gRPC transport stopped")
    
    async def send(self, message: Message, recipient: str) -> bool:
        """Send message to specific recipient"""
        logger.warning("gRPC send() not implemented - gRPC uses request/response pattern")
        return False
    
    async def broadcast(self, message: Message) -> list:
        """Broadcast to all connected clients"""
        logger.warning("gRPC broadcast() not implemented - gRPC is connection-oriented")
        return []


# Example usage
async def example_server():
    """Example gRPC server"""
    from agent_ros_bridge.gateway_v2.core import Bridge
    
    gateway = Bridge()
    
    # Create gRPC transport
    grpc_transport = GRPCTransport({
        "host": "0.0.0.0",
        "port": 50051,
        "reflection": True
    })
    
    gateway.transport_manager.register(grpc_transport)
    
    async with gateway.run():
        logger.info("gRPC server running on port 50051")
        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    import asyncio
    asyncio.run(example_server())
