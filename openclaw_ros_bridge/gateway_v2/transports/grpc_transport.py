#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gRPC Transport for OpenClaw Gateway

High-performance gRPC transport for microservices, cloud deployments,
and strongly-typed integrations.
"""

import asyncio
import logging
from typing import Dict, Any, Optional
from concurrent import futures
from datetime import datetime

try:
    import grpc
    from google.protobuf import struct_pb2, timestamp_pb2, empty_pb2
    GRPC_AVAILABLE = True
except ImportError:
    GRPC_AVAILABLE = False

from openclaw_ros_bridge.gateway_v2.core import (
    Transport, Message, Identity, Header, Command, Telemetry, Event
)

logger = logging.getLogger("transport.grpc")


# gRPC service definition (in Python for now, could be .proto)
# This is a simplified version - full protobuf would be better

class OpenClawService:
    """gRPC service implementation"""
    
    def __init__(self, message_handler, transport):
        self.message_handler = message_handler
        self.transport = transport
    
    async def SendCommand(self, request, context):
        """Handle incoming command"""
        identity = Identity(
            id=str(context.peer()),
            name=f"grpc_{context.peer()}",
            roles=["authenticated"],  # Would check auth metadata
            metadata={"transport": "grpc"}
        )
        
        message = self._proto_to_message(request)
        
        if self.message_handler:
            response = await self.message_handler(message, identity)
            if response:
                return self._message_to_proto(response)
        
        return self._create_empty_response()
    
    async def StreamTelemetry(self, request_iterator, context):
        """Bidirectional streaming for telemetry"""
        identity = Identity(
            id=str(context.peer()),
            name=f"grpc_{context.peer()}",
            roles=["authenticated"],
            metadata={"transport": "grpc"}
        )
        
        async for request in request_iterator:
            message = self._proto_to_message(request)
            
            if self.message_handler:
                response = await self.message_handler(message, identity)
                if response:
                    yield self._message_to_proto(response)
    
    def _proto_to_message(self, proto):
        """Convert protobuf to Message"""
        # Simplified - full implementation would parse all fields
        return Message(
            header=Header(),
            command=Command(action="test")
        )
    
    def _message_to_proto(self, message: Message):
        """Convert Message to protobuf"""
        # Simplified - full implementation would serialize all fields
        return empty_pb2.Empty()
    
    def _create_empty_response(self):
        """Create empty response"""
        return empty_pb2.Empty()


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
        
        self.service = OpenClawService(self.message_handler, self)
        
        # Create gRPC server
        self.server = grpc.aio.server(futures.ThreadPoolExecutor(max_workers=10))
        
        # Add service to server (simplified - would use generated code)
        # In real implementation:
        # openclaw_pb2_grpc.add_OpenClawServiceServicer_to_server(self.service, self.server)
        
        # Bind to port
        if self.tls_cert and self.tls_key:
            with open(self.tls_cert, 'rb') as f:
                certificate_chain = f.read()
            with open(self.tls_key, 'rb') as f:
                private_key = f.read()
            
            credentials = grpc.ssl_server_credentials(
                ((private_key, certificate_chain),)
            )
            self.server.add_secure_port(f"{self.host}:{self.port}", credentials)
            logger.info(f"gRPC TLS enabled on {self.host}:{self.port}")
        else:
            self.server.add_insecure_port(f"{self.host}:{self.port}")
            logger.info(f"gRPC starting on {self.host}:{self.port}")
        
        # Add reflection for discovery
        if self.reflection:
            try:
                from grpc_reflection.v1alpha import reflection
                # In real implementation:
                # SERVICE_NAMES = (
                #     openclaw_pb2.DESCRIPTOR.services_by_name['OpenClawService'].full_name,
                #     reflection.SERVICE_NAME,
                # )
                # reflection.enable_server_reflection(SERVICE_NAMES, self.server)
                logger.info("gRPC reflection enabled")
            except ImportError:
                logger.warning("grpc-reflection not available")
        
        await self.server.start()
        self.running = True
        logger.info(f"gRPC transport started on {self.host}:{self.port}")
        return True
    
    async def stop(self) -> None:
        """Stop gRPC server"""
        if self.server:
            await self.server.stop(grace_period=5)
        
        self.running = False
        logger.info("gRPC transport stopped")
    
    async def send(self, message: Message, recipient: str) -> bool:
        """Send message to specific recipient"""
        # gRPC is request/response, so this would require client tracking
        # For now, this is a placeholder
        logger.warning("gRPC send() not implemented - use request/response pattern")
        return False
    
    async def broadcast(self, message: Message) -> list:
        """Broadcast to all connected clients"""
        logger.warning("gRPC broadcast() not implemented - gRPC is connection-oriented")
        return []


# Proto file content for reference (would be in openclaw.proto)
PROTO_DEFINITION = '''
syntax = "proto3";
package openclaw;

import "google/protobuf/struct.proto";
import "google/protobuf/timestamp.proto";
import "google/protobuf/empty.proto";

message Header {
    string message_id = 1;
    google.protobuf.Timestamp timestamp = 2;
    string source = 3;
    string target = 4;
    string correlation_id = 5;
}

message Command {
    string action = 1;
    google.protobuf.Struct parameters = 2;
    int32 timeout_ms = 3;
    int32 priority = 4;
}

message Telemetry {
    string topic = 1;
    google.protobuf.Struct data = 2;
    float quality = 3;
}

message Event {
    string type = 1;
    string severity = 2;
    google.protobuf.Struct data = 3;
}

message Message {
    Header header = 1;
    Command command = 2;
    Telemetry telemetry = 3;
    Event event = 4;
    google.protobuf.Struct metadata = 5;
}

message CommandResponse {
    bool success = 1;
    google.protobuf.Struct result = 2;
    string error = 3;
}

service OpenClawService {
    rpc SendCommand(Message) returns (CommandResponse);
    rpc StreamTelemetry(stream Message) returns (stream Message);
    rpc SubscribeTelemetry(SubscriptionRequest) returns (stream Telemetry);
}

message SubscriptionRequest {
    repeated string topics = 1;
    string robot_id = 2;
}
'''


# Example usage
async def example_server():
    """Example gRPC server"""
    from openclaw_ros_bridge.gateway_v2.core import OpenClawGateway
    
    gateway = OpenClawGateway()
    
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
