# -*- coding: utf-8 -*-
"""Generated Python gRPC code for agent_ros_bridge.proto
This is a simplified implementation - full protoc generation recommended for production.
"""

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
import grpc
from google.protobuf import struct_pb2
from google.protobuf import timestamp_pb2
from google.protobuf import empty_pb2

# Symbol database
_sym_db = _symbol_database.Default()

# Message definitions
class Header(_message.Message):
    """Message header"""
    __slots__ = ('message_id', 'timestamp', 'source', 'target', 'correlation_id')
    
    def __init__(self, message_id=None, timestamp=None, source=None, target=None, correlation_id=None):
        self.message_id = message_id or ""
        self.timestamp = timestamp
        self.source = source or ""
        self.target = target or ""
        self.correlation_id = correlation_id or ""

class Command(_message.Message):
    """Robot command"""
    __slots__ = ('action', 'parameters', 'timeout_ms', 'priority')
    
    def __init__(self, action=None, parameters=None, timeout_ms=None, priority=None):
        self.action = action or ""
        self.parameters = parameters or struct_pb2.Struct()
        self.timeout_ms = timeout_ms or 5000
        self.priority = priority or 5

class Telemetry(_message.Message):
    """Telemetry data"""
    __slots__ = ('topic', 'data', 'quality')
    
    def __init__(self, topic=None, data=None, quality=None):
        self.topic = topic or ""
        self.data = data or struct_pb2.Struct()
        self.quality = quality or 1.0

class Event(_message.Message):
    """System event"""
    __slots__ = ('type', 'severity', 'data')
    
    def __init__(self, type=None, severity=None, data=None):
        self.type = type or ""
        self.severity = severity or "info"
        self.data = data or struct_pb2.Struct()

class Message(_message.Message):
    """Unified message"""
    __slots__ = ('header', 'command', 'telemetry', 'event', 'metadata')
    
    def __init__(self, header=None, command=None, telemetry=None, event=None, metadata=None):
        self.header = header or Header()
        self.command = command
        self.telemetry = telemetry
        self.event = event
        self.metadata = metadata or struct_pb2.Struct()

class CommandResponse(_message.Message):
    """Command response"""
    __slots__ = ('success', 'result', 'error')
    
    def __init__(self, success=None, result=None, error=None):
        self.success = success or False
        self.result = result or struct_pb2.Struct()
        self.error = error or ""

class SubscriptionRequest(_message.Message):
    """Subscription request"""
    __slots__ = ('topics', 'robot_id')
    
    def __init__(self, topics=None, robot_id=None):
        self.topics = topics or []
        self.robot_id = robot_id or ""


# gRPC Service
class BridgeServiceServicer(object):
    """Bridge service implementation interface"""
    
    def SendCommand(self, request, context):
        """Send a command to the bridge"""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')
    
    def StreamTelemetry(self, request_iterator, context):
        """Bidirectional telemetry streaming"""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')
    
    def SubscribeTelemetry(self, request, context):
        """Subscribe to telemetry stream"""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')
    
    def HealthCheck(self, request, context):
        """Health check endpoint"""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


class BridgeServiceStub(object):
    """Bridge service client stub"""
    
    def __init__(self, channel):
        self.SendCommand = channel.unary_unary(
            '/agent_ros_bridge.BridgeService/SendCommand',
            request_serializer=Message.SerializeToString,
            response_deserializer=CommandResponse.FromString,
        )
        self.StreamTelemetry = channel.stream_stream(
            '/agent_ros_bridge.BridgeService/StreamTelemetry',
            request_serializer=Message.SerializeToString,
            response_deserializer=Message.FromString,
        )
        self.SubscribeTelemetry = channel.unary_stream(
            '/agent_ros_bridge.BridgeService/SubscribeTelemetry',
            request_serializer=SubscriptionRequest.SerializeToString,
            response_deserializer=Telemetry.FromString,
        )
        self.HealthCheck = channel.unary_unary(
            '/agent_ros_bridge.BridgeService/HealthCheck',
            request_serializer=empty_pb2.Empty.SerializeToString,
            response_deserializer=CommandResponse.FromString,
        )


def add_BridgeServiceServicer_to_server(servicer, server):
    """Add BridgeService servicer to gRPC server"""
    rpc_method_handlers = {
        'SendCommand': grpc.unary_unary_rpc_method_handler(
            servicer.SendCommand,
            request_deserializer=Message.FromString,
            response_serializer=CommandResponse.SerializeToString,
        ),
        'StreamTelemetry': grpc.stream_stream_rpc_method_handler(
            servicer.StreamTelemetry,
            request_deserializer=Message.FromString,
            response_serializer=Message.SerializeToString,
        ),
        'SubscribeTelemetry': grpc.unary_stream_rpc_method_handler(
            servicer.SubscribeTelemetry,
            request_deserializer=SubscriptionRequest.FromString,
            response_serializer=Telemetry.SerializeToString,
        ),
        'HealthCheck': grpc.unary_unary_rpc_method_handler(
            servicer.HealthCheck,
            request_deserializer=empty_pb2.Empty.FromString,
            response_serializer=CommandResponse.SerializeToString,
        ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
        'agent_ros_bridge.BridgeService', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


# Service name for reflection
SERVICE_NAME = 'agent_ros_bridge.BridgeService'
