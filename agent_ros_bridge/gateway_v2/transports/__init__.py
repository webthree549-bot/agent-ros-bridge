"""Transport implementations for various protocols.

This package provides transport layer implementations for:
    - WebSocket: For browser-based clients and dashboards
    - gRPC: For high-performance microservices
    - MQTT: For IoT device integration

Example:
    from agent_ros_bridge.gateway_v2.transports import WebSocketTransport

    ws = WebSocketTransport({"port": 8765})
"""

from agent_ros_bridge.gateway_v2.transports.mqtt_transport import MQTTTransport
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

# gRPC is optional - only import if protobuf is available
try:
    from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

    __all__ = ["WebSocketTransport", "GRPCTransport", "MQTTTransport"]
except ImportError:
    __all__ = ["WebSocketTransport", "MQTTTransport"]
