"""Transport implementations for various protocols.

This package provides transport layer implementations for:
    - WebSocket: For browser-based clients and dashboards
    - gRPC: For high-performance microservices
    - MQTT: For IoT device integration
    - LCM: For low-latency, high-performance internal communication
    - Shared Memory: For zero-copy local communication

Example:
    from agent_ros_bridge.gateway_v2.transports import WebSocketTransport

    ws = WebSocketTransport({"port": 8765})
"""

from agent_ros_bridge.gateway_v2.transports.mqtt_transport import MQTTTransport
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

# gRPC is optional - only import if protobuf is available
try:
    from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

    _GRPC_AVAILABLE = True
except ImportError:
    _GRPC_AVAILABLE = False

# LCM is optional - only import if lcm is available
try:
    from agent_ros_bridge.gateway_v2.transports.lcm_transport import (
        LCMMessage,
        LCMPublisher,
        LCMSubscriber,
        LCMTransport,
        SharedMemoryTransport,
    )

    _LCM_AVAILABLE = True
except ImportError:
    _LCM_AVAILABLE = False

# Build __all__ based on available transports
__all__ = ["WebSocketTransport", "MQTTTransport"]

if _GRPC_AVAILABLE:
    __all__.append("GRPCTransport")

if _LCM_AVAILABLE:
    __all__.extend(
        [
            "LCMTransport",
            "SharedMemoryTransport",
            "LCMMessage",
            "LCMPublisher",
            "LCMSubscriber",
        ]
    )
