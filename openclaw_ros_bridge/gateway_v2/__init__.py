"""OpenClaw Gateway v2 - Universal Robot Gateway

Multi-protocol, multi-robot, cloud-native robot connectivity platform.
"""

__version__ = "2.0.0"

from openclaw_ros_bridge.gateway_v2.core import (
    OpenClawGateway,
    Transport,
    TransportManager,
    Connector,
    ConnectorRegistry,
    Robot,
    RobotFleet,
    Plugin,
    PluginManager,
    Message,
    Header,
    Command,
    Telemetry,
    Event,
    Identity,
    QoS,
)

from openclaw_ros_bridge.gateway_v2.config import (
    GatewayConfig,
    TransportConfig,
    ConnectorConfig,
    SecurityConfig,
    PluginConfig,
    ConfigLoader,
)

__all__ = [
    "OpenClawGateway",
    "Transport",
    "TransportManager",
    "Connector",
    "ConnectorRegistry",
    "Robot",
    "RobotFleet",
    "Plugin",
    "PluginManager",
    "Message",
    "Header",
    "Command",
    "Telemetry",
    "Event",
    "Identity",
    "QoS",
    "GatewayConfig",
    "TransportConfig",
    "ConnectorConfig",
    "SecurityConfig",
    "PluginConfig",
    "ConfigLoader",
]
