"""Agent ROS Bridge v2 - Universal ROS Bridge

Multi-protocol, multi-robot, cloud-native connectivity platform.
"""

__version__ = "0.2.2"

from agent_ros_bridge.gateway_v2.core import (
    Bridge,
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

from agent_ros_bridge.gateway_v2.config import (
    BridgeConfig,
    TransportConfig,
    ConnectorConfig,
    SecurityConfig,
    PluginConfig,
    ConfigLoader,
)

__all__ = [
    "Bridge",
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
    "BridgeConfig",
    "TransportConfig",
    "ConnectorConfig",
    "SecurityConfig",
    "PluginConfig",
    "ConfigLoader",
]
