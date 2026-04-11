"""Agent ROS Bridge v2 - Universal ROS Bridge.

Multi-protocol, multi-robot, cloud-native connectivity platform.
"""

__version__ = "0.6.7"

from agent_ros_bridge.gateway_v2.config import (
    BridgeConfig,
    ConfigLoader,
    ConnectorConfig,
    PluginConfig,
    SecurityConfig,
    TransportConfig,
)
from agent_ros_bridge.gateway_v2.core import (
    Bridge,
    Command,
    Connector,
    ConnectorRegistry,
    Event,
    Header,
    Identity,
    Message,
    Plugin,
    PluginManager,
    QoS,
    Robot,
    RobotFleet,
    Telemetry,
    Transport,
    TransportManager,
)

# New: Blueprint and Module patterns (inspired by dimos)
try:
    from agent_ros_bridge.gateway_v2.blueprint import (
        Blueprint,
        Connection,
        ModuleBlueprint,
        RPCDefinition,
        StreamDefinition,
        autoconnect,
        rpc,
        skill,
    )
    from agent_ros_bridge.gateway_v2.module import CompositeModule, In, Module, Out, Stream

    _BLUEPRINT_AVAILABLE = True
except ImportError:
    _BLUEPRINT_AVAILABLE = False

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

# Add blueprint exports if available
if _BLUEPRINT_AVAILABLE:
    __all__.extend(
        [
            "Blueprint",
            "ModuleBlueprint",
            "Connection",
            "StreamDefinition",
            "RPCDefinition",
            "autoconnect",
            "skill",
            "rpc",
            "Module",
            "CompositeModule",
            "Stream",
            "In",
            "Out",
        ]
    )
