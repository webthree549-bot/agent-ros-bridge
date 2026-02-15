"""
Agent ROS Bridge - Universal ROS1/ROS2 bridge for AI agents.

This package provides a multi-protocol, multi-robot gateway enabling
AI agents to control ROS-based robots and embodied intelligence systems.

Example:
    >>> from agent_ros_bridge import Bridge
    >>> from agent_ros_bridge.transports.websocket import WebSocketTransport
    >>> 
    >>> bridge = Bridge()
    >>> bridge.transport_manager.register(WebSocketTransport({"port": 8765}))
    >>> await bridge.start()
"""

__version__ = "0.2.1"
__author__ = "Agent ROS Bridge Team"
__email__ = "dev@agent-ros-bridge.org"

# Import main components for convenience
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

# Fleet management
try:
    from agent_ros_bridge.fleet import (
        FleetOrchestrator,
        FleetRobot,
        RobotCapability,
        Task,
        TaskStatus,
        RobotStatus,
        FleetMetrics,
    )
    FLEET_AVAILABLE = True
except ImportError:
    FLEET_AVAILABLE = False

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

if FLEET_AVAILABLE:
    __all__.extend([
        "FleetOrchestrator",
        "FleetRobot",
        "RobotCapability",
        "Task",
        "TaskStatus",
        "RobotStatus",
        "FleetMetrics",
    ])
