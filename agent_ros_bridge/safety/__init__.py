"""
Unified Safety Module for Agent ROS Bridge

This module provides comprehensive safety functionality:
- High-level: SafetyManager (confirmation flows, policies)
- Low-level: ROS2 safety nodes (emergency stop, limits, validation)

Usage:
    # High-level safety management
    from agent_ros_bridge.safety import SafetyManager, SafetyLevel
    safety = SafetyManager()
    
    # Low-level ROS2 safety nodes
    from agent_ros_bridge.safety import (
        SafetyLimitsNode,
        SafetyValidatorNode,
        EmergencyStopNode,
        WatchdogNode
    )
"""

# High-level safety manager (from integrations)
from agent_ros_bridge.integrations.safety import (
    SafetyManager,
    SafetyLevel,
    SafetyPolicy,
    ConfirmationRequest,
)

# Low-level ROS2 safety nodes (new in v0.6.0)
try:
    from .limits import SafetyLimitsNode
    from .validator import SafetyValidatorNode
    from .emergency_stop import EmergencyStopNode
    from .watchdog import WatchdogNode
    _ROS2_NODES_AVAILABLE = True
except ImportError:
    # ROS2 not available
    SafetyLimitsNode = None
    SafetyValidatorNode = None
    EmergencyStopNode = None
    WatchdogNode = None
    _ROS2_NODES_AVAILABLE = False

__all__ = [
    # High-level safety
    'SafetyManager',
    'SafetyLevel',
    'SafetyPolicy',
    'ConfirmationRequest',
    # Low-level ROS2 nodes
    'SafetyLimitsNode',
    'SafetyValidatorNode',
    'EmergencyStopNode',
    'WatchdogNode',
]
