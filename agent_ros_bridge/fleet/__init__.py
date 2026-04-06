"""Fleet management module for Agent ROS Bridge.

Multi-robot orchestration, task allocation, and fleet coordination.

Note: This module is now available as a separate package:
    pip install agent-ros-bridge-fleet
    from agent_ros_bridge_fleet import FleetOrchestrator

This module provides backward compatibility imports.
"""

# Try to import from the new standalone package
try:
    from agent_ros_bridge_fleet import (
        FleetMetrics,
        FleetOrchestrator,
        FleetRobot,
        RobotCapability,
        RobotStatus,
        Task,
        TaskStatus,
    )
except ImportError:
    # Fallback to local imports (for development without the package installed)
    from .orchestrator import (
        FleetMetrics,
        FleetOrchestrator,
        FleetRobot,
        RobotCapability,
        RobotStatus,
        Task,
        TaskStatus,
    )

__all__ = [
    "FleetOrchestrator",
    "FleetRobot",
    "RobotCapability",
    "Task",
    "TaskStatus",
    "RobotStatus",
    "FleetMetrics",
]
