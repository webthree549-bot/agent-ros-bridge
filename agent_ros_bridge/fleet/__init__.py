"""Fleet management module for Agent ROS Bridge

Multi-robot orchestration, task allocation, and fleet coordination.
"""

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
