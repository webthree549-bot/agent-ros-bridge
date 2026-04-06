"""Agent ROS Bridge Fleet - Multi-robot orchestration for production deployments.

This package provides fleet management capabilities for Agent ROS Bridge,
enabling coordination of multiple heterogeneous robots (ROS1/ROS2 mixed).

Example:
    >>> from agent_ros_bridge_fleet import FleetOrchestrator, Task, RobotCapability
    >>> orchestrator = FleetOrchestrator()
    >>> await orchestrator.add_robot(robot)
    >>> await orchestrator.submit_task(Task(type="navigate", target="zone_a"))
"""

__version__ = "0.7.0.dev1"

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
    "RobotStatus",
    "Task",
    "TaskStatus",
    "FleetMetrics",
    "__version__",
]
