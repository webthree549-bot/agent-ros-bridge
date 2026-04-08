"""Action types and data models for Agent ROS Bridge."""

from dataclasses import dataclass, field
from datetime import UTC, datetime
from enum import Enum, auto
from typing import Any


class ActionStatus(Enum):
    """Action client status."""

    IDLE = auto()
    PENDING = auto()  # Goal sent, waiting for acceptance
    ACTIVE = auto()  # Goal accepted and executing
    PREEMPTING = auto()  # Cancel requested
    SUCCEEDED = auto()  # Goal completed successfully
    ABORTED = auto()  # Goal aborted by server
    REJECTED = auto()  # Goal rejected by server
    CANCELED = auto()  # Goal canceled successfully
    LOST = auto()  # Action server lost


@dataclass
class ActionGoal:
    """Action goal definition."""

    goal_id: str
    goal_data: dict[str, Any]
    action_type: str  # e.g., "nav2_msgs/action/NavigateToPose"
    timeout_sec: float = 30.0
    created_at: datetime = field(default_factory=lambda: datetime.now(UTC))


@dataclass
class ActionFeedback:
    """Action feedback during execution."""

    goal_id: str
    feedback_data: dict[str, Any]
    timestamp: datetime = field(default_factory=lambda: datetime.now(UTC))


@dataclass
class ActionResult:
    """Action final result."""

    goal_id: str
    success: bool
    status: ActionStatus
    result_data: dict[str, Any] = field(default_factory=dict)
    error_message: str | None = None
    execution_time_sec: float = 0.0
