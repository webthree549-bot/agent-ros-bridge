"""Task management for fleet operations."""

from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum, auto
from typing import Any


class TaskStatus(Enum):
    """Task execution status."""

    PENDING = auto()
    ASSIGNED = auto()
    EXECUTING = auto()
    COMPLETED = auto()
    FAILED = auto()
    CANCELLED = auto()


@dataclass
class Task:
    """Represents a task to be executed by a robot."""

    id: str
    type: str
    target_location: str | None = None
    priority: int = 5  # 1-10, lower is higher priority
    status: TaskStatus = TaskStatus.PENDING
    assigned_robot: str | None = None
    dependencies: list[str] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)
    created_at: datetime = field(default_factory=datetime.now)
    started_at: datetime | None = None
    completed_at: datetime | None = None

    def __lt__(self, other: "Task") -> bool:
        """Compare tasks by priority (lower number = higher priority)."""
        return self.priority < other.priority

    def __eq__(self, other: object) -> bool:
        """Check if tasks have equal priority."""
        if not isinstance(other, Task):
            return NotImplemented
        return self.priority == other.priority

    def mark_assigned(self, robot_id: str) -> None:
        """Mark task as assigned to a robot."""
        self.status = TaskStatus.ASSIGNED
        self.assigned_robot = robot_id

    def mark_executing(self) -> None:
        """Mark task as executing."""
        self.status = TaskStatus.EXECUTING
        self.started_at = datetime.now()

    def mark_completed(self) -> None:
        """Mark task as completed."""
        self.status = TaskStatus.COMPLETED
        self.completed_at = datetime.now()

    def mark_failed(self, reason: str = "") -> None:
        """Mark task as failed."""
        self.status = TaskStatus.FAILED
        self.metadata["failure_reason"] = reason

    def mark_cancelled(self) -> None:
        """Mark task as cancelled."""
        self.status = TaskStatus.CANCELLED

    @property
    def duration_seconds(self) -> float | None:
        """Get task duration if completed."""
        if self.started_at and self.completed_at:
            return (self.completed_at - self.started_at).total_seconds()
        return None
