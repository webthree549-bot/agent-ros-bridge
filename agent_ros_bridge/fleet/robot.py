"""Fleet robot management."""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any


class RobotStatus(Enum):
    """Robot operational status."""

    IDLE = auto()
    BUSY = auto()
    RETURNING = auto()
    CHARGING = auto()
    ERROR = auto()
    OFFLINE = auto()


@dataclass
class RobotCapabilities:
    """Robot capabilities."""

    can_navigate: bool = True
    can_manipulate: bool = False
    can_lift: bool = False
    max_payload_kg: float = 0.0
    max_speed_ms: float = 1.0


@dataclass
class FleetRobot:
    """Represents a robot in the fleet."""

    robot_id: str
    name: str
    status: RobotStatus = RobotStatus.IDLE
    battery_percent: float = 100.0
    current_location: str | None = None
    current_task: str | None = None
    capabilities: RobotCapabilities = field(default_factory=RobotCapabilities)
    metadata: dict[str, Any] = field(default_factory=dict)

    def is_available(self) -> bool:
        """Check if robot is available for tasks."""
        return self.status == RobotStatus.IDLE and self.battery_percent > 20

    def needs_charging(self) -> bool:
        """Check if robot needs charging."""
        return self.battery_percent < 20

    def can_handle_task(self, task_type: str) -> bool:
        """Check if robot can handle a task type."""
        if task_type == "navigate":
            return self.capabilities.can_navigate
        elif task_type in ["manipulate", "pick", "place"]:
            return self.capabilities.can_manipulate
        elif task_type == "transport":
            return self.capabilities.can_lift
        return True
