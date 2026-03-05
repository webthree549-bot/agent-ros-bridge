"""Fleet intelligence for multi-robot coordination.

Provides spatial reasoning, robot selection, and coordination algorithms
for managing multiple robots.

This fulfills SKILL promises like:
- "Which robot is closest to the kitchen?"
- "Send the best robot to deliver this"
- "Search the building together"
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any


class RobotStatus(Enum):
    """Robot operational status."""

    IDLE = "idle"
    BUSY = "busy"
    CHARGING = "charging"
    ERROR = "error"
    OFFLINE = "offline"


@dataclass
class RobotCapabilities:
    """Capabilities of a robot."""

    can_navigate: bool = True
    can_manipulate: bool = False
    can_carry: bool = True
    max_payload: float = 0.0  # kg
    max_speed: float = 1.0  # m/s


@dataclass
class RobotState:
    """Current state of a robot."""

    robot_id: str
    name: str
    status: RobotStatus
    position: dict[str, float]  # x, y, z
    battery_percent: float
    capabilities: RobotCapabilities
    current_task: str | None = None
    estimated_completion: float | None = None  # seconds


class FleetIntelligence:
    """Intelligent fleet management and coordination.

    Provides:
    - Spatial reasoning (distances, closest robot)
    - Multi-criteria robot selection
    - Task allocation algorithms
    - Coordination patterns
    """

    def __init__(self):
        """Initialize fleet intelligence."""
        self.robot_states: dict[str, RobotState] = {}
        self.known_locations: dict[str, dict[str, float]] = {}

    def update_robot_state(self, state: RobotState):
        """Update state for a robot.

        Args:
            state: Current robot state
        """
        self.robot_states[state.robot_id] = state

    def learn_location(self, name: str, coordinates: dict[str, float]):
        """Learn a named location for spatial reasoning.

        Args:
            name: Location name
            coordinates: x, y, z coordinates
        """
        self.known_locations[name.lower()] = coordinates

    def calculate_distance(self, pos1: dict[str, float], pos2: dict[str, float]) -> float:
        """Calculate Euclidean distance between two positions.

        Args:
            pos1: First position {x, y, z}
            pos2: Second position {x, y, z}

        Returns:
            Distance in meters
        """
        x1, y1 = pos1.get("x", 0), pos1.get("y", 0)
        x2, y2 = pos2.get("x", 0), pos2.get("y", 0)
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_distance_to_location(self, robot_id: str, location_name: str) -> float | None:
        """Get distance from robot to a named location.

        Args:
            robot_id: Robot identifier
            location_name: Named location

        Returns:
            Distance in meters or None if unknown
        """
        if robot_id not in self.robot_states:
            return None

        if location_name.lower() not in self.known_locations:
            return None

        robot_pos = self.robot_states[robot_id].position
        location_pos = self.known_locations[location_name.lower()]

        return self.calculate_distance(robot_pos, location_pos)

    def select_best_robot(self, task: dict[str, Any], criteria: list[str] = None) -> str | None:
        """Select the best robot for a task.

        Args:
            task: Task description with requirements
            criteria: Selection criteria (distance, battery, availability)

        Returns:
            Best robot_id or None if no suitable robot

        Example:
            >>> fleet.select_best_robot(
            ...     {"type": "navigate", "to": "kitchen"},
            ...     criteria=["distance", "battery"]
            ... )
            'robot-2'
        """
        if criteria is None:
            criteria = ["availability", "distance", "battery"]

        # Filter available robots
        available = [r for r in self.robot_states.values() if r.status == RobotStatus.IDLE]

        if not available:
            return None

        # Score each robot
        scores = {}
        for robot in available:
            score = 0.0

            for criterion in criteria:
                if criterion == "distance":
                    score += self._score_distance(robot, task)
                elif criterion == "battery":
                    score += self._score_battery(robot)
                elif criterion == "availability":
                    score += self._score_availability(robot)
                elif criterion == "capability":
                    score += self._score_capability(robot, task)

            scores[robot.robot_id] = score

        # Return highest scoring robot
        if scores:
            return max(scores, key=scores.get)
        return None

    def _score_distance(self, robot: RobotState, task: dict) -> float:
        """Score based on distance to task location.

        Returns higher score for closer robots.
        """
        target = task.get("to") or task.get("location")
        if not target:
            return 0.5  # Neutral if no target

        if isinstance(target, str):
            # Named location
            distance = self.get_distance_to_location(robot.robot_id, target)
        else:
            # Coordinates
            distance = self.calculate_distance(robot.position, target)

        if distance is None:
            return 0.0

        # Closer = higher score (inverse relationship)
        # Max reasonable distance: 100m
        return max(0, 1.0 - (distance / 100.0))

    def _score_battery(self, robot: RobotState) -> float:
        """Score based on battery level.

        Returns higher score for robots with more battery.
        """
        # Linear score: 100% = 1.0, 0% = 0.0
        return robot.battery_percent / 100.0

    def _score_availability(self, robot: RobotState) -> float:
        """Score based on availability.

        Returns 1.0 for idle, 0.0 for busy/error.
        """
        if robot.status == RobotStatus.IDLE:
            return 1.0
        elif robot.status == RobotStatus.CHARGING:
            return 0.5
        else:
            return 0.0

    def _score_capability(self, robot: RobotState, task: dict) -> float:
        """Score based on task capability match.

        Returns higher score if robot can perform the task.
        """
        task_type = task.get("type", "navigate")
        caps = robot.capabilities

        if (
            task_type == "navigate"
            and caps.can_navigate
            or task_type == "manipulate"
            and caps.can_manipulate
        ):
            return 1.0
        elif task_type == "carry" and caps.can_carry:
            payload = task.get("payload", 0)
            if payload <= caps.max_payload:
                return 1.0
            else:
                return 0.3  # Can carry but over capacity

        return 0.0

    def find_closest_robot(self, location: str) -> tuple[str, float] | None:
        """Find the closest robot to a location.

        Args:
            location: Named location or coordinates

        Returns:
            Tuple of (robot_id, distance) or None
        """
        distances = {}

        for robot_id in self.robot_states:
            if isinstance(location, str):
                dist = self.get_distance_to_location(robot_id, location)
            else:
                robot_pos = self.robot_states[robot_id].position
                dist = self.calculate_distance(robot_pos, location)

            if dist is not None:
                distances[robot_id] = dist

        if not distances:
            return None

        closest = min(distances, key=distances.get)
        return (closest, distances[closest])

    def get_robot_with_most_battery(self) -> str | None:
        """Find robot with highest battery.

        Returns:
            Robot ID or None
        """
        if not self.robot_states:
            return None

        return max(self.robot_states.values(), key=lambda r: r.battery_percent).robot_id

    def allocate_task(self, task: dict[str, Any]) -> dict[str, Any]:
        """Allocate a task to the best robot.

        Args:
            task: Task description

        Returns:
            Allocation result with robot_id and confidence
        """
        robot_id = self.select_best_robot(task)

        if robot_id is None:
            return {
                "success": False,
                "error": "No suitable robot available",
                "suggestion": "Wait for a robot to become idle or check fleet status",
            }

        robot = self.robot_states[robot_id]

        return {
            "success": True,
            "robot_id": robot_id,
            "robot_name": robot.name,
            "confidence": "high",
            "estimated_time": self._estimate_task_time(robot, task),
            "reason": "Selected based on proximity and battery level",
        }

    def _estimate_task_time(self, robot: RobotState, task: dict) -> float:
        """Estimate time to complete task.

        Args:
            robot: Robot state
            task: Task description

        Returns:
            Estimated time in seconds
        """
        # Base estimate
        time_estimate = 30.0  # seconds

        # Add travel time if navigation involved
        target = task.get("to") or task.get("location")
        if target:
            if isinstance(target, str):
                distance = self.get_distance_to_location(robot.robot_id, target)
            else:
                distance = self.calculate_distance(robot.position, target)

            if distance:
                # Travel time = distance / speed
                speed = robot.capabilities.max_speed
                time_estimate += distance / speed

        return time_estimate

    def plan_coordination(self, task_type: str, robots: list[str] = None) -> dict[str, Any]:
        """Plan multi-robot coordination.

        Args:
            task_type: Type of coordination (search, convoy, patrol)
            robots: List of robot IDs to coordinate (None = all available)

        Returns:
            Coordination plan
        """
        if robots is None:
            robots = [
                r.robot_id for r in self.robot_states.values() if r.status == RobotStatus.IDLE
            ]

        if task_type == "search":
            return self._plan_search(robots)
        elif task_type == "convoy":
            return self._plan_convoy(robots)
        elif task_type == "patrol":
            return self._plan_patrol(robots)
        else:
            return {"error": f"Unknown coordination type: {task_type}"}

    def _plan_search(self, robots: list[str]) -> dict[str, Any]:
        """Plan parallel search pattern."""
        # Divide area among robots
        n = len(robots)
        sectors = []

        for i, robot_id in enumerate(robots):
            sector = {"robot_id": robot_id, "sector": i + 1, "of": n, "pattern": "systematic"}
            sectors.append(sector)

        return {
            "type": "search",
            "robots": robots,
            "strategy": "parallel_sector_search",
            "sectors": sectors,
        }

    def _plan_convoy(self, robots: list[str]) -> dict[str, Any]:
        """Plan convoy formation."""
        if len(robots) < 2:
            return {"error": "Convoy requires at least 2 robots"}

        return {
            "type": "convoy",
            "leader": robots[0],
            "followers": robots[1:],
            "formation": "follow_the_leader",
            "spacing_meters": 2.0,
        }

    def _plan_patrol(self, robots: list[str]) -> dict[str, Any]:
        """Plan patrol rotation."""
        return {
            "type": "patrol",
            "robots": robots,
            "strategy": "rotation",
            "shift_duration_minutes": 30,
        }

    def get_fleet_summary(self) -> dict[str, Any]:
        """Get summary of fleet status.

        Returns:
            Fleet summary statistics
        """
        total = len(self.robot_states)
        by_status = {}

        for robot in self.robot_states.values():
            status = robot.status.value
            by_status[status] = by_status.get(status, 0) + 1

        avg_battery = (
            sum(r.battery_percent for r in self.robot_states.values()) / total if total > 0 else 0
        )

        return {
            "total_robots": total,
            "by_status": by_status,
            "available": by_status.get("idle", 0),
            "average_battery": round(avg_battery, 1),
            "ready_for_task": by_status.get("idle", 0) > 0,
        }


# Convenience functions for common queries


def find_closest_robot_to_location(fleet: FleetIntelligence, location: str) -> str | None:
    """Find closest robot to a location.

    Convenience function for SKILL fulfillment.
    """
    result = fleet.find_closest_robot(location)
    return result[0] if result else None


def find_robot_with_most_battery(fleet: FleetIntelligence) -> str | None:
    """Find robot with highest battery.

    Convenience function for SKILL fulfillment.
    """
    return fleet.get_robot_with_most_battery()


def send_best_robot(fleet: FleetIntelligence, task: dict) -> dict[str, Any]:
    """Send the best robot for a task.

    Convenience function for SKILL fulfillment.
    """
    return fleet.allocate_task(task)
