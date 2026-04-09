"""Robot memory system for learning from experience."""

from dataclasses import dataclass, field
from datetime import datetime
from typing import Any


@dataclass
class RobotMemory:
    """Memory system for robots to learn from past experiences."""

    robot_id: str
    paths: dict[str, Any] = field(default_factory=dict)
    failures: list[dict[str, Any]] = field(default_factory=list)
    risky_locations: set[str] = field(default_factory=set)

    def store_path(self, path: dict[str, Any]) -> None:
        """Store a successful path."""
        key = f"{path['start']}:{path['end']}"

        if key not in self.paths:
            self.paths[key] = []

        self.paths[key].append(
            {
                **path,
                "timestamp": datetime.now().isoformat(),
            }
        )

    def get_path(self, start: str, end: str) -> dict[str, Any] | None:
        """Get the best path between two locations."""
        key = f"{start}:{end}"

        if key not in self.paths or not self.paths[key]:
            return None

        # Return the fastest successful path
        paths = [p for p in self.paths[key] if p.get("success", False)]
        if not paths:
            return None

        return min(paths, key=lambda p: p.get("duration", float("inf")))

    def get_all_paths(self, start: str, end: str) -> list[dict[str, Any]]:
        """Get all known paths between locations."""
        key = f"{start}:{end}"
        return self.paths.get(key, [])

    def record_failure(self, task: str, location: str, reason: str) -> None:
        """Record a failure for learning."""
        self.failures.append(
            {
                "task": task,
                "location": location,
                "reason": reason,
                "timestamp": datetime.now().isoformat(),
            }
        )

        # Mark location as risky
        if reason in ["collision", "blocked", "unsafe"]:
            self.risky_locations.add(location)

    def is_location_risky(self, location: str) -> bool:
        """Check if a location is known to be risky."""
        return location in self.risky_locations

    def suggest_path(self, start: str, end: str) -> dict[str, Any] | None:
        """Suggest the best path based on learned experience."""
        return self.get_path(start, end)

    def update_path_status(self, via: str, blocked: bool = False) -> None:
        """Update path status (e.g., when corridor becomes blocked)."""
        # Remove paths that go through blocked locations
        if blocked:
            for key in list(self.paths.keys()):
                self.paths[key] = [p for p in self.paths[key] if p.get("via") != via]
