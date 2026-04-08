"""Metrics collection for simulation runs."""

from dataclasses import dataclass, field
from typing import Any


@dataclass
class MetricsCollector:
    """Collects metrics during simulation."""
    
    collisions: int = 0
    goals_reached: int = 0
    path_deviations: list[float] = field(default_factory=list)
    timestamps: dict[str, float] = field(default_factory=dict)
    
    def record_collision(self, timestamp: float, location: tuple[float, float]) -> None:
        """Record a collision event."""
        self.collisions += 1
        self.timestamps[f"collision_{self.collisions}"] = timestamp
    
    def record_goal_reached(self, timestamp: float, duration: float) -> None:
        """Record goal reached event."""
        self.goals_reached += 1
        self.timestamps[f"goal_{self.goals_reached}"] = timestamp
    
    def record_path_deviation(self, distance: float) -> None:
        """Record path deviation."""
        self.path_deviations.append(distance)
    
    def get_metrics(self) -> dict[str, Any]:
        """Get collected metrics."""
        return {
            "collisions": self.collisions,
            "goals_reached": self.goals_reached,
            "path_deviations": len(self.path_deviations),
        }
    
    def get_path_deviation_stats(self) -> dict[str, float]:
        """Get path deviation statistics."""
        if not self.path_deviations:
            return {"mean": 0.0, "max": 0.0, "min": 0.0}
        
        return {
            "mean": sum(self.path_deviations) / len(self.path_deviations),
            "max": max(self.path_deviations),
            "min": min(self.path_deviations),
        }
