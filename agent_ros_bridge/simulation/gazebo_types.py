"""Data types for Gazebo batch simulation.

Contains dataclasses and type definitions used by GazeboBatchRunner.
"""

import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class WorldConfig:
    """Configuration for a single Gazebo world"""

    world_id: int
    gzserver_port: int
    gazebo_master_uri: str
    ros_namespace: str
    foxglove_namespace: str
    log_file: str | None = None
    process: subprocess.Popen | None = None
    is_running: bool = False


@dataclass
class WorldResult:
    """Result from executing a scenario in a world"""

    scenario_name: str
    world_id: int
    success: bool = False
    completed: bool = False
    duration_sec: float = 0.0
    trajectory: list[tuple[float, float, float]] = field(default_factory=list)
    collision_count: int = 0
    safety_violations: list[str] = field(default_factory=list)
    max_deviation_m: float = 0.0
    error_message: str | None = None


@dataclass
class BatchConfig:
    """Configuration for batch execution."""

    num_worlds: int = 4
    headless: bool = True
    scenario_dir: Path = field(default_factory=lambda: Path("scenarios"))
    world_template: str | None = None
    enable_foxglove: bool = True
    foxglove_port: int = 8765


@dataclass
class ExecutionMetrics:
    """Metrics for a single scenario execution."""

    duration_sec: float = 0.0
    collision_count: int = 0
    safety_violations: list[str] = field(default_factory=list)
    max_deviation_m: float = 0.0
    trajectory_length: int = 0
    goal_reached: bool = False
    error_message: str | None = None

    def to_dict(self) -> dict[str, Any]:
        """Convert metrics to dictionary."""
        return {
            "duration_sec": self.duration_sec,
            "collision_count": self.collision_count,
            "safety_violations": self.safety_violations,
            "max_deviation_m": self.max_deviation_m,
            "trajectory_length": self.trajectory_length,
            "goal_reached": self.goal_reached,
            "error_message": self.error_message,
        }
