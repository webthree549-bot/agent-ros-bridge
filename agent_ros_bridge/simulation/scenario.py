"""Simulation scenarios for testing."""

from dataclasses import dataclass
from typing import Any


@dataclass
class SimulationScenario:
    """A simulation scenario with parameters."""

    name: str
    duration_sec: float = 60.0
    obstacles: list[dict[str, Any]] = None
    goals: list[dict[str, Any]] = None

    def __post_init__(self):
        if self.obstacles is None:
            self.obstacles = []
        if self.goals is None:
            self.goals = []
