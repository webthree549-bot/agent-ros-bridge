"""Base tool classes for Agent ROS Bridge tool ecosystem."""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any


@dataclass
class ToolResult:
    """Result of tool execution."""

    success: bool
    output: str
    error: str | None = None
    data: dict[str, Any] = field(default_factory=dict)
    execution_time_ms: float = 0.0


class ROSTool(ABC):
    """Base class for ROS tools.

    Compatible with NASA ROSA tool ecosystem.
    """

    name: str
    description: str
    version: str = "1.0.0"

    @abstractmethod
    def execute(self, **kwargs) -> ToolResult:
        """Execute the tool with given parameters."""
        pass

    def validate_params(self, params: dict[str, Any]) -> tuple[bool, str]:
        """Validate input parameters. Override for custom validation."""
        return True, ""

    def get_schema(self) -> dict[str, Any]:
        """Get JSON schema for tool parameters."""
        return {
            "name": self.name,
            "description": self.description,
            "version": self.version,
        }


# Alias for backward compatibility
BaseTool = ROSTool
