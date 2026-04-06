"""Base tool class for Agent ROS Bridge Tools."""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, TypeVar

T = TypeVar("T")

# Tool registry for plugin discovery
_TOOL_REGISTRY: dict[str, type["Tool"]] = {}


@dataclass
class ToolResult:
    """Result of tool execution."""

    success: bool = True
    data: Any = None
    error_message: str | None = None
    execution_time_ms: float = 0.0
    metadata: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def success_result(cls, data: Any, **kwargs) -> "ToolResult":
        """Create a successful result."""
        return cls(success=True, data=data, **kwargs)

    @classmethod
    def error_result(cls, message: str, **kwargs) -> "ToolResult":
        """Create an error result."""
        return cls(success=False, error_message=message, **kwargs)


class Tool(ABC):
    """Base class for all ROS tools.

    Compatible with NASA ROSA tool format.
    """

    name: str = ""
    description: str = ""
    version: str = "1.0.0"

    def __init__(self, config: dict[str, Any] | None = None):
        """Initialize tool with optional configuration.

        Args:
            config: Tool-specific configuration
        """
        self.config = config or {}

    @abstractmethod
    async def execute(self, *args, **kwargs) -> ToolResult:
        """Execute the tool.

        Returns:
            ToolResult with execution outcome
        """
        raise NotImplementedError

    def get_schema(self) -> dict[str, Any]:
        """Get JSON schema for tool parameters.

        Returns:
            JSON schema dict
        """
        return {
            "name": self.name,
            "description": self.description,
            "version": self.version,
        }


def register_tool(cls: type[T]) -> type[T]:
    """Decorator to register a tool in the global registry.

    Example:
        @register_tool
        class MyCustomTool(Tool):
            name = "my_tool"
    """
    _TOOL_REGISTRY[cls.name] = cls
    return cls


def get_tool(name: str) -> type[Tool] | None:
    """Get a tool class by name.

    Args:
        name: Tool name

    Returns:
        Tool class or None if not found
    """
    return _TOOL_REGISTRY.get(name)


def list_tools() -> list[str]:
    """List all registered tool names.

    Returns:
        List of tool names
    """
    return list(_TOOL_REGISTRY.keys())
