"""Tool registry for managing ROS tools."""

from typing import Any, Optional
from .base import ROSTool


class ToolRegistry:
    """Registry for managing available tools."""

    def __init__(self):
        self._tools: dict[str, ROSTool] = {}

    def register(self, tool: ROSTool) -> None:
        """Register a tool."""
        self._tools[tool.name] = tool

    def unregister(self, name: str) -> bool:
        """Unregister a tool by name."""
        if name in self._tools:
            del self._tools[name]
            return True
        return False

    def get_tool(self, name: str) -> Optional[ROSTool]:
        """Get a tool by name."""
        return self._tools.get(name)

    def get_all_tools(self) -> list[ROSTool]:
        """Get all registered tools."""
        return list(self._tools.values())

    def list_tools(self) -> list[str]:
        """List all tool names."""
        return list(self._tools.keys())

    def get_tools_by_category(self, category: str) -> list[ROSTool]:
        """Get tools by category."""
        return [
            tool for tool in self._tools.values()
            if getattr(tool, 'category', None) == category
        ]

    def clear(self) -> None:
        """Clear all tools."""
        self._tools.clear()
