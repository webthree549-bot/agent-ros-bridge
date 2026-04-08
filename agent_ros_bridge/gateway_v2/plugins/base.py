"""Base plugin system for Agent ROS Bridge."""

from abc import ABC, abstractmethod
from typing import Any


class Plugin(ABC):
    """Base class for plugins."""
    
    name: str = "base_plugin"
    version: str = "1.0.0"
    
    def __init__(self):
        self.enabled = True
        self.initialized = False
    
    @abstractmethod
    async def initialize(self) -> bool:
        """Initialize the plugin."""
        pass
    
    @abstractmethod
    async def shutdown(self) -> bool:
        """Shutdown the plugin."""
        pass


class PluginRegistry:
    """Registry for managing plugins."""
    
    def __init__(self):
        self._plugins: dict[str, Plugin] = {}
    
    def register(self, plugin: Plugin) -> None:
        """Register a plugin."""
        self._plugins[plugin.name] = plugin
    
    def get_plugin(self, name: str) -> Plugin | None:
        """Get a plugin by name."""
        return self._plugins.get(name)
    
    def list_plugins(self) -> list[str]:
        """List all registered plugin names."""
        return list(self._plugins.keys())
    
    def unregister(self, name: str) -> bool:
        """Unregister a plugin."""
        if name in self._plugins:
            del self._plugins[name]
            return True
        return False
