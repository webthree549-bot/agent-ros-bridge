"""Plugin system for extending gateway functionality.

This package provides the plugin architecture for adding custom functionality
to the agent-ros-bridge gateway. Plugins can:
    - Intercept and modify messages
    - Add custom commands
    - Integrate with external systems
    - Extend robot capabilities

Example:
    from agent_ros_bridge.gateway_v2.core import Plugin

    class MyPlugin(Plugin):
        name = "my_plugin"
        version = "1.0.0"

        async def initialize(self, gateway):
            # Set up plugin
            pass
"""

from agent_ros_bridge.gateway_v2.core import Plugin

__all__ = ["Plugin"]
