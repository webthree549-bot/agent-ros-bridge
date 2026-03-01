"""AutoGPT Integration - Adapter for gateway_v2."""

import json
import logging
from typing import Dict, List

logger = logging.getLogger(__name__)


class AutoGPTAdapter:
    """Adapter for AutoGPT integration.

    Exposes bridge actions as AutoGPT commands.

    Example:
        from agent_ros_bridge.gateway_v2.core import Bridge
        from agent_ros_bridge.integrations.autogpt_adapter import AutoGPTAdapter

        bridge = Bridge()
        adapter = AutoGPTAdapter(bridge)

        # Get AutoGPT-compatible commands
        commands = adapter.get_commands()
    """

    def __init__(self, bridge):
        self.bridge = bridge
        self.commands: Dict[str, Dict] = {}
        logger.info("AutoGPTAdapter initialized")

    def discover_commands(self) -> Dict[str, Dict]:
        """Discover all available commands from bridge."""
        commands = {}

        # Get actions from bridge
        if self.bridge and hasattr(self.bridge, "get_actions"):
            actions = self.bridge.get_actions()
            for action in actions:
                commands[f"ros_{action}"] = {
                    "name": f"ros_{action}",
                    "description": f"Execute ROS action: {action}",
                    "params": ["params"],
                }

        self.commands = commands
        return commands

    def get_commands(self) -> List[Dict]:
        """Get commands in AutoGPT format."""
        if not self.commands:
            self.discover_commands()

        return [
            {"name": name, "description": cmd["description"], "arguments": cmd.get("params", [])}
            for name, cmd in self.commands.items()
        ]

    async def execute_command(self, command_name: str, **kwargs) -> str:
        """Execute a command.

        Args:
            command_name: Name of the command (e.g., "ros_navigate")
            **kwargs: Command parameters

        Returns:
            JSON string with result
        """
        try:
            # Extract action name from command
            action = command_name.replace("ros_", "")

            if not self.bridge:
                return json.dumps({"error": "Bridge not available"})

            # Execute via bridge
            if hasattr(self.bridge, "execute_action"):
                result = await self.bridge.execute_action(action, kwargs)
                return json.dumps(result)
            else:
                return json.dumps({"error": "Bridge doesn't support execute_action"})

        except Exception as e:
            logger.error(f"Command execution error: {e}")
            return json.dumps({"error": str(e)})

    def to_autogpt_plugin_format(self) -> Dict:
        """Export as AutoGPT plugin manifest."""
        return {
            "name": "agent_ros_bridge",
            "version": "0.5.0",
            "description": "Control ROS robots via Agent ROS Bridge",
            "commands": self.get_commands(),
        }
