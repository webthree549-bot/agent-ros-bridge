"""AutoGPT plugin integration for Agent ROS Bridge.

Allows AutoGPT to control ROS robots through the bridge.

Example:
    # In AutoGPT's .env:
    # ALLOWLISTED_PLUGINS=agent_ros_bridge
    
    # The plugin is automatically discovered and loaded
"""

import json
import logging
from typing import Any, Dict, List, Optional
from dataclasses import dataclass, asdict

try:
    from autogpt.plugins import AutoGPTPluginTemplate
    AUTOGPT_AVAILABLE = True
except ImportError:
    AUTOGPT_AVAILABLE = False
    AutoGPTPluginTemplate = object

from agent_ros_bridge import ROSBridge
from agent_ros_bridge.discovery import ToolDiscovery, SafetyLevel

logger = logging.getLogger(__name__)


@dataclass
class AutoGPTCommand:
    """AutoGPT command specification."""
    name: str
    description: str
    parameters: Dict[str, Any]
    
    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)


class AutoGPTPlugin(AutoGPTPluginTemplate if AUTOGPT_AVAILABLE else object):
    """AutoGPT plugin for ROS robot control.
    
    This plugin exposes ROS robot capabilities as AutoGPT commands.
    
    Installation:
        1. Install package: pip install agent-ros-bridge
        2. Add to AutoGPT's .env: ALLOWLISTED_PLUGINS=agent_ros_bridge
        3. Set ROS_BRIDGE_URL in .env
    """
    
    def __init__(self):
        super().__init__()
        self._name = "agent-ros-bridge"
        self._version = "0.4.0"
        self._description = "Control ROS robots through Agent ROS Bridge"
        self.bridge: Optional[ROSBridge] = None
        self.discovery: Optional[ToolDiscovery] = None
        self._commands: List[AutoGPTCommand] = []
    
    def can_handle_post_prompt(self) -> bool:
        """Return True to add commands to prompt."""
        return True
    
    def post_prompt(self, prompt: Any) -> Any:
        """Add ROS commands to AutoGPT prompt.
        
        Args:
            prompt: AutoGPT prompt
            
        Returns:
            Modified prompt with ROS commands
        """
        if not AUTOGPT_AVAILABLE:
            logger.warning("AutoGPT not available")
            return prompt
        
        # Initialize bridge if needed
        if self.bridge is None:
            self._init_bridge()
        
        # Discover commands
        self._discover_commands()
        
        # Add commands to prompt
        for cmd in self._commands:
            # Create closure to capture cmd_name
            def make_handler(name):
                return lambda **kwargs: self._execute_command(name, **kwargs)
            
            prompt.add_command(
                cmd.name,
                cmd.description,
                cmd.parameters,
                make_handler(cmd.name)
            )
        
        return prompt
    
    def _init_bridge(self):
        """Initialize ROSBridge connection."""
        import os
        
        # Get configuration from environment
        ros_version = int(os.getenv("ROS_VERSION", "2"))
        
        self.bridge = ROSBridge(ros_version=ros_version)
        self.discovery = ToolDiscovery(self.bridge)
        
        logger.info(f"AutoGPT plugin initialized with ROS{ros_version}")
    
    def _discover_commands(self):
        """Discover available ROS commands."""
        if not self.bridge:
            return
        
        # Get registered actions
        actions = self.bridge.get_registered_actions()
        
        # Create commands
        self._commands = []
        for action in actions:
            cmd = AutoGPTCommand(
                name=f"ros_{action}",
                description=f"Execute ROS action: {action}",
                parameters={
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "const": action,
                            "description": f"Action name: {action}"
                        }
                    },
                    "required": ["action"]
                }
            )
            self._commands.append(cmd)
        
        logger.info(f"Discovered {len(self._commands)} ROS commands")
    
    def _execute_command(self, command_name: str, **kwargs) -> str:
        """Execute a ROS command.
        
        Args:
            command_name: Name of command to execute
            **kwargs: Command parameters
            
        Returns:
            Command result as string
        """
        import asyncio
        
        if not self.bridge:
            return "Error: Bridge not initialized"
        
        # Extract action name from command
        action = command_name.replace("ros_", "")
        
        # Execute async
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        
        result = loop.run_until_complete(
            self.bridge.call_action(action, **kwargs)
        )
        
        if result.get("success"):
            return json.dumps(result.get("result"))
        else:
            return f"Error: {result.get('error', 'Unknown error')}"
    
    def can_handle_on_response(self) -> bool:
        """Return True to handle responses."""
        return False
    
    def on_response(self, response: str, *args, **kwargs) -> str:
        """Handle response (optional)."""
        return response
    
    def can_handle_post_planning(self) -> bool:
        """Return True to handle planning."""
        return False
    
    def post_planning(self, response: str) -> str:
        """Handle post-planning (optional)."""
        return response


# Alternative: Direct integration without plugin system
class AutoGPTBridge:
    """Direct AutoGPT integration without plugin system.
    
    Use this for custom AutoGPT setups or testing.
    
    Example:
        bridge = AutoGPTBridge(ros_bridge)
        result = bridge.execute("navigate", x=5, y=3)
    """
    
    def __init__(self, bridge: ROSBridge):
        """Initialize bridge.
        
        Args:
            bridge: ROSBridge instance
        """
        self.bridge = bridge
        self.discovery = ToolDiscovery(bridge)
    
    def get_commands(self) -> List[Dict[str, Any]]:
        """Get list of available commands.
        
        Returns:
            List of command specifications
        """
        actions = self.bridge.get_registered_actions()
        
        commands = []
        for action in actions:
            commands.append({
                "name": f"ros_{action}",
                "description": f"Execute ROS action: {action}",
                "args": {
                    "action": action
                }
            })
        
        return commands
    
    async def execute(self, action: str, **kwargs) -> str:
        """Execute an action.
        
        Args:
            action: Action name
            **kwargs: Action parameters
            
        Returns:
            Result as string
        """
        result = await self.bridge.call_action(action, **kwargs)
        
        if result.get("success"):
            return f"Success: {json.dumps(result.get('result'))}"
        else:
            return f"Error: {result.get('error', 'Unknown error')}"
    
    def format_for_autogpt(self) -> str:
        """Format commands for AutoGPT prompt.
        
        Returns:
            Formatted command descriptions
        """
        commands = self.get_commands()
        
        lines = ["Available ROS Commands:"]
        for cmd in commands:
            lines.append(f"  {cmd['name']}: {cmd['description']}")
        
        return "\n".join(lines)


# Setup function for AutoGPT
def setup_autogpt_plugin() -> AutoGPTPlugin:
    """Setup function called by AutoGPT.
    
    Returns:
        Plugin instance
    """
    return AutoGPTPlugin()


# If running directly, print setup instructions
if __name__ == "__main__":
    print("""
Agent ROS Bridge - AutoGPT Plugin
=================================

To use with AutoGPT:

1. Install the package:
   pip install agent-ros-bridge

2. Add to AutoGPT's .env file:
   ALLOWLISTED_PLUGINS=agent_ros_bridge
   ROS_VERSION=2

3. Start AutoGPT - the plugin will be automatically loaded

The following commands will be available:
  - ros_navigate: Navigate to coordinates
  - ros_move_arm: Move robotic arm
  - ros_grasp: Control gripper
  - ros_get_status: Get robot status
  - And any custom actions you register

For more information, see: https://docs.openclaw.ai/agent-ros-bridge/autogpt
""")


__all__ = [
    "AutoGPTPlugin",
    "AutoGPTBridge",
    "AutoGPTCommand",
    "setup_autogpt_plugin"
]
