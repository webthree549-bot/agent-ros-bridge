"""OpenClaw Integration - Adapter for OpenClaw AI Agent Framework.

This module provides integration between Agent ROS Bridge and the OpenClaw
framework, allowing OpenClaw agents to control ROS robots through the bridge.

The OpenClaw integration works via ClawHub skills - Agent ROS Bridge provides
a skill that OpenClaw agents use to understand how to control robots.

Example:
    from agent_ros_bridge import Bridge
    from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter

    bridge = Bridge()
    adapter = bridge.get_openclaw_adapter()

    # Get skill path for ClawHub
    skill_path = adapter.get_skill_path()

    # Or use direct integration (if extension mode)
    tools = adapter.get_tools()
    result = await adapter.execute_tool("ros2_publish", {...})
"""

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Union

logger = logging.getLogger(__name__)


@dataclass
class OpenClawTool:
    """OpenClaw tool definition.
    
    Mirrors the structure expected by OpenClaw's tool system.
    """
    name: str
    description: str
    parameters: Dict[str, Any] = field(default_factory=dict)
    handler: Optional[Callable[..., Any]] = None
    dangerous: bool = False
    
    def to_openclaw_format(self) -> Dict[str, Any]:
        """Convert to OpenClaw tool format."""
        return {
            "name": self.name,
            "description": self.description,
            "parameters": {
                "type": "object",
                "properties": self.parameters,
                "required": list(self.parameters.keys()) if self.parameters else [],
            },
            "dangerous": self.dangerous,
        }


class OpenClawAdapter:
    """Adapter for OpenClaw framework integration.
    
    Provides two modes of integration:
    
    1. **Skill Mode** (default): Provides a ClawHub skill that teaches OpenClaw
       agents how to use Agent ROS Bridge via WebSocket/MQTT/gRPC APIs.
       
    2. **Extension Mode** (optional): Direct integration as an OpenClaw extension
       with native tool registration (requires OpenClaw extension support).
    
    Example:
        bridge = Bridge()
        adapter = bridge.get_openclaw_adapter()
        
        # Get skill for ClawHub distribution
        skill_path = adapter.get_skill_path()
        
        # Or use extension mode
        tools = adapter.get_tools()
    """

    # Standard ROS2 tools that match RosClaw's interface
    DEFAULT_TOOLS = [
        {
            "name": "ros2_publish",
            "description": "Publish a message to any ROS2 topic",
            "parameters": {
                "topic": {"type": "string", "description": "ROS2 topic name (e.g., /cmd_vel)"},
                "message_type": {"type": "string", "description": "ROS2 message type (e.g., geometry_msgs/Twist)"},
                "message": {"type": "object", "description": "Message data as JSON object"},
            },
            "dangerous": False,
        },
        {
            "name": "ros2_subscribe_once",
            "description": "Read the latest message from a ROS2 topic",
            "parameters": {
                "topic": {"type": "string", "description": "ROS2 topic name to read from"},
                "timeout": {"type": "number", "description": "Timeout in seconds", "default": 5.0},
            },
            "dangerous": False,
        },
        {
            "name": "ros2_service_call",
            "description": "Call a ROS2 service",
            "parameters": {
                "service": {"type": "string", "description": "Service name (e.g., /spawn)"},
                "service_type": {"type": "string", "description": "Service type (e.g., turtlesim/srv/Spawn)"},
                "request": {"type": "object", "description": "Service request data"},
            },
            "dangerous": False,
        },
        {
            "name": "ros2_action_goal",
            "description": "Send an action goal with feedback streaming",
            "parameters": {
                "action_name": {"type": "string", "description": "Action server name (e.g., /navigate_to_pose)"},
                "action_type": {"type": "string", "description": "Action type"},
                "goal": {"type": "object", "description": "Goal data"},
            },
            "dangerous": True,
        },
        {
            "name": "ros2_param_get",
            "description": "Get a ROS2 node parameter value",
            "parameters": {
                "node_name": {"type": "string", "description": "Fully qualified node name"},
                "param_name": {"type": "string", "description": "Parameter name"},
            },
            "dangerous": False,
        },
        {
            "name": "ros2_param_set",
            "description": "Set a ROS2 node parameter value",
            "parameters": {
                "node_name": {"type": "string", "description": "Fully qualified node name"},
                "param_name": {"type": "string", "description": "Parameter name"},
                "value": {"type": "any", "description": "Parameter value"},
            },
            "dangerous": True,
        },
        {
            "name": "ros2_list_topics",
            "description": "List all available ROS2 topics",
            "parameters": {
                "filter": {"type": "string", "description": "Optional topic name filter", "default": ""},
            },
            "dangerous": False,
        },
        {
            "name": "ros2_list_services",
            "description": "List all available ROS2 services",
            "parameters": {},
            "dangerous": False,
        },
        {
            "name": "ros2_camera_snapshot",
            "description": "Capture a frame from a camera topic",
            "parameters": {
                "camera_topic": {"type": "string", "description": "Camera topic (e.g., /camera/image_raw)"},
                "encoding": {"type": "string", "description": "Image encoding format", "default": "jpeg"},
            },
            "dangerous": False,
        },
    ]

    # Agent ROS Bridge specific extensions
    EXTENDED_TOOLS = [
        {
            "name": "bridge_list_robots",
            "description": "List all robots registered in the bridge",
            "parameters": {},
            "dangerous": False,
        },
        {
            "name": "bridge_get_robot_status",
            "description": "Get detailed status of a specific robot",
            "parameters": {
                "robot_id": {"type": "string", "description": "Robot identifier"},
            },
            "dangerous": False,
        },
        {
            "name": "fleet_submit_task",
            "description": "Submit a task to the fleet orchestrator",
            "parameters": {
                "task_type": {"type": "string", "description": "Task type (navigate, pickup, etc.)"},
                "target_location": {"type": "string", "description": "Target location or coordinates"},
                "priority": {"type": "integer", "description": "Task priority (1-10, lower is higher)", "default": 5},
            },
            "dangerous": False,
        },
        {
            "name": "fleet_get_metrics",
            "description": "Get fleet-wide metrics and status",
            "parameters": {},
            "dangerous": False,
        },
        {
            "name": "safety_trigger_estop",
            "description": "Trigger emergency stop on a robot",
            "parameters": {
                "robot_id": {"type": "string", "description": "Robot to emergency stop"},
                "reason": {"type": "string", "description": "Reason for emergency stop"},
            },
            "dangerous": True,
        },
        {
            "name": "safety_release_estop",
            "description": "Release emergency stop on a robot",
            "parameters": {
                "robot_id": {"type": "string", "description": "Robot to release"},
            },
            "dangerous": True,
        },
        {
            "name": "memory_set",
            "description": "Store data in agent memory",
            "parameters": {
                "key": {"type": "string", "description": "Memory key"},
                "value": {"type": "any", "description": "Value to store"},
                "ttl": {"type": "integer", "description": "Time-to-live in seconds", "default": 3600},
            },
            "dangerous": False,
        },
        {
            "name": "memory_get",
            "description": "Retrieve data from agent memory",
            "parameters": {
                "key": {"type": "string", "description": "Memory key"},
            },
            "dangerous": False,
        },
    ]

    def __init__(self, bridge, include_ros1: bool = False):
        """Initialize OpenClaw adapter with bridge reference.
        
        Args:
            bridge: Gateway bridge instance for robot control.
            include_ros1: Whether to include ROS1-specific tools.
        """
        self.bridge = bridge
        self.include_ros1 = include_ros1
        self._tools: Dict[str, OpenClawTool] = {}
        self._register_default_tools()
        
        # Find skill path
        self._skill_path = self._find_skill_path()
        
        logger.info("OpenClawAdapter initialized")

    def _find_skill_path(self) -> Optional[Path]:
        """Find the ClawHub skill directory."""
        # Look in common locations
        possible_paths = [
            Path(__file__).parent.parent.parent / "skills" / "agent-ros-bridge",
            Path.cwd() / "skills" / "agent-ros-bridge",
            Path.home() / ".openclaw" / "skills" / "agent-ros-bridge",
        ]
        
        for path in possible_paths:
            if path.exists() and (path / "SKILL.md").exists():
                return path
        
        logger.warning("Could not find OpenClaw skill directory")
        return None

    def _register_default_tools(self):
        """Register default tool definitions."""
        for tool_def in self.DEFAULT_TOOLS + self.EXTENDED_TOOLS:
            tool = OpenClawTool(
                name=tool_def["name"],
                description=tool_def["description"],
                parameters=tool_def["parameters"],
                dangerous=tool_def.get("dangerous", False),
            )
            self._tools[tool.name] = tool

        if self.include_ros1:
            self._register_ros1_tools()

    def _register_ros1_tools(self):
        """Register ROS1-specific tools."""
        ros1_tools = [
            {
                "name": "ros1_publish",
                "description": "Publish a message to a ROS1 topic",
                "parameters": {
                    "topic": {"type": "string", "description": "ROS1 topic name"},
                    "message_type": {"type": "string", "description": "ROS1 message type"},
                    "message": {"type": "object", "description": "Message data"},
                },
                "dangerous": False,
            },
            {
                "name": "ros1_subscribe_once",
                "description": "Read the latest message from a ROS1 topic",
                "parameters": {
                    "topic": {"type": "string", "description": "ROS1 topic name"},
                },
                "dangerous": False,
            },
            {
                "name": "ros1_service_call",
                "description": "Call a ROS1 service",
                "parameters": {
                    "service": {"type": "string", "description": "Service name"},
                    "request": {"type": "object", "description": "Service request"},
                },
                "dangerous": False,
            },
        ]
        for tool_def in ros1_tools:
            tool = OpenClawTool(
                name=tool_def["name"],
                description=tool_def["description"],
                parameters=tool_def["parameters"],
                dangerous=tool_def.get("dangerous", False),
            )
            self._tools[tool.name] = tool

    def get_skill_path(self) -> Optional[Path]:
        """Get the path to the ClawHub skill directory.
        
        Returns:
            Path to skill directory or None if not found.
        """
        return self._skill_path

    def get_tools(self) -> List[Dict[str, Any]]:
        """Get all tools in OpenClaw format.
        
        Returns:
            List of tool definitions compatible with OpenClaw.
        """
        return [tool.to_openclaw_format() for tool in self._tools.values()]

    def get_tool(self, name: str) -> Optional[OpenClawTool]:
        """Get a specific tool by name.
        
        Args:
            name: Tool name.
            
        Returns:
            Tool definition or None if not found.
        """
        return self._tools.get(name)

    async def execute_tool(self, tool_name: str, arguments: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a tool with the given arguments.
        
        This is the main entry point for OpenClaw to invoke robot actions.
        
        Args:
            tool_name: Name of the tool to execute.
            arguments: Tool arguments.
            
        Returns:
            Execution result with success status and data/error.
        """
        try:
            if not self.bridge:
                return {"success": False, "error": "Bridge not available"}

            tool = self._tools.get(tool_name)
            if not tool:
                return {"success": False, "error": f"Unknown tool: {tool_name}"}

            # Route to appropriate handler
            handler = getattr(self, f"_handle_{tool_name}", None)
            if handler:
                return await handler(arguments)
            
            # Fallback to bridge execution
            if hasattr(self.bridge, "execute_action"):
                result = await self.bridge.execute_action(tool_name, arguments)
                return {"success": True, "data": result}
            
            return {"success": False, "error": f"No handler for tool: {tool_name}"}

        except Exception as e:
            logger.error(f"Tool execution error: {e}")
            return {"success": False, "error": str(e)}

    # ===================================================================
    # Tool Handlers
    # ===================================================================

    async def _handle_ros2_publish(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle ros2_publish tool."""
        topic = args.get("topic")
        message = args.get("message")
        
        if not topic or message is None:
            return {"success": False, "error": "Missing required parameters: topic, message"}
        
        # Find ROS2 connector
        ros2_connector = self._get_ros2_connector()
        if not ros2_connector:
            return {"success": False, "error": "ROS2 connector not available"}
        
        try:
            result = await ros2_connector.publish(topic, message)
            return {"success": True, "data": {"published": True, "topic": topic}}
        except Exception as e:
            return {"success": False, "error": f"Publish failed: {e}"}

    async def _handle_ros2_subscribe_once(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle ros2_subscribe_once tool."""
        topic = args.get("topic")
        timeout = args.get("timeout", 5.0)
        
        if not topic:
            return {"success": False, "error": "Missing required parameter: topic"}
        
        ros2_connector = self._get_ros2_connector()
        if not ros2_connector:
            return {"success": False, "error": "ROS2 connector not available"}
        
        try:
            result = await ros2_connector.subscribe_once(topic, timeout=timeout)
            return {"success": True, "data": result}
        except Exception as e:
            return {"success": False, "error": f"Subscribe failed: {e}"}

    async def _handle_ros2_list_topics(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle ros2_list_topics tool."""
        filter_pattern = args.get("filter", "")
        
        ros2_connector = self._get_ros2_connector()
        if not ros2_connector:
            return {"success": False, "error": "ROS2 connector not available"}
        
        try:
            topics = await ros2_connector.list_topics()
            if filter_pattern:
                topics = [t for t in topics if filter_pattern in t.get("name", "")]
            return {"success": True, "data": {"topics": topics}}
        except Exception as e:
            return {"success": False, "error": f"List topics failed: {e}"}

    async def _handle_bridge_list_robots(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle bridge_list_robots tool."""
        if not hasattr(self.bridge, "fleets"):
            return {"success": False, "error": "Bridge doesn't support fleet management"}
        
        robots = []
        for fleet in self.bridge.fleets.values():
            for robot in fleet.robots.values():
                robots.append({
                    "id": robot.robot_id,
                    "name": robot.name,
                    "status": robot.status.value if hasattr(robot.status, "value") else str(robot.status),
                    "connector_type": getattr(robot, "connector_type", "unknown"),
                })
        
        return {"success": True, "data": {"robots": robots}}

    async def _handle_fleet_get_metrics(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle fleet_get_metrics tool."""
        if not hasattr(self.bridge, "orchestrator") or not self.bridge.orchestrator:
            return {"success": False, "error": "Fleet orchestrator not available"}
        
        try:
            metrics = self.bridge.orchestrator.get_metrics()
            return {"success": True, "data": metrics}
        except Exception as e:
            return {"success": False, "error": f"Failed to get metrics: {e}"}

    async def _handle_safety_trigger_estop(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle safety_trigger_estop tool."""
        robot_id = args.get("robot_id")
        reason = args.get("reason", "Emergency stop triggered via OpenClaw")
        
        if not robot_id:
            return {"success": False, "error": "Missing required parameter: robot_id"}
        
        if not hasattr(self.bridge, "safety_manager"):
            return {"success": False, "error": "Safety manager not available"}
        
        try:
            self.bridge.safety_manager.trigger_emergency_stop(reason, robot_id=robot_id)
            return {"success": True, "data": {"estop_triggered": True, "robot_id": robot_id}}
        except Exception as e:
            return {"success": False, "error": f"E-stop failed: {e}"}

    async def _handle_memory_set(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle memory_set tool."""
        key = args.get("key")
        value = args.get("value")
        ttl = args.get("ttl", 3600)
        
        if key is None or value is None:
            return {"success": False, "error": "Missing required parameters: key, value"}
        
        if not hasattr(self.bridge, "memory") or not self.bridge.memory:
            return {"success": False, "error": "Memory not available"}
        
        try:
            await self.bridge.memory.set(key, value, ttl=ttl)
            return {"success": True, "data": {"stored": True, "key": key}}
        except Exception as e:
            return {"success": False, "error": f"Memory set failed: {e}"}

    async def _handle_memory_get(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle memory_get tool."""
        key = args.get("key")
        
        if key is None:
            return {"success": False, "error": "Missing required parameter: key"}
        
        if not hasattr(self.bridge, "memory") or not self.bridge.memory:
            return {"success": False, "error": "Memory not available"}
        
        try:
            value = await self.bridge.memory.get(key)
            return {"success": True, "data": {"key": key, "value": value}}
        except Exception as e:
            return {"success": False, "error": f"Memory get failed: {e}"}

    # ===================================================================
    # Helper Methods
    # ===================================================================

    def _get_ros2_connector(self):
        """Get the ROS2 connector from the bridge."""
        if not self.bridge:
            return None
        
        # Try to find ROS2 connector in transport manager
        if hasattr(self.bridge, "transport_manager"):
            for connector in getattr(self.bridge.transport_manager, "connectors", []):
                if getattr(connector, "connector_type", None) == "ros2":
                    return connector
        
        # Try direct access
        if hasattr(self.bridge, "ros2_connector"):
            return self.bridge.ros2_connector
        
        return None

    def _get_ros1_connector(self):
        """Get the ROS1 connector from the bridge."""
        if not self.bridge:
            return None
        
        if hasattr(self.bridge, "transport_manager"):
            for connector in getattr(self.bridge.transport_manager, "connectors", []):
                if getattr(connector, "connector_type", None) == "ros1":
                    return connector
        
        if hasattr(self.bridge, "ros1_connector"):
            return self.bridge.ros1_connector
        
        return None

    # ===================================================================
    # ClawHub Skill Interface
    # ===================================================================

    def package_skill(self, output_dir: Path) -> Path:
        """Package the skill for ClawHub distribution.
        
        Args:
            output_dir: Directory to output .skill file
            
        Returns:
            Path to packaged .skill file
        """
        if not self._skill_path:
            raise RuntimeError("Skill path not found")
        
        import zipfile
        
        skill_name = self._skill_path.name
        output_file = output_dir / f"{skill_name}.skill"
        
        with zipfile.ZipFile(output_file, 'w', zipfile.ZIP_DEFLATED) as zf:
            for file_path in self._skill_path.rglob('*'):
                if file_path.is_file():
                    arcname = file_path.relative_to(self._skill_path)
                    zf.write(file_path, arcname)
        
        logger.info(f"Packaged skill to {output_file}")
        return output_file

    # ===================================================================
    # RosClaw Compatibility
    # ===================================================================

    def to_rosclaw_compatible_format(self) -> List[Dict[str, Any]]:
        """Export tools in RosClaw-compatible format.
        
        This allows Agent ROS Bridge to be used as a drop-in replacement
        for RosClaw in OpenClaw environments.
        
        Returns:
            List of tool definitions matching RosClaw's interface.
        """
        rosclaw_tools = []
        
        for tool_def in self.DEFAULT_TOOLS:
            rosclaw_tools.append({
                "type": "function",
                "function": {
                    "name": tool_def["name"],
                    "description": tool_def["description"],
                    "parameters": {
                        "type": "object",
                        "properties": tool_def["parameters"],
                        "required": list(tool_def["parameters"].keys()),
                    },
                },
            })
        
        return rosclaw_tools

    # ===================================================================
    # Natural Language Support (NEW)
    # ===================================================================

    def enable_natural_language(self):
        """Enable natural language interpretation.
        
        This adds NL support to fulfill SKILL promises about
        natural language commands.
        """
        from .nl_interpreter import RuleBasedInterpreter
        
        self.nl_interpreter = RuleBasedInterpreter()
        logger.info("Natural language support enabled")

    async def execute_nl(self, nl_command: str, session_id: str = "default") -> Dict[str, Any]:
        """Execute natural language command.
        
        This method fulfills the SKILL promise of natural language control.
        
        Args:
            nl_command: Natural language command (e.g., "Move forward 2 meters")
            session_id: Session ID for context tracking
            
        Returns:
            Execution result with interpretation details
            
        Examples:
            >>> await adapter.execute_nl("Move forward 2 meters")
            {
                "command": "Move forward 2 meters",
                "interpreted_as": {"tool": "ros2_publish", "topic": "/cmd_vel", ...},
                "result": {"success": True, ...}
            }
            
            >>> await adapter.execute_nl("Turn left 90 degrees")
            {
                "command": "Turn left 90 degrees",
                "interpreted_as": {"tool": "ros2_publish", "topic": "/cmd_vel", ...},
                "result": {"success": True, ...}
            }
        """
        if not hasattr(self, 'nl_interpreter'):
            self.enable_natural_language()
        
        # Interpret the natural language command
        interpretation = self.nl_interpreter.interpret(nl_command, context=None)
        
        if "error" in interpretation:
            return {
                "success": False,
                "command": nl_command,
                "error": interpretation["error"],
                "suggestion": interpretation.get("suggestion", "")
            }
        
        # Extract tool and parameters
        tool_name = interpretation.get("tool")
        params = {k: v for k, v in interpretation.items() 
                  if k not in ["tool", "explanation", "note"]}
        
        # Execute the interpreted command
        result = await self.execute_tool(tool_name, params)
        
        return {
            "success": result.get("success", False),
            "command": nl_command,
            "interpretation": {
                "tool": tool_name,
                "parameters": params,
                "explanation": interpretation.get("explanation", ""),
                "note": interpretation.get("note", "")
            },
            "result": result
        }
