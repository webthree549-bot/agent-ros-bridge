"""Tool discovery for automatic ROS capability detection.

Discovers ROS topics, actions, and services, then generates MCP tool definitions.

Example:
    from agent_ros_bridge.discovery import ToolDiscovery
    
    discovery = ToolDiscovery(bridge)
    tools = await discovery.discover_all()
    
    # Auto-generate MCP tools
    mcp_tools = discovery.to_mcp_tools()
"""

import logging
from typing import Any, Dict, List, Optional, Set
from dataclasses import dataclass, field
from enum import Enum

from agent_ros_bridge import ROSBridge

logger = logging.getLogger(__name__)


class SafetyLevel(Enum):
    """Safety classification for robot actions."""
    SAFE = "safe"           # Read-only, no physical movement
    MEDIUM = "medium"       # Movement in constrained space
    DANGEROUS = "dangerous" # High velocity, collision risk, etc.


@dataclass
class DiscoveredTool:
    """A discovered robot capability."""
    name: str
    type: str  # "topic", "action", "service"
    description: str
    ros_name: str  # Original ROS name
    message_type: str
    safety_level: SafetyLevel
    parameters: Dict[str, Any] = field(default_factory=dict)
    requirements: List[str] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def to_mcp_tool(self) -> Dict[str, Any]:
        """Convert to MCP tool definition."""
        return {
            "name": f"ros_{self.name}",
            "description": self.description,
            "inputSchema": {
                "type": "object",
                "properties": self.parameters,
                "required": list(self.parameters.keys()) if self.parameters else []
            }
        }
    
    def to_openai_function(self) -> Dict[str, Any]:
        """Convert to OpenAI function definition."""
        return {
            "name": f"ros_{self.name}",
            "description": self.description,
            "parameters": {
                "type": "object",
                "properties": self.parameters,
                "required": list(self.parameters.keys()) if self.parameters else []
            }
        }


class ToolDiscovery:
    """Discovers ROS capabilities and converts to AI tool definitions.
    
    Automatically detects:
    - Topics (pub/sub)
    - Actions (goal/cancel/result)
    - Services (request/response)
    
    Classifies safety levels and generates parameter schemas.
    """
    
    # Safety classification rules
    SAFETY_RULES = {
        SafetyLevel.DANGEROUS: [
            "cmd_vel", "joint_command", "gripper", "actuator",
            "emergency", "motor", "servo"
        ],
        SafetyLevel.MEDIUM: [
            "navigate", "move", "goto", "position", "trajectory",
            "arm", "manipulator", "pick", "place"
        ],
        SafetyLevel.SAFE: [
            "status", "sensor", "camera", "lidar", "odom",
            "battery", "diagnostic", "log"
        ]
    }
    
    # Tool descriptions
    TOOL_DESCRIPTIONS = {
        "navigate": "Navigate the robot to a specific position",
        "move_arm": "Move the robotic arm to a position",
        "grasp": "Control the gripper to grasp or release objects",
        "patrol": "Start an autonomous patrol route",
        "get_status": "Get the current robot status",
        "get_battery": "Get battery level and status",
        "get_position": "Get current robot position",
    }
    
    def __init__(self, bridge: ROSBridge):
        """Initialize tool discovery.
        
        Args:
            bridge: ROSBridge instance
        """
        self.bridge = bridge
        self._discovered_tools: List[DiscoveredTool] = []
    
    def _classify_safety(self, name: str) -> SafetyLevel:
        """Classify safety level based on name."""
        name_lower = name.lower()
        
        for level, keywords in self.SAFETY_RULES.items():
            for keyword in keywords:
                if keyword in name_lower:
                    return level
        
        return SafetyLevel.SAFE  # Default
    
    def _generate_description(self, name: str, msg_type: str) -> str:
        """Generate human-readable description."""
        # Check known descriptions
        if name in self.TOOL_DESCRIPTIONS:
            return self.TOOL_DESCRIPTIONS[name]
        
        # Generate from name and type
        words = name.replace("_", " ").split()
        action = words[0] if words else "Execute"
        
        if "nav" in name.lower():
            return f"Navigate the robot: {name}"
        elif "arm" in name.lower() or "manip" in name.lower():
            return f"Control robotic arm: {name}"
        elif "grip" in name.lower() or "grasp" in name.lower():
            return f"Control gripper: {name}"
        elif "sensor" in name.lower() or "camera" in name.lower():
            return f"Read sensor data: {name}"
        else:
            return f"Execute ROS action: {name}"
    
    def _infer_parameters(self, name: str, msg_type: str) -> Dict[str, Any]:
        """Infer parameter schema from tool name and message type."""
        params = {}
        
        # Navigation parameters
        if any(x in name.lower() for x in ["nav", "goto", "position", "pose"]):
            params = {
                "x": {"type": "number", "description": "X coordinate in meters"},
                "y": {"type": "number", "description": "Y coordinate in meters"},
                "theta": {"type": "number", "description": "Orientation in radians", "default": 0.0}
            }
        
        # Arm/joint parameters
        elif any(x in name.lower() for x in ["arm", "joint", "manipulator"]):
            params = {
                "position": {"type": "string", "description": "Target position name or coordinates"},
                "speed": {"type": "number", "description": "Movement speed (0-1)", "default": 0.5}
            }
        
        # Gripper parameters
        elif any(x in name.lower() for x in ["grip", "grasp"]):
            params = {
                "state": {
                    "type": "string",
                    "enum": ["open", "close", "half"],
                    "description": "Gripper state"
                }
            }
        
        # Patrol parameters
        elif "patrol" in name.lower():
            params = {
                "area": {"type": "string", "description": "Area to patrol"},
                "rounds": {"type": "integer", "description": "Number of rounds", "default": 1}
            }
        
        # Generic action parameters
        else:
            params = {
                "parameters": {
                    "type": "object",
                    "description": f"Parameters for {name}"
                }
            }
        
        return params
    
    async def discover_from_topics(self) -> List[DiscoveredTool]:
        """Discover tools from ROS topics.
        
        Returns:
            List of discovered tools
        """
        tools = []
        
        try:
            topics = self.bridge.get_available_topics()
            
            for topic in topics:
                # Skip internal topics
                if topic.startswith('/_'):
                    continue
                
                # Get topic type from connector
                topic_type = "unknown"
                for connector in self.bridge.connector_manager.get_connectors():
                    if hasattr(connector, 'get_topic_types'):
                        types = connector.get_topic_types()
                        if topic in types:
                            topic_type = types[topic]
                            break
                
                safety = self._classify_safety(topic)
                
                tool = DiscoveredTool(
                    name=topic.replace('/', '_').strip('_'),
                    type="topic",
                    description=self._generate_description(topic, topic_type),
                    ros_name=topic,
                    message_type=topic_type,
                    safety_level=safety,
                    parameters={} if safety == SafetyLevel.SAFE else self._infer_parameters(topic, topic_type),
                    requirements=[topic]
                )
                
                tools.append(tool)
                logger.debug(f"Discovered topic: {topic}")
        
        except Exception as e:
            logger.error(f"Error discovering topics: {e}")
        
        return tools
    
    async def discover_from_actions(self) -> List[DiscoveredTool]:
        """Discover tools from registered actions.
        
        Returns:
            List of discovered tools
        """
        tools = []
        
        try:
            actions = self.bridge.get_registered_actions()
            
            for action in actions:
                safety = self._classify_safety(action)
                
                tool = DiscoveredTool(
                    name=action,
                    type="action",
                    description=self._generate_description(action, ""),
                    ros_name=action,
                    message_type="action",
                    safety_level=safety,
                    parameters=self._infer_parameters(action, ""),
                    requirements=[action]
                )
                
                tools.append(tool)
                logger.debug(f"Discovered action: {action}")
        
        except Exception as e:
            logger.error(f"Error discovering actions: {e}")
        
        return tools
    
    async def discover_all(self) -> List[DiscoveredTool]:
        """Discover all available tools.
        
        Returns:
            List of all discovered tools
        """
        self._discovered_tools = []
        
        # Discover from topics
        topic_tools = await self.discover_from_topics()
        self._discovered_tools.extend(topic_tools)
        
        # Discover from actions
        action_tools = await self.discover_from_actions()
        self._discovered_tools.extend(action_tools)
        
        logger.info(f"Discovered {len(self._discovered_tools)} tools")
        
        return self._discovered_tools
    
    def get_tools_by_safety(self, level: SafetyLevel) -> List[DiscoveredTool]:
        """Get tools filtered by safety level.
        
        Args:
            level: Safety level to filter by
            
        Returns:
            List of tools with matching safety level
        """
        return [t for t in self._discovered_tools if t.safety_level == level]
    
    def get_dangerous_tools(self) -> List[DiscoveredTool]:
        """Get tools requiring confirmation."""
        return self.get_tools_by_safety(SafetyLevel.DANGEROUS)
    
    def to_mcp_tools(self) -> List[Dict[str, Any]]:
        """Convert all discovered tools to MCP format.
        
        Returns:
            List of MCP tool definitions
        """
        return [tool.to_mcp_tool() for tool in self._discovered_tools]
    
    def to_openai_functions(self) -> List[Dict[str, Any]]:
        """Convert all discovered tools to OpenAI function format.
        
        Returns:
            List of OpenAI function definitions
        """
        return [tool.to_openai_function() for tool in self._discovered_tools]
    
    def export_skills(self) -> Dict[str, Any]:
        """Export discovered tools as skill library.
        
        Returns:
            Skill library document
        """
        return {
            "name": "ros_capabilities",
            "version": "1.0.0",
            "tools": [
                {
                    "name": tool.name,
                    "type": tool.type,
                    "description": tool.description,
                    "safety_level": tool.safety_level.value,
                    "parameters": tool.parameters
                }
                for tool in self._discovered_tools
            ],
            "safety_summary": {
                "safe": len(self.get_tools_by_safety(SafetyLevel.SAFE)),
                "medium": len(self.get_tools_by_safety(SafetyLevel.MEDIUM)),
                "dangerous": len(self.get_tools_by_safety(SafetyLevel.DANGEROUS))
            }
        }


# Convenience function
async def discover_tools(bridge: ROSBridge) -> List[DiscoveredTool]:
    """Quick tool discovery.
    
    Args:
        bridge: ROSBridge instance
        
    Returns:
        List of discovered tools
    """
    discovery = ToolDiscovery(bridge)
    return await discovery.discover_all()


__all__ = [
    "ToolDiscovery",
    "DiscoveredTool",
    "SafetyLevel",
    "discover_tools"
]
