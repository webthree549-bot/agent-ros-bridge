"""Tool Discovery - Auto-discover ROS tools and export to AI formats."""

import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


@dataclass
class ROSAction:
    """Discovered ROS action."""

    name: str
    action_type: str  # "topic", "service", "action"
    ros_type: str
    description: str
    parameters: Dict[str, Any]
    dangerous: bool = False


class ToolDiscovery:
    """Discover ROS tools and export to AI-compatible formats.

    Example:
        discovery = ToolDiscovery(bridge)
        tools = discovery.discover_all()

        # Export to MCP format
        mcp_tools = discovery.to_mcp_tools(tools)

        # Export to OpenAI format
        openai_tools = discovery.to_openai_functions(tools)
    """

    def __init__(self, bridge=None):
        self.bridge = bridge
        self._cache: Dict[str, ROSAction] = {}
        logger.info("ToolDiscovery initialized")

    def discover_all(self) -> List[ROSAction]:
        """Discover all available ROS tools."""
        tools = []

        if self.bridge:
            # Discover from actual ROS
            tools.extend(self._discover_topics())
            tools.extend(self._discover_services())
            tools.extend(self._discover_actions())
        else:
            # Return cached or default
            tools = list(self._cache.values())

        # Update cache
        for tool in tools:
            self._cache[tool.name] = tool

        return tools

    def _discover_topics(self) -> List[ROSAction]:
        """Discover ROS topics from connected robots."""
        tools = []

        if not self.bridge:
            return tools

        # Get all connected robots
        for fleet in self.bridge.fleets.values():
            for robot in fleet.robots.values():
                if hasattr(robot, "_cmd_get_topics"):
                    try:
                        topics = robot._cmd_get_topics()
                        for topic_info in topics:
                            name = topic_info.get("name", "")
                            types = topic_info.get("types", [])
                            msg_type = types[0] if types else "unknown"

                            # Determine if this is a publisher or subscriber topic
                            # Heuristic: /cmd_vel, /move_base are typically inputs
                            # /odom, /scan, /camera are typically outputs
                            direction = self._infer_topic_direction(name)

                            tool_name = (
                                f"read_{name.replace('/', '_').strip('_')}"
                                if direction == "output"
                                else f"write_{name.replace('/', '_').strip('_')}"
                            )
                            tool = ROSAction(
                                name=tool_name,
                                action_type="topic",
                                ros_type=msg_type,
                                description=f"{direction.upper()} ROS topic: {name} ({msg_type})",
                                parameters=self._infer_topic_parameters(msg_type, direction),
                                dangerous=self._is_dangerous_topic(name),
                            )
                            tools.append(tool)
                    except Exception as e:
                        logger.warning(f"Failed to discover topics from {robot.name}: {e}")

        return tools

    def _discover_services(self) -> List[ROSAction]:
        """Discover ROS services from connected robots."""
        tools = []

        if not self.bridge:
            return tools

        for fleet in self.bridge.fleets.values():
            for robot in fleet.robots.values():
                if hasattr(robot, "ros_node") and robot.ros_node:
                    try:
                        # Get services from ROS2 node
                        service_names_and_types = robot.ros_node.get_service_names_and_types()
                        for service_name, service_types in service_names_and_types:
                            service_type = service_types[0] if service_types else "unknown"

                            tool = ROSAction(
                                name=f"call_{service_name.replace('/', '_').strip('_')}",
                                action_type="service",
                                ros_type=service_type,
                                description=f"ROS service: {service_name} ({service_type})",
                                parameters={
                                    "request": {
                                        "type": "object",
                                        "description": "Service request data",
                                    }
                                },
                                dangerous=self._is_dangerous_service(service_name),
                            )
                            tools.append(tool)
                    except Exception as e:
                        logger.warning(f"Failed to discover services from {robot.name}: {e}")

        return tools

    def _discover_actions(self) -> List[ROSAction]:
        """Discover ROS actions from connected robots."""
        tools = []
        # Actions discovery would require action client introspection
        # For now, add common actions as templates

        common_actions = [
            (
                "navigate_to_pose",
                "Navigate to a specific pose",
                {"x": "float", "y": "float", "theta": "float"},
            ),  # noqa: E501
            ("move_base", "Move base to target", {"target_x": "float", "target_y": "float"}),
            ("pick_object", "Pick up an object", {"object_id": "string", "gripper_force": "float"}),
            ("place_object", "Place object down", {"location_id": "string"}),
        ]

        for name, desc, params in common_actions:
            tool_params = {
                k: {"type": v, "description": f"{k} parameter"} for k, v in params.items()
            }
            tool = ROSAction(
                name=name,
                action_type="action",
                ros_type=f"{name}_action/Goal",
                description=desc,
                parameters=tool_params,
                dangerous=name in ["pick_object", "place_object"],
            )
            tools.append(tool)

        return tools

    def _infer_topic_direction(self, topic_name: str) -> str:
        """Infer if topic is input (command) or output (telemetry)."""
        # Common command topics (inputs to robot)
        command_patterns = ["/cmd_vel", "/cmd", "/command", "/goal", "/target", "/set_", "/move_"]
        # Common telemetry topics (outputs from robot)
        telemetry_patterns = [
            "/odom",
            "/scan",
            "/camera",
            "/imu",
            "/gps",
            "/status",
            "/state",
            "/feedback",
            "/joint_states",
        ]

        topic_lower = topic_name.lower()

        for pattern in command_patterns:
            if pattern in topic_lower:
                return "input"

        for pattern in telemetry_patterns:
            if pattern in topic_lower:
                return "output"

        # Default: assume it's readable
        return "output"

    def _infer_topic_parameters(self, msg_type: str, direction: str) -> Dict[str, Any]:
        """Infer parameters from message type."""
        # Common ROS message type mappings
        # fmt: off
        type_mappings = {
            'std_msgs/String': {"data": {"type": "string", "description": "String message content"}},  # noqa: E501
            'std_msgs/Float64': {"data": {"type": "number", "description": "Float value"}},
            'std_msgs/Int32': {"data": {"type": "integer", "description": "Integer value"}},
            'std_msgs/Bool': {"data": {"type": "boolean", "description": "Boolean value"}},
            'geometry_msgs/Twist': {
                "linear": {"type": "object", "description": "Linear velocity {x, y, z}"},
                "angular": {"type": "object", "description": "Angular velocity {x, y, z}"}
            },
            'geometry_msgs/Pose': {
                "position": {"type": "object", "description": "Position {x, y, z}"},
                "orientation": {"type": "object", "description": "Orientation quaternion"}  # noqa: E501
            },
            'sensor_msgs/LaserScan': {
                "ranges": {"type": "array", "description": "Array of range measurements"},
                "angle_min": {"type": "number", "description": "Start angle of scan"},
                "angle_max": {"type": "number", "description": "End angle of scan"}
            },
            'nav_msgs/Odometry': {
                "pose": {"type": "object", "description": "Robot pose {position, orientation}"},  # noqa: E501
                "twist": {"type": "object", "description": "Robot twist {linear, angular}"}
            },
        }
        # fmt: on

        if msg_type in type_mappings and direction == "input":
            return type_mappings[msg_type]
        elif direction == "output":
            return {}  # No parameters needed for reading
        else:
            return {"data": {"type": "any", "description": f"Data for {msg_type}"}}

    def _is_dangerous_topic(self, topic_name: str) -> bool:
        """Determine if a topic is potentially dangerous."""
        dangerous_patterns = [
            "/cmd_vel",
            "/joint_command",
            "/gripper",
            "/arm",
            "/dangerous",
            "/emergency",
            "/estop",
            "/power",
            "/motor",
        ]
        topic_lower = topic_name.lower()
        return any(pattern in topic_lower for pattern in dangerous_patterns)

    def _is_dangerous_service(self, service_name: str) -> bool:
        """Determine if a service is potentially dangerous."""
        dangerous_patterns = ["estop", "emergency", "power", "reboot", "reset", "home", "calibrate"]
        service_lower = service_name.lower()
        return any(pattern in service_lower for pattern in dangerous_patterns)

    def to_mcp_tools(self, tools: Optional[List[ROSAction]] = None) -> List[Dict]:
        """Convert to MCP (Model Context Protocol) tool format."""
        if tools is None:
            tools = self.discover_all()

        mcp_tools = []
        for tool in tools:
            mcp_tool = {
                "name": tool.name,
                "description": tool.description,
                "inputSchema": {
                    "type": "object",
                    "properties": tool.parameters,
                    "required": list(tool.parameters.keys()),
                },
            }
            mcp_tools.append(mcp_tool)

        return mcp_tools

    def to_openai_functions(self, tools: Optional[List[ROSAction]] = None) -> List[Dict]:
        """Convert to OpenAI function calling format."""
        if tools is None:
            tools = self.discover_all()

        functions = []
        for tool in tools:
            func = {
                "type": "function",
                "function": {
                    "name": tool.name,
                    "description": tool.description,
                    "parameters": {
                        "type": "object",
                        "properties": tool.parameters,
                        "required": list(tool.parameters.keys()),
                    },
                },
            }
            functions.append(func)

        return functions

    def get_dangerous_tools(self) -> List[ROSAction]:
        """Get list of dangerous tools requiring confirmation."""
        all_tools = self.discover_all()
        return [t for t in all_tools if t.dangerous]

    def get_tool(self, name: str) -> Optional[ROSAction]:
        """Get a specific tool by name."""
        return self._cache.get(name)

    def invalidate_cache(self):
        """Invalidate tool cache."""
        self._cache.clear()
        logger.debug("Tool cache invalidated")
