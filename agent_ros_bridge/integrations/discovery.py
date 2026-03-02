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

    Supports both ROS1 and ROS2 connectors.

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

    def _get_all_robots(self):
        """Get all robots from all fleets."""
        robots = []
        if not self.bridge:
            return robots

        for fleet in self.bridge.fleets.values():
            for robot in fleet.robots.values():
                robots.append(robot)
        return robots

    def _discover_topics(self) -> List[ROSAction]:
        """Discover ROS topics from connected robots (ROS1 & ROS2)."""
        tools = []

        for robot in self._get_all_robots():
            try:
                connector_type = getattr(robot, "connector_type", "unknown")

                if connector_type == "ros2":
                    tools.extend(self._discover_ros2_topics(robot))
                elif connector_type == "ros1":
                    tools.extend(self._discover_ros1_topics(robot))

            except Exception as e:
                logger.warning(f"Failed to discover topics from {robot.name}: {e}")

        return tools

    def _discover_ros2_topics(self, robot) -> List[ROSAction]:
        """Discover ROS2 topics."""
        tools = []

        if not hasattr(robot, "ros_node") or not robot.ros_node:
            return tools

        try:
            topic_names_and_types = robot.ros_node.get_topic_names_and_types()
            for topic_name, types in topic_names_and_types:
                msg_type = types[0] if types else "unknown"

                direction = self._infer_topic_direction(topic_name)
                tool_name = self._make_tool_name(topic_name, direction)

                tool = ROSAction(
                    name=tool_name,
                    action_type="topic",
                    ros_type=msg_type,
                    description=f"{direction.upper()} ROS2 topic: {topic_name} ({msg_type})",
                    parameters=self._infer_topic_parameters(msg_type, direction),
                    dangerous=self._is_dangerous_topic(topic_name),
                )
                tools.append(tool)
        except Exception as e:
            logger.debug(f"ROS2 topic discovery error: {e}")

        return tools

    def _discover_ros1_topics(self, robot) -> List[ROSAction]:
        """Discover ROS1 topics."""
        tools = []

        try:
            topics = robot._cmd_get_topics()
            for topic_info in topics:
                name = topic_info.get("name", "")
                msg_type = topic_info.get("type", "unknown")

                direction = self._infer_topic_direction(name)
                tool_name = self._make_tool_name(name, direction)

                tool = ROSAction(
                    name=tool_name,
                    action_type="topic",
                    ros_type=msg_type,
                    description=f"{direction.upper()} ROS1 topic: {name} ({msg_type})",
                    parameters=self._infer_topic_parameters(msg_type, direction),
                    dangerous=self._is_dangerous_topic(name),
                )
                tools.append(tool)
        except Exception as e:
            logger.debug(f"ROS1 topic discovery error: {e}")

        return tools

    def _discover_services(self) -> List[ROSAction]:
        """Discover ROS services from connected robots (ROS1 & ROS2)."""
        tools = []

        for robot in self._get_all_robots():
            try:
                connector_type = getattr(robot, "connector_type", "unknown")

                if connector_type == "ros2":
                    tools.extend(self._discover_ros2_services(robot))
                elif connector_type == "ros1":
                    tools.extend(self._discover_ros1_services(robot))

            except Exception as e:
                logger.warning(f"Failed to discover services from {robot.name}: {e}")

        return tools

    def _discover_ros2_services(self, robot) -> List[ROSAction]:
        """Discover ROS2 services."""
        tools = []

        if not hasattr(robot, "ros_node") or not robot.ros_node:
            return tools

        try:
            service_names_and_types = robot.ros_node.get_service_names_and_types()
            for service_name, service_types in service_names_and_types:
                service_type = service_types[0] if service_types else "unknown"

                tool = ROSAction(
                    name=f"call_{service_name.replace('/', '_').strip('_')}",
                    action_type="service",
                    ros_type=service_type,
                    description=f"ROS2 service: {service_name} ({service_type})",
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
            logger.debug(f"ROS2 service discovery error: {e}")

        return tools

    def _discover_ros1_services(self, robot) -> List[ROSAction]:
        """Discover ROS1 services."""
        tools = []

        try:
            services = robot._cmd_get_services()
            for svc_info in services:
                service_name = svc_info.get("name", "")
                providers = svc_info.get("providers", [])

                tool = ROSAction(
                    name=f"call_{service_name.replace('/', '_').strip('_')}",
                    action_type="service",
                    ros_type="unknown",  # ROS1 doesn't easily expose service types
                    description=f"ROS1 service: {service_name} (providers: {len(providers)})",
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
            logger.debug(f"ROS1 service discovery error: {e}")

        return tools

    def _discover_actions(self) -> List[ROSAction]:
        """Discover ROS actions from connected robots (ROS1 & ROS2)."""
        tools = []

        # Common actions for both ROS1 and ROS2
        common_actions = [
            (
                "navigate_to_pose",
                "Navigate to a specific pose",
                {"x": "float", "y": "float", "theta": "float"},
            ),
            ("move_base", "Move base to target", {"target_x": "float", "target_y": "float"}),
            ("pick_object", "Pick up an object", {"object_id": "string", "gripper_force": "float"}),
            ("place_object", "Place object down", {"location_id": "string"}),
            ("follow_waypoints", "Follow a sequence of waypoints", {"waypoints": "array"}),
            ("rotate", "Rotate robot in place", {"angle": "float", "speed": "float"}),
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

    def _make_tool_name(self, topic_name: str, direction: str) -> str:
        """Create a tool name from topic name."""
        clean_name = topic_name.replace("/", "_").strip("_")
        prefix = "read" if direction == "output" else "write"
        return f"{prefix}_{clean_name}"

    def _infer_topic_direction(self, topic_name: str) -> str:
        """Infer if topic is input (command) or output (telemetry)."""
        # Common command topics (inputs to robot)
        command_patterns = [
            "/cmd_vel",
            "/cmd",
            "/command",
            "/goal",
            "/target",
            "/set_",
            "/move_",
            "/joint_command",
            "/gripper_command",
        ]
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
            "/tf",
            "/map",
            "/battery",
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
        type_mappings = {
            "std_msgs/String": {
                "data": {"type": "string", "description": "String message content"}
            },
            "std_msgs/Float64": {"data": {"type": "number", "description": "Float value"}},
            "std_msgs/Float32": {"data": {"type": "number", "description": "Float value"}},
            "std_msgs/Int32": {"data": {"type": "integer", "description": "Integer value"}},
            "std_msgs/Int64": {"data": {"type": "integer", "description": "Integer value"}},
            "std_msgs/Bool": {"data": {"type": "boolean", "description": "Boolean value"}},
            "geometry_msgs/Twist": {
                "linear": {"type": "object", "description": "Linear velocity {x, y, z}"},
                "angular": {"type": "object", "description": "Angular velocity {x, y, z}"},
            },
            "geometry_msgs/Pose": {
                "position": {"type": "object", "description": "Position {x, y, z}"},
                "orientation": {"type": "object", "description": "Orientation quaternion"},
            },
            "geometry_msgs/PoseStamped": {
                "header": {"type": "object", "description": "Message header"},
                "pose": {"type": "object", "description": "Robot pose"},
            },
            "sensor_msgs/LaserScan": {
                "ranges": {"type": "array", "description": "Array of range measurements"},
                "angle_min": {"type": "number", "description": "Start angle of scan"},
                "angle_max": {"type": "number", "description": "End angle of scan"},
            },
            "sensor_msgs/Image": {
                "data": {"type": "string", "description": "Image data (base64)"},
                "encoding": {"type": "string", "description": "Image encoding"},
            },
            "nav_msgs/Odometry": {
                "pose": {"type": "object", "description": "Robot pose {position, orientation}"},
                "twist": {"type": "object", "description": "Robot twist {linear, angular}"},
            },
            "nav_msgs/Path": {
                "poses": {"type": "array", "description": "Array of poses"},
            },
            "sensor_msgs/JointState": {
                "name": {"type": "array", "description": "Joint names"},
                "position": {"type": "array", "description": "Joint positions"},
                "velocity": {"type": "array", "description": "Joint velocities"},
                "effort": {"type": "array", "description": "Joint efforts"},
            },
        }

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
            "/move_base",
        ]
        topic_lower = topic_name.lower()
        return any(pattern in topic_lower for pattern in dangerous_patterns)

    def _is_dangerous_service(self, service_name: str) -> bool:
        """Determine if a service is potentially dangerous."""
        dangerous_patterns = [
            "estop",
            "emergency",
            "power",
            "reboot",
            "reset",
            "home",
            "calibrate",
            "shutdown",
        ]
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
                    "required": list(tool.parameters.keys()) if tool.parameters else [],
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
                        "required": list(tool.parameters.keys()) if tool.parameters else [],
                    },
                },
            }
            functions.append(func)

        return functions

    def to_langchain_tools(self, tools: Optional[List[ROSAction]] = None) -> List[Dict]:
        """Convert to LangChain tool format."""
        if tools is None:
            tools = self.discover_all()

        lc_tools = []
        for tool in tools:
            lc_tool = {
                "name": tool.name,
                "description": tool.description,
                "args_schema": {
                    "type": "object",
                    "properties": tool.parameters,
                },
            }
            lc_tools.append(lc_tool)

        return lc_tools

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
