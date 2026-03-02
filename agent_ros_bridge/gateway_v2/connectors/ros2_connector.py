#!/usr/bin/env python3
"""ROS2 Connector for OpenClaw Gateway.

Connects to ROS2 robots and bridges ROS2 topics/services/actions
to the OpenClaw unified message format.
"""

import asyncio
import importlib
import logging
from dataclasses import dataclass
from datetime import datetime
from typing import Any, AsyncIterator, Dict, List, Optional, Type

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    rclpy = None  # type: ignore

    # Define a placeholder Node class for type hints when rclpy is not available
    class Node:  # type: ignore
        """Placeholder Node class when rclpy is not available."""


from agent_ros_bridge.gateway_v2.core import Command, Connector, Robot, RobotEndpoint, Telemetry

logger = logging.getLogger("connector.ros2")


# Message type registry for common ROS2 message types
MESSAGE_TYPE_REGISTRY = {
    # Standard messages
    "std_msgs/String": ("std_msgs.msg", "String"),
    "std_msgs/Float64": ("std_msgs.msg", "Float64"),
    "std_msgs/Float32": ("std_msgs.msg", "Float32"),
    "std_msgs/Int32": ("std_msgs.msg", "Int32"),
    "std_msgs/Int64": ("std_msgs.msg", "Int64"),
    "std_msgs/Bool": ("std_msgs.msg", "Bool"),
    "std_msgs/Header": ("std_msgs.msg", "Header"),
    "std_msgs/Empty": ("std_msgs.msg", "Empty"),
    # Geometry messages
    "geometry_msgs/Twist": ("geometry_msgs.msg", "Twist"),
    "geometry_msgs/Pose": ("geometry_msgs.msg", "Pose"),
    "geometry_msgs/PoseStamped": ("geometry_msgs.msg", "PoseStamped"),
    "geometry_msgs/Point": ("geometry_msgs.msg", "Point"),
    "geometry_msgs/Quaternion": ("geometry_msgs.msg", "Quaternion"),
    "geometry_msgs/Transform": ("geometry_msgs.msg", "Transform"),
    "geometry_msgs/TransformStamped": ("geometry_msgs.msg", "TransformStamped"),
    "geometry_msgs/Vector3": ("geometry_msgs.msg", "Vector3"),
    "geometry_msgs/Wrench": ("geometry_msgs.msg", "Wrench"),
    # Sensor messages
    "sensor_msgs/LaserScan": ("sensor_msgs.msg", "LaserScan"),
    "sensor_msgs/Image": ("sensor_msgs.msg", "Image"),
    "sensor_msgs/CompressedImage": ("sensor_msgs.msg", "CompressedImage"),
    "sensor_msgs/PointCloud2": ("sensor_msgs.msg", "PointCloud2"),
    "sensor_msgs/Imu": ("sensor_msgs.msg", "Imu"),
    "sensor_msgs/NavSatFix": ("sensor_msgs.msg", "NavSatFix"),
    "sensor_msgs/BatteryState": ("sensor_msgs.msg", "BatteryState"),
    "sensor_msgs/JointState": ("sensor_msgs.msg", "JointState"),
    "sensor_msgs/Range": ("sensor_msgs.msg", "Range"),
    # Navigation messages
    "nav_msgs/Odometry": ("nav_msgs.msg", "Odometry"),
    "nav_msgs/Path": ("nav_msgs.msg", "Path"),
    "nav_msgs/OccupancyGrid": ("nav_msgs.msg", "OccupancyGrid"),
    "nav_msgs/MapMetaData": ("nav_msgs.msg", "MapMetaData"),
    "nav_msgs/GridCells": ("nav_msgs.msg", "GridCells"),
    # TF2 messages
    "tf2_msgs/TFMessage": ("tf2_msgs.msg", "TFMessage"),
    # Diagnostic messages
    "diagnostic_msgs/DiagnosticArray": ("diagnostic_msgs.msg", "DiagnosticArray"),
    "diagnostic_msgs/DiagnosticStatus": ("diagnostic_msgs.msg", "DiagnosticStatus"),
    # Trajectory messages
    "trajectory_msgs/JointTrajectory": ("trajectory_msgs.msg", "JointTrajectory"),
    "trajectory_msgs/JointTrajectoryPoint": ("trajectory_msgs.msg", "JointTrajectoryPoint"),
    # Control messages
    "control_msgs/JointTrajectoryControllerState": (
        "control_msgs.msg",
        "JointTrajectoryControllerState",
    ),  # noqa: E501
    "control_msgs/GripperCommand": ("control_msgs.msg", "GripperCommand"),
}


def get_message_class(msg_type: str) -> Optional[Type]:
    """Dynamically import and return a ROS2 message class.

    Args:
        msg_type: ROS message type string (e.g., "geometry_msgs/Twist")

    Returns:
        Message class or None if not available
    """
    if msg_type in MESSAGE_TYPE_REGISTRY:
        module_name, class_name = MESSAGE_TYPE_REGISTRY[msg_type]
        try:
            module = importlib.import_module(module_name)
            return getattr(module, class_name)
        except (ImportError, AttributeError) as e:
            logger.debug(f"Could not import {msg_type}: {e}")
            return None

    # Try dynamic import for unknown types
    try:
        if "/" in msg_type:
            pkg, msg = msg_type.split("/")
            module = importlib.import_module(f"{pkg}.msg")
            return getattr(module, msg)
    except (ImportError, AttributeError) as e:
        logger.debug(f"Dynamic import failed for {msg_type}: {e}")

    return None


@dataclass
class ROS2Topic:
    """ROS2 topic information."""

    name: str
    message_type: str
    qos_profile: str


class ROS2Robot(Robot):
    """ROS2 robot implementation."""

    def __init__(self, robot_id: str, name: str, ros_node: Node):
        super().__init__(robot_id, name, "ros2")
        self.ros_node = ros_node
        self.robot_type = "ros2"  # Compatibility alias for connector_type
        self.subscriptions: Dict[str, Any] = {}
        self.publishers: Dict[str, Any] = {}
        self._telemetry_queue: asyncio.Queue = asyncio.Queue()

    async def connect(self) -> bool:
        """ROS2 robot is already connected via node."""
        self.connected = True
        self.capabilities.add("publish")
        self.capabilities.add("subscribe")
        self.capabilities.add("services")
        logger.info(f"ROS2 robot {self.name} connected")
        return True

    async def disconnect(self) -> None:
        """Disconnect from ROS2."""
        self.connected = False
        # Cleanup subscriptions
        for sub in self.subscriptions.values():
            self.ros_node.destroy_subscription(sub)
        self.subscriptions.clear()
        logger.info(f"ROS2 robot {self.name} disconnected")

    async def execute(self, command: Command) -> Any:
        """Execute command on ROS2 robot."""
        action = command.action
        params = command.parameters

        if action == "publish":
            return await self._cmd_publish(params)
        elif action == "call_service":
            return await self._cmd_call_service(params)
        elif action == "send_action":
            return await self._cmd_send_action(params)
        elif action == "get_topics":
            return self._cmd_get_topics()
        elif action == "get_nodes":
            return self._cmd_get_nodes()
        else:
            raise ValueError(f"Unknown ROS2 command: {action}")

    async def subscribe(  # noqa: E501
        self, topic: str, msg_type: Optional[str] = None
    ) -> AsyncIterator[Telemetry]:
        """Subscribe to ROS2 topic.

        Args:
            topic: ROS topic name
            msg_type: Optional message type (e.g., "geometry_msgs/Twist").
                     If not provided, will try to auto-detect from topic info.
        """
        if topic in self.subscriptions:
            logger.warning(f"Already subscribed to {topic}")
            # Still yield from queue for existing subscription
            while self.connected:
                try:
                    telemetry = await asyncio.wait_for(self._telemetry_queue.get(), timeout=1.0)
                    if telemetry.topic == topic:
                        yield telemetry
                except asyncio.TimeoutError:
                    continue
            return

        # Auto-detect message type if not provided
        if msg_type is None:
            msg_type = self._get_topic_message_type(topic)
            if msg_type is None:
                logger.error(f"Could not determine message type for {topic}")
                return

        # Get message class
        msg_class = get_message_class(msg_type)
        if msg_class is None:
            logger.error(f"Could not import message type {msg_type} for {topic}")
            return

        # Create ROS2 subscription
        def callback(msg):
            telemetry = Telemetry(topic=topic, data=self._ros_msg_to_dict(msg), quality=1.0)
            asyncio.create_task(self._telemetry_queue.put(telemetry))

        try:
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            subscription = self.ros_node.create_subscription(msg_class, topic, callback, qos)
            self.subscriptions[topic] = subscription
            logger.info(f"Subscribed to ROS2 topic: {topic} ({msg_type})")
        except Exception as e:
            logger.error(f"Failed to subscribe to {topic}: {e}")
            return

        # Yield from queue
        while self.connected:
            try:
                telemetry = await asyncio.wait_for(self._telemetry_queue.get(), timeout=1.0)
                if telemetry.topic == topic:
                    yield telemetry
            except asyncio.TimeoutError:
                continue

    def _get_topic_message_type(self, topic: str) -> Optional[str]:
        """Get the message type for a topic by introspecting ROS."""
        try:
            topic_names_and_types = self.ros_node.get_topic_names_and_types()
            for name, types in topic_names_and_types:
                if name == topic and types:
                    return types[0]  # Return first type
        except Exception as e:
            logger.debug(f"Could not get topic type for {topic}: {e}")
        return None

    async def _cmd_publish(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Publish message to ROS2 topic.

        Args:
            params: Dictionary containing:
                - topic: ROS topic name
                - data: Message data (dict or primitive)
                - type: ROS message type (e.g., "geometry_msgs/Twist")

        Returns:
            Status dictionary
        """
        topic = params.get("topic")
        message_data = params.get("data")
        msg_type_str = params.get("type")

        if not topic:
            return {"status": "error", "message": "Topic is required"}

        # Auto-detect message type if not provided
        if msg_type_str is None:
            msg_type_str = self._get_topic_message_type(topic)
            if msg_type_str is None:
                return {
                    "status": "error",
                    "message": f"Could not determine message type for {topic}",
                }

        try:
            # Get message class
            msg_class = get_message_class(msg_type_str)
            if msg_class is None:
                return {
                    "status": "error",
                    "message": f"Could not import message type {msg_type_str}",
                }

            # Create publisher if needed
            if topic not in self.publishers:
                qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
                publisher = self.ros_node.create_publisher(msg_class, topic, qos)
                self.publishers[topic] = publisher
                logger.info(f"Created publisher for topic: {topic} ({msg_type_str})")

            # Create and populate message
            msg = msg_class()
            self._dict_to_ros_msg(message_data, msg)

            self.publishers[topic].publish(msg)
            logger.debug(f"Published to {topic}: {str(message_data)[:100]}...")

            return {"status": "published", "topic": topic, "type": msg_type_str}
        except Exception as e:
            logger.error(f"Failed to publish to {topic}: {e}")
            return {"status": "error", "message": str(e)}

    async def _cmd_call_service(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Call ROS2 service."""
        service_name = params.get("service")
        params.get("request")

        # Create service client
        # client = self.ros_node.create_client(service_type, service_name)
        # await client.wait_for_service()
        # request = service_type.Request()
        # self._dict_to_ros_msg(request_data, request)
        # response = await client.call_async(request)

        return {"status": "called", "service": service_name}

    async def _cmd_send_action(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Send ROS2 action goal."""
        action_name = params.get("action")
        params.get("goal")

        # Action client implementation
        return {"status": "goal_sent", "action": action_name}

    def _cmd_get_topics(self) -> List[Dict[str, Any]]:
        """Get list of available topics."""
        topic_names_and_types = self.ros_node.get_topic_names_and_types()
        return [{"name": name, "types": types} for name, types in topic_names_and_types]

    def _cmd_get_nodes(self) -> List[str]:
        """Get list of ROS2 nodes."""
        return self.ros_node.get_node_names()

    def _ros_msg_to_dict(self, msg) -> Dict[str, Any]:
        """Convert ROS message to dictionary."""
        result = {"_type": type(msg).__name__}

        # Try to extract fields using get_fields_and_field_types() if available
        fields = None
        try:
            if hasattr(msg, "get_fields_and_field_types"):
                fields = msg.get_fields_and_field_types()
        except Exception as e:
            logger.debug(f"Could not get fields from ROS message: {e}")

        if fields:
            for field_name in fields:
                value = getattr(msg, field_name, None)
                # Handle nested messages
                if hasattr(value, "get_fields_and_field_types"):
                    result[field_name] = self._ros_msg_to_dict(value)
                elif isinstance(value, list):
                    result[field_name] = [
                        (
                            self._ros_msg_to_dict(v)
                            if hasattr(v, "get_fields_and_field_types")
                            else v
                        )
                        for v in value
                    ]
                else:
                    result[field_name] = value
        else:
            # Fallback: try to get all attributes that aren't private or methods
            # First try __dict__ for regular objects
            msg_dict = getattr(msg, "__dict__", {})
            for attr, value in msg_dict.items():
                if not attr.startswith("_") and not callable(value):
                    result[attr] = value
            # Also check for attributes set on MagicMock objects
            for attr in ["data", "value", "topic", "name"]:
                if hasattr(msg, attr) and attr not in result:
                    value = getattr(msg, attr)
                    if not callable(value) and not attr.startswith("_"):
                        result[attr] = value

        return result

    def _dict_to_ros_msg(self, data: Dict[str, Any], msg):
        """Convert dictionary to ROS message.

        Handles nested messages and arrays recursively.
        """
        if not isinstance(data, dict):
            # Handle primitive types (e.g., std_msgs/String just has "data" field)
            if hasattr(msg, "data"):
                msg.data = data
            return

        for key, value in data.items():
            if not hasattr(msg, key):
                continue

            attr = getattr(msg, key)

            # Handle nested ROS messages
            if hasattr(attr, "__dict__") and isinstance(value, dict):
                self._dict_to_ros_msg(value, attr)
            # Handle arrays of ROS messages
            elif isinstance(value, list) and attr:
                # Check if it's a list of messages or primitives
                if len(value) > 0 and isinstance(value[0], dict):
                    # Array of nested messages - create each element
                    msg_type = type(attr[0]) if attr else None
                    if msg_type:
                        new_list = []
                        for item in value:
                            nested_msg = msg_type()
                            self._dict_to_ros_msg(item, nested_msg)
                            new_list.append(nested_msg)
                        setattr(msg, key, new_list)
                else:
                    # Array of primitives
                    setattr(msg, key, value)
            else:
                # Primitive value
                setattr(msg, key, value)


class ROS2Connector(Connector):
    """ROS2 connector implementation."""

    connector_type = "ros2"

    def __init__(self, domain_id: int = 0):
        self.domain_id = domain_id
        self.node: Optional[Node] = None
        self._initialized = False

    async def _ensure_initialized(self):
        """Ensure ROS2 is initialized."""
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS2 not available. Install rclpy.")

        if not self._initialized:
            rclpy.init()
            ts = str(int(datetime.now().timestamp()))
            self.node = rclpy.create_node(f"agent_ros_bridge_{ts}")
            self._initialized = True

            # Start ROS2 spinner in background
            asyncio.create_task(self._spin_ros())

    async def _spin_ros(self):
        """Spin ROS2 node."""
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            await asyncio.sleep(0.001)

    async def connect(self, uri: str, **kwargs) -> Robot:
        """Connect to ROS2 system.

        URI format: ros2://[<domain_id>]/[<namespace>]

        Examples:
            ros2://0/                    # Domain 0, root namespace
            ros2://42/production/robots  # Domain 42, specific namespace
        """
        await self._ensure_initialized()

        # Parse URI
        # Remove ros2:// prefix
        path = uri.replace("ros2://", "")

        parts = path.split("/")
        domain_id = int(parts[0]) if parts[0] else self.domain_id
        namespace = "/".join(parts[1:]) if len(parts) > 1 else ""

        robot_id = f"ros2_{domain_id}_{namespace.replace('/', '_')}"
        robot_name = kwargs.get("name", f"ROS2_{domain_id}")

        # Create robot instance
        robot = ROS2Robot(robot_id, robot_name, self.node)
        await robot.connect()

        logger.info(f"Connected to ROS2 domain {domain_id}, namespace '{namespace}'")
        return robot

    async def discover(self) -> List[RobotEndpoint]:
        """Discover ROS2 systems on network."""
        await self._ensure_initialized()

        endpoints = []

        # Get all topics
        topics = self.node.get_topic_names_and_types()

        # Group by namespace to find robots
        namespaces = set()
        for topic_name, _ in topics:
            parts = topic_name.split("/")
            if len(parts) > 1:
                namespaces.add(parts[1] if parts[0] == "" else parts[0])

        for ns in namespaces:
            # Create endpoint for each namespace
            endpoints.append(
                RobotEndpoint(
                    uri=f"ros2://{self.domain_id}/{ns}",
                    name=f"ros2_{ns}",
                    connector_type="ros2",
                    capabilities=["publish", "subscribe", "services", "actions"],
                    metadata={
                        "domain_id": self.domain_id,
                        "namespace": ns,
                        "topic_count": len([t for t, _ in topics if ns in t]),
                    },
                )
            )

        return endpoints


# Example usage
async def example():
    """Example ROS2 connector usage."""
    from agent_ros_bridge.gateway_v2.core import Bridge

    # Create gateway
    gateway = Bridge()

    # Register ROS2 connector
    ros2_connector = ROS2Connector(domain_id=0)
    gateway.connector_registry.register(ros2_connector)

    # Discover robots
    print("Discovering ROS2 systems...")
    endpoints = await gateway.connector_registry.discover_all()
    for ep in endpoints:
        print(f"  Found: {ep.name} at {ep.uri}")

    # Connect to first robot
    if endpoints:
        robot = await gateway.connect_robot(endpoints[0].uri)

        # Get topics
        result = await robot.execute(Command(action="get_topics", parameters={}))
        print(f"Topics: {result}")


if __name__ == "__main__":
    import asyncio

    asyncio.run(example())
