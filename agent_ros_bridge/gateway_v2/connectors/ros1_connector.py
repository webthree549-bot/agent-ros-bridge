#!/usr/bin/env python3
"""ROS1 Connector for OpenClaw Gateway.

Connects to ROS1 robots (Noetic) and bridges ROS topics/services/actions
to the OpenClaw unified message format.
"""

import asyncio
import importlib
import logging
from dataclasses import dataclass
from datetime import datetime
from typing import Any, AsyncIterator, Dict, List, Optional, Type

try:
    import rospy

    ROS1_AVAILABLE = True
except ImportError:
    ROS1_AVAILABLE = False
    rospy = None  # type: ignore

from agent_ros_bridge.gateway_v2.core import Command, Connector, Robot, RobotEndpoint, Telemetry

logger = logging.getLogger("connector.ros1")


# Message type registry for common ROS1 message types
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
    "std_msgs/UInt8": ("std_msgs.msg", "UInt8"),
    "std_msgs/UInt16": ("std_msgs.msg", "UInt16"),
    "std_msgs/UInt32": ("std_msgs.msg", "UInt32"),
    "std_msgs/UInt64": ("std_msgs.msg", "UInt64"),
    "std_msgs/Int8": ("std_msgs.msg", "Int8"),
    "std_msgs/Int16": ("std_msgs.msg", "Int16"),
    "std_msgs/ColorRGBA": ("std_msgs.msg", "ColorRGBA"),
    "std_msgs/Time": ("std_msgs.msg", "Time"),
    "std_msgs/Duration": ("std_msgs.msg", "Duration"),
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
    "geometry_msgs/PoseWithCovariance": ("geometry_msgs.msg", "PoseWithCovariance"),
    "geometry_msgs/TwistWithCovariance": ("geometry_msgs.msg", "TwistWithCovariance"),
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
    "sensor_msgs/PointCloud": ("sensor_msgs.msg", "PointCloud"),
    "sensor_msgs/ChannelFloat32": ("sensor_msgs.msg", "ChannelFloat32"),
    # Navigation messages
    "nav_msgs/Odometry": ("nav_msgs.msg", "Odometry"),
    "nav_msgs/Path": ("nav_msgs.msg", "Path"),
    "nav_msgs/OccupancyGrid": ("nav_msgs.msg", "OccupancyGrid"),
    "nav_msgs/MapMetaData": ("nav_msgs.msg", "MapMetaData"),
    "nav_msgs/GridCells": ("nav_msgs.msg", "GridCells"),
    "nav_msgs/GetMap": ("nav_msgs.srv", "GetMap"),
    # TF messages
    "tf2_msgs/TFMessage": ("tf2_msgs.msg", "TFMessage"),
    "tf/tfMessage": ("tf.msg", "tfMessage"),
    # Diagnostic messages
    "diagnostic_msgs/DiagnosticArray": ("diagnostic_msgs.msg", "DiagnosticArray"),
    "diagnostic_msgs/DiagnosticStatus": ("diagnostic_msgs.msg", "DiagnosticStatus"),
    # Trajectory messages
    "trajectory_msgs/JointTrajectory": ("trajectory_msgs.msg", "JointTrajectory"),
    "trajectory_msgs/JointTrajectoryPoint": ("trajectory_msgs.msg", "JointTrajectoryPoint"),
    "trajectory_msgs/MultiDOFJointTrajectory": ("trajectory_msgs.msg", "MultiDOFJointTrajectory"),
    # Control messages
    "control_msgs/JointTrajectoryControllerState": (
        "control_msgs.msg",
        "JointTrajectoryControllerState",
    ),
    "control_msgs/GripperCommand": ("control_msgs.msg", "GripperCommand"),
    # Actionlib messages
    "actionlib/GoalStatus": ("actionlib_msgs.msg", "GoalStatus"),
    "actionlib/GoalStatusArray": ("actionlib_msgs.msg", "GoalStatusArray"),
}


def get_message_class(msg_type: str) -> Optional[Type]:
    """Dynamically import and return a ROS1 message class.

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
class ROS1Topic:
    """ROS1 topic information."""

    name: str
    message_type: str


@dataclass
class ROS1Service:
    """ROS1 service information."""

    name: str
    service_type: str


@dataclass
class ROS1Action:
    """ROS1 action information."""

    name: str
    action_type: str


class ROS1Robot(Robot):
    """ROS1 robot implementation."""

    def __init__(self, robot_id: str, name: str):
        """Initialize ROS1 robot with ID and name.

        Args:
            robot_id: Unique identifier for the robot.
            name: Human-readable robot name.
        """
        super().__init__(robot_id, name, "ros1")
        self.robot_type = "ros1"  # Compatibility alias
        self.subscribers: Dict[str, Any] = {}
        self.publishers: Dict[str, Any] = {}
        self.service_proxies: Dict[str, Any] = {}
        self._telemetry_queue: asyncio.Queue = asyncio.Queue()
        self._node_name: Optional[str] = None

    async def connect(self) -> bool:
        """Initialize ROS1 node."""
        if not ROS1_AVAILABLE:
            logger.error("ROS1 (rospy) not available")
            return False

        try:
            # Generate unique node name
            ts = str(int(datetime.now().timestamp()))
            self._node_name = f"agent_ros_bridge_{self.robot_id}_{ts}"
            rospy.init_node(self._node_name, anonymous=True)
            self.connected = True
            self.capabilities.add("publish")
            self.capabilities.add("subscribe")
            self.capabilities.add("services")
            self.capabilities.add("actions")
            logger.info(f"ROS1 robot {self.name} connected (node: {self._node_name})")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to ROS1: {e}")
            return False

    async def disconnect(self) -> None:
        """Disconnect from ROS1."""
        self.connected = False
        # Unregister all subscribers and publishers
        for sub in self.subscribers.values():
            sub.unregister()
        for pub in self.publishers.values():
            pub.unregister()
        self.subscribers.clear()
        self.publishers.clear()
        self.service_proxies.clear()
        logger.info(f"ROS1 robot {self.name} disconnected")

    async def execute(self, command: Command) -> Any:
        """Execute command on ROS1 robot."""
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
        elif action == "get_services":
            return self._cmd_get_services()
        elif action == "get_params":
            return self._cmd_get_params(params)
        elif action == "set_param":
            return self._cmd_set_param(params)
        else:
            raise ValueError(f"Unknown ROS1 command: {action}")

    async def subscribe(
        self, topic: str, msg_type: Optional[str] = None
    ) -> AsyncIterator[Telemetry]:
        """Subscribe to ROS1 topic.

        Args:
            topic: ROS topic name
            msg_type: Optional message type (e.g., "geometry_msgs/Twist").
                     If not provided, will try to auto-detect from topic info.
        """
        if topic in self.subscribers:
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

        # Create ROS1 subscriber
        def callback(msg):
            telemetry = Telemetry(topic=topic, data=self._ros_msg_to_dict(msg), quality=1.0)
            asyncio.create_task(self._telemetry_queue.put(telemetry))

        try:
            subscriber = rospy.Subscriber(topic, msg_class, callback)
            self.subscribers[topic] = subscriber
            logger.info(f"Subscribed to ROS1 topic: {topic} ({msg_type})")
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
            topics = rospy.get_published_topics()
            for name, msg_type in topics:
                if name == topic:
                    return msg_type
        except Exception as e:
            logger.debug(f"Could not get topic type for {topic}: {e}")
        return None

    async def _cmd_publish(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Publish message to ROS1 topic.

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
                self.publishers[topic] = rospy.Publisher(topic, msg_class, queue_size=10)
                rospy.sleep(0.1)  # Allow publisher to register
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
        """Call ROS1 service."""
        service_name = params.get("service")
        request_data = params.get("request", {})

        try:
            rospy.wait_for_service(service_name, timeout=5.0)

            # Get service type from system state
            service_type = self._get_service_type(service_name)
            if service_type is None:
                return {
                    "status": "error",
                    "message": f"Could not determine service type for {service_name}",
                }

            # Import service class
            try:
                pkg, srv = service_type.split("/")
                srv_module = importlib.import_module(f"{pkg}.srv")
                srv_class = getattr(srv_module, srv)
            except Exception as e:
                return {
                    "status": "error",
                    "message": f"Could not import service type {service_type}: {e}",
                }

            # Create service proxy
            proxy = rospy.ServiceProxy(service_name, srv_class)
            self.service_proxies[service_name] = proxy

            # Build request
            request = srv_class._request_class()
            self._dict_to_ros_msg(request_data, request)

            # Call service
            response = proxy(request)

            return {
                "status": "called",
                "service": service_name,
                "response": self._ros_msg_to_dict(response),
            }
        except rospy.ROSException as e:
            return {"status": "error", "message": f"Service timeout: {e}"}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def _get_service_type(self, service_name: str) -> Optional[str]:
        """Get service type from ROS master."""
        try:
            import rosgraph

            master = rosgraph.Master(self._node_name or "agent_ros_bridge")
            _, _, services = master.getSystemState()
            for svc, nodes in services:
                if svc == service_name and nodes:
                    # Get service type from first provider
                    # This is complex in ROS1, often requires parsing node XML-RPC
                    # For now, return a common default or None
                    return None
        except Exception as e:
            logger.debug(f"Could not get service type for {service_name}: {e}")
        return None

    async def _cmd_send_action(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Send ROS1 action goal."""
        action_name = params.get("action")
        goal_data = params.get("goal", {})
        action_type = params.get("type")

        # ROS1 actions use actionlib
        # This is a simplified implementation
        try:
            import actionlib

            if action_type is None:
                return {"status": "error", "message": "Action type required for ROS1 actions"}

            # Parse action type
            pkg, action = action_type.split("/")
            action_module = importlib.import_module(f"{pkg}.msg")
            action_class = getattr(action_module, action + "Action")
            goal_class = getattr(action_module, action + "Goal")

            # Create action client
            client = actionlib.SimpleActionClient(action_name, action_class)
            client.wait_for_server(timeout=rospy.Duration(5.0))

            # Create and send goal
            goal = goal_class()
            self._dict_to_ros_msg(goal_data, goal)
            client.send_goal(goal)

            return {"status": "goal_sent", "action": action_name}
        except Exception as e:
            logger.error(f"Failed to send action goal: {e}")
            return {"status": "error", "message": str(e)}

    def _cmd_get_topics(self) -> List[Dict[str, Any]]:
        """Get list of available topics."""
        if not ROS1_AVAILABLE:
            return []
        try:
            topics = rospy.get_published_topics()
            return [{"name": name, "type": msg_type} for name, msg_type in topics]
        except Exception as e:
            logger.error(f"Failed to get topics: {e}")
            return []

    def _cmd_get_nodes(self) -> List[str]:
        """Get list of ROS1 nodes."""
        if not ROS1_AVAILABLE:
            return []
        try:
            import rosgraph

            master = rosgraph.Master(self._node_name or "agent_ros_bridge")
            code, _, nodes = master.getNodeNames()
            if code == 1:
                return nodes
            return []
        except Exception as e:
            logger.error(f"Failed to get nodes: {e}")
            return []

    def _cmd_get_services(self) -> List[Dict[str, Any]]:
        """Get list of available services."""
        if not ROS1_AVAILABLE:
            return []
        try:
            import rosgraph

            master = rosgraph.Master(self._node_name or "agent_ros_bridge")
            _, _, services = master.getSystemState()
            return [{"name": svc, "providers": nodes} for svc, nodes in services]
        except Exception as e:
            logger.error(f"Failed to get services: {e}")
            return []

    def _cmd_get_params(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Get ROS parameters."""
        namespace = params.get("namespace", "")
        try:
            import rosgraph

            master = rosgraph.Master(self._node_name or "agent_ros_bridge")
            code, msg, params_list = master.getParamNames()
            if code == 1:
                filtered = (
                    [p for p in params_list if p.startswith(namespace)]
                    if namespace
                    else params_list
                )
                return {"status": "ok", "params": filtered}
            return {"status": "error", "message": msg}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def _cmd_set_param(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Set ROS parameter."""
        param_name = params.get("name")
        param_value = params.get("value")
        try:
            rospy.set_param(param_name, param_value)
            return {"status": "set", "param": param_name}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def _ros_msg_to_dict(self, msg) -> Dict[str, Any]:
        """Convert ROS message to dictionary."""
        result = {"_type": type(msg).__name__}

        # ROS1 messages use __slots__
        if hasattr(msg, "__slots__"):
            for slot in msg.__slots__:
                if slot.startswith("_"):
                    continue
                try:
                    value = getattr(msg, slot)
                    # Handle nested messages
                    if hasattr(value, "__slots__"):
                        result[slot] = self._ros_msg_to_dict(value)
                    elif isinstance(value, list):
                        result[slot] = [
                            self._ros_msg_to_dict(v) if hasattr(v, "__slots__") else v
                            for v in value
                        ]
                    else:
                        result[slot] = value
                except Exception as e:
                    logger.debug(f"Could not get slot {slot}: {e}")
        else:
            # Fallback: try __dict__ for regular objects (e.g., MagicMock in tests)
            msg_dict = getattr(msg, "__dict__", {})
            for attr, value in msg_dict.items():
                if not attr.startswith("_") and not callable(value):
                    result[attr] = value

        return result

    def _dict_to_ros_msg(self, data: Any, msg):
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
            if hasattr(attr, "__slots__") and isinstance(value, dict):
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


class ROS1Connector(Connector):
    """ROS1 (Noetic) connector."""

    connector_type = "ros1"

    def __init__(self, master_uri: Optional[str] = None):
        """Initialize ROS1 connector with optional master URI.

        Args:
            master_uri: ROS master URI (e.g., "http://localhost:11311").
        """
        self.master_uri = master_uri
        self.robots: Dict[str, ROS1Robot] = {}
        self._initialized = False

    async def _ensure_initialized(self):
        """Ensure ROS1 environment is set up."""
        if not ROS1_AVAILABLE:
            raise RuntimeError("ROS1 not available. Install rospy.")

        if self.master_uri:
            import os

            os.environ["ROS_MASTER_URI"] = self.master_uri
        self._initialized = True

    async def discover(self) -> List[RobotEndpoint]:
        """Discover ROS1 systems on network."""
        if not ROS1_AVAILABLE:
            return []

        await self._ensure_initialized()
        endpoints = []

        try:
            # Get system state
            import rosgraph

            master = rosgraph.Master("agent_ros_bridge_discovery")

            # Get published topics
            code, status, topics = master.getPublishedTopics("/")
            if code == 1:
                # Group by namespace to find robots
                namespaces = set()
                for topic_name, _ in topics:
                    parts = topic_name.strip("/").split("/")
                    if parts and parts[0]:
                        namespaces.add(parts[0])

                for ns in namespaces:
                    topic_count = len([t for t, _ in topics if ns in t])
                    endpoints.append(
                        RobotEndpoint(
                            uri=f"ros1:///{ns}",
                            name=f"ros1_{ns}",
                            connector_type="ros1",
                            capabilities=["publish", "subscribe", "services", "actions"],
                            metadata={
                                "namespace": ns,
                                "topic_count": topic_count,
                                "ros_version": "noetic",
                            },
                        )
                    )

            # Also add individual nodes as endpoints
            code, status, nodes = master.getNodeNames()
            if code == 1:
                for node in nodes:
                    if node.startswith("/"):
                        node_clean = node[1:].replace("/", "_")
                    else:
                        node_clean = node.replace("/", "_")
                    endpoints.append(
                        RobotEndpoint(
                            uri=f"ros1://{node_clean}",
                            name=node_clean,
                            connector_type="ros1",
                            capabilities=["publish", "subscribe"],
                            metadata={"node_name": node, "ros_version": "noetic"},
                        )
                    )

        except Exception as e:
            logger.error(f"ROS1 discovery failed: {e}")

        return endpoints

    async def connect(self, uri: str, **kwargs) -> Optional[Robot]:
        """Connect to a ROS1 robot.

        URI formats:
            ros1:///<namespace>         # Connect to namespace
            ros1://<node_name>          # Connect to specific node
        """
        await self._ensure_initialized()

        # Parse URI
        path = uri.replace("ros1://", "").lstrip("/")
        robot_id = path.replace("/", "_") if path else "default"
        robot_name = kwargs.get("name", f"ROS1_{robot_id}")

        robot = ROS1Robot(robot_id, robot_name)
        if await robot.connect():
            self.robots[robot_id] = robot
            return robot
        return None


# Example usage
async def example():
    """Example ROS1 connector usage."""
    from agent_ros_bridge.gateway_v2.core import Bridge

    # Create gateway
    gateway = Bridge()

    # Register ROS1 connector
    ros1_connector = ROS1Connector()
    gateway.connector_registry.register(ros1_connector)

    # Discover robots
    print("Discovering ROS1 systems...")
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
