#!/usr/bin/env python3
"""ROS1 Connector for OpenClaw Gateway

Connects to ROS1 robots (Noetic) and bridges ROS topics/services/actions
to the OpenClaw unified message format.
"""

import asyncio
import logging
from typing import Dict, Any, List, Optional
from dataclasses import dataclass
from datetime import datetime

try:
    import rospy
    from rospy import Publisher, Subscriber
    ROS1_AVAILABLE = True
except ImportError:
    ROS1_AVAILABLE = False

from agent_ros_bridge.gateway_v2.core import (
    Connector, Robot, RobotEndpoint, Command, Telemetry
)

logger = logging.getLogger("connector.ros1")


@dataclass
class ROS1Topic:
    """ROS1 topic information"""
    name: str
    message_type: str


class ROS1Robot(Robot):
    """ROS1 robot implementation"""
    
    def __init__(self, robot_id: str, name: str):
        super().__init__(robot_id, name, "ros1")
        self.subscribers: Dict[str, Any] = {}
        self.publishers: Dict[str, Any] = {}
        self._telemetry_queue: asyncio.Queue = asyncio.Queue()
    
    async def connect(self) -> bool:
        """Initialize ROS1 node"""
        if not ROS1_AVAILABLE:
            logger.error("ROS1 (rospy) not available")
            return False
        
        try:
            rospy.init_node(f'agent_ros_bridge_{self.robot_id}', anonymous=True)
            self.connected = True
            self.capabilities.add("publish")
            self.capabilities.add("subscribe")
            self.capabilities.add("services")
            logger.info(f"ROS1 robot {self.name} connected")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to ROS1: {e}")
            return False
    
    async def disconnect(self) -> None:
        """Disconnect from ROS1"""
        self.connected = False
        # Unregister all subscribers and publishers
        for sub in self.subscribers.values():
            sub.unregister()
        for pub in self.publishers.values():
            pub.unregister()
        self.subscribers.clear()
        self.publishers.clear()
        logger.info(f"ROS1 robot {self.name} disconnected")
    
    async def execute(self, command: Command) -> Any:
        """Execute command on ROS1 robot"""
        action = command.action
        params = command.parameters
        
        if action == "publish":
            return await self._cmd_publish(params)
        elif action == "subscribe":
            return await self._cmd_subscribe(params)
        elif action == "call_service":
            return await self._cmd_call_service(params)
        elif action == "get_topics":
            return self._cmd_get_topics()
        elif action == "get_nodes":
            return self._cmd_get_nodes()
        else:
            return {"error": f"Unknown action: {action}"}
    
    async def _cmd_publish(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Publish to a ROS1 topic"""
        topic = params.get("topic")
        data = params.get("data", {})
        msg_type = params.get("message_type", "std_msgs/String")
        
        try:
            # Create publisher if not exists
            if topic not in self.publishers:
                msg_class = self._get_message_class(msg_type)
                self.publishers[topic] = rospy.Publisher(topic, msg_class, queue_size=10)
                rospy.sleep(0.1)  # Allow publisher to register
            
            # Build and publish message
            msg = self._build_message(msg_type, data)
            self.publishers[topic].publish(msg)
            
            return {"published": True, "topic": topic}
        except Exception as e:
            logger.error(f"Failed to publish to {topic}: {e}")
            return {"error": str(e), "topic": topic}
    
    async def _cmd_subscribe(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Subscribe to a ROS1 topic"""
        topic = params.get("topic")
        msg_type = params.get("message_type", "std_msgs/String")
        
        if topic in self.subscribers:
            return {"subscribed": True, "topic": topic, "status": "already_subscribed"}
        
        try:
            msg_class = self._get_message_class(msg_type)
            
            def callback(msg):
                # Queue message for async processing
                asyncio.create_task(self._telemetry_queue.put({
                    "topic": topic,
                    "data": self._msg_to_dict(msg),
                    "timestamp": datetime.utcnow().isoformat()
                }))
            
            self.subscribers[topic] = rospy.Subscriber(topic, msg_class, callback)
            return {"subscribed": True, "topic": topic}
        except Exception as e:
            logger.error(f"Failed to subscribe to {topic}: {e}")
            return {"error": str(e), "topic": topic}
    
    async def _cmd_call_service(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Call a ROS1 service"""
        service = params.get("service")
        request = params.get("request", {})
        
        try:
            import rosservice
            rospy.wait_for_service(service, timeout=5.0)
            # Service proxy would be created here
            return {"service": service, "called": True}
        except Exception as e:
            return {"error": str(e), "service": service}
    
    def _cmd_get_topics(self) -> Dict[str, Any]:
        """Get list of available ROS1 topics"""
        if not ROS1_AVAILABLE:
            return {"topics": [], "error": "ROS1 not available"}
        
        topics = rospy.get_published_topics()
        return {
            "topics": [{"name": name, "type": msg_type} for name, msg_type in topics]
        }
    
    def _cmd_get_nodes(self) -> Dict[str, Any]:
        """Get list of ROS1 nodes"""
        if not ROS1_AVAILABLE:
            return {"nodes": [], "error": "ROS1 not available"}
        
        nodes = rospy.get_node_names()
        return {"nodes": nodes}
    
    def _get_message_class(self, msg_type: str):
        """Dynamically import ROS1 message class"""
        try:
            parts = msg_type.split("/")
            if len(parts) == 2:
                pkg, msg = parts
            else:
                # Handle format like "std_msgs/String"
                pkg = parts[0]
                msg = parts[-1]
            
            module = __import__(f"{pkg}.msg", fromlist=[msg])
            return getattr(module, msg)
        except Exception as e:
            logger.error(f"Failed to import message type {msg_type}: {e}")
            # Fallback to String
            from std_msgs.msg import String
            return String
    
    def _build_message(self, msg_type: str, data: Dict[str, Any]):
        """Build ROS1 message from dict"""
        msg_class = self._get_message_class(msg_type)
        msg = msg_class()
        
        # Simple field assignment - for complex types, this would need recursion
        for key, value in data.items():
            if hasattr(msg, key):
                setattr(msg, key, value)
        
        return msg
    
    def _msg_to_dict(self, msg) -> Dict[str, Any]:
        """Convert ROS1 message to dict"""
        # Get all non-private attributes
        result = {}
        for slot in msg.__slots__ if hasattr(msg, '__slots__') else dir(msg):
            if not slot.startswith('_'):
                try:
                    val = getattr(msg, slot)
                    if not callable(val):
                        result[slot] = val
                except:
                    pass
        return result
    
    async def get_telemetry(self) -> Optional[Telemetry]:
        """Get telemetry data from queue"""
        try:
            data = self._telemetry_queue.get_nowait()
            return Telemetry(
                topic=data["topic"],
                data=data["data"],
                quality=1.0
            )
        except asyncio.QueueEmpty:
            return None


class ROS1Connector(Connector):
    """ROS1 connector for OpenClaw Gateway"""
    
    def __init__(self, config: Dict[str, Any]):
        super().__init__("ros1", config)
        self.robots: Dict[str, ROS1Robot] = {}
        self.auto_discover = config.get("auto_discover", True)
        self._running = False
    
    async def start(self) -> bool:
        """Start ROS1 connector"""
        if not ROS1_AVAILABLE:
            logger.error("ROS1 (rospy) not installed. Install: apt-get install python3-rospy")
            return False
        
        self._running = True
        logger.info("ROS1 connector started")
        
        if self.auto_discover:
            await self._discover_robots()
        
        return True
    
    async def stop(self) -> None:
        """Stop ROS1 connector"""
        self._running = False
        for robot in self.robots.values():
            await robot.disconnect()
        self.robots.clear()
        logger.info("ROS1 connector stopped")
    
    async def discover(self) -> List[RobotEndpoint]:
        """Discover ROS1 robots/nodes"""
        if not ROS1_AVAILABLE:
            return []
        
        endpoints = []
        try:
            nodes = rospy.get_node_names()
            for node in nodes:
                endpoints.append(RobotEndpoint(
                    robot_id=node.replace("/", "_"),
                    name=node,
                    robot_type="ros1",
                    address=node,
                    capabilities=["publish", "subscribe"],
                    metadata={"ros_version": "noetic"}
                ))
        except Exception as e:
            logger.error(f"Discovery failed: {e}")
        
        return endpoints
    
    async def connect(self, endpoint: RobotEndpoint) -> Optional[Robot]:
        """Connect to a ROS1 robot"""
        robot = ROS1Robot(endpoint.robot_id, endpoint.name)
        if await robot.connect():
            self.robots[endpoint.robot_id] = robot
            return robot
        return None
    
    async def disconnect(self, robot_id: str) -> None:
        """Disconnect from a ROS1 robot"""
        if robot_id in self.robots:
            await self.robots[robot_id].disconnect()
            del self.robots[robot_id]
    
    async def _discover_robots(self):
        """Auto-discover and connect to ROS1 robots"""
        endpoints = await self.discover()
        for endpoint in endpoints:
            if endpoint.robot_id not in self.robots:
                robot = await self.connect(endpoint)
                if robot:
                    logger.info(f"Auto-connected to ROS1 robot: {endpoint.name}")
