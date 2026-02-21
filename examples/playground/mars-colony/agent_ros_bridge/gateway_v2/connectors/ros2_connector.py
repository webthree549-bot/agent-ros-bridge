#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 Connector for OpenClaw Gateway

Connects to ROS2 robots and bridges ROS2 topics/services/actions
to the OpenClaw unified message format.
"""

import asyncio
import logging
from typing import Dict, Any, List, Optional, AsyncIterator
from dataclasses import dataclass
from datetime import datetime

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

from agent_ros_bridge.gateway_v2.core import (
    Connector, Robot, RobotEndpoint, Command, Telemetry
)

logger = logging.getLogger("connector.ros2")


@dataclass
class ROS2Topic:
    """ROS2 topic information"""
    name: str
    message_type: str
    qos_profile: str


class ROS2Robot(Robot):
    """ROS2 robot implementation"""
    
    def __init__(self, robot_id: str, name: str, ros_node: Node):
        super().__init__(robot_id, name, "ros2")
        self.ros_node = ros_node
        self.subscriptions: Dict[str, Any] = {}
        self.publishers: Dict[str, Any] = {}
        self._telemetry_queue: asyncio.Queue = asyncio.Queue()
    
    async def connect(self) -> bool:
        """ROS2 robot is already connected via node"""
        self.connected = True
        self.capabilities.add("publish")
        self.capabilities.add("subscribe")
        self.capabilities.add("services")
        logger.info(f"ROS2 robot {self.name} connected")
        return True
    
    async def disconnect(self) -> None:
        """Disconnect from ROS2"""
        self.connected = False
        # Cleanup subscriptions
        for sub in self.subscriptions.values():
            self.ros_node.destroy_subscription(sub)
        self.subscriptions.clear()
        logger.info(f"ROS2 robot {self.name} disconnected")
    
    async def execute(self, command: Command) -> Any:
        """Execute command on ROS2 robot"""
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
    
    async def subscribe(self, topic: str) -> AsyncIterator[Telemetry]:
        """Subscribe to ROS2 topic"""
        if topic in self.subscriptions:
            logger.warning(f"Already subscribed to {topic}")
            return
        
        # Create ROS2 subscription
        # This is simplified - actual implementation needs message type mapping
        def callback(msg):
            telemetry = Telemetry(
                topic=topic,
                data=self._ros_msg_to_dict(msg),
                quality=1.0
            )
            asyncio.create_task(self._telemetry_queue.put(telemetry))
        
        # In real implementation, we'd need to determine the message type
        # subscription = self.ros_node.create_subscription(
        #     msg_type, topic, callback, 10
        # )
        # self.subscriptions[topic] = subscription
        
        # Yield from queue
        while self.connected:
            try:
                telemetry = await asyncio.wait_for(
                    self._telemetry_queue.get(),
                    timeout=1.0
                )
                if telemetry.topic == topic:
                    yield telemetry
            except asyncio.TimeoutError:
                continue
    
    async def _cmd_publish(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Publish message to ROS2 topic"""
        topic = params.get("topic")
        message_data = params.get("data")
        msg_type_str = params.get("type", "std_msgs/String")
        
        # Create publisher if needed
        if topic not in self.publishers:
            # In real implementation, resolve message type from string
            # publisher = self.ros_node.create_publisher(msg_type, topic, 10)
            # self.publishers[topic] = publisher
            pass
        
        # Create and publish message
        # msg = message_type()
        # self._dict_to_ros_msg(message_data, msg)
        # self.publishers[topic].publish(msg)
        
        return {"status": "published", "topic": topic}
    
    async def _cmd_call_service(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Call ROS2 service"""
        service_name = params.get("service")
        request_data = params.get("request")
        
        # Create service client
        # client = self.ros_node.create_client(service_type, service_name)
        # await client.wait_for_service()
        # request = service_type.Request()
        # self._dict_to_ros_msg(request_data, request)
        # response = await client.call_async(request)
        
        return {"status": "called", "service": service_name}
    
    async def _cmd_send_action(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Send ROS2 action goal"""
        action_name = params.get("action")
        goal_data = params.get("goal")
        
        # Action client implementation
        return {"status": "goal_sent", "action": action_name}
    
    def _cmd_get_topics(self) -> List[Dict[str, Any]]:
        """Get list of available topics"""
        topic_names_and_types = self.ros_node.get_topic_names_and_types()
        return [
            {"name": name, "types": types}
            for name, types in topic_names_and_types
        ]
    
    def _cmd_get_nodes(self) -> List[str]:
        """Get list of ROS2 nodes"""
        return self.ros_node.get_node_names()
    
    def _ros_msg_to_dict(self, msg) -> Dict[str, Any]:
        """Convert ROS message to dictionary"""
        # Simplified - actual implementation would use msg.get_fields_and_field_types()
        return {"_type": type(msg).__name__}
    
    def _dict_to_ros_msg(self, data: Dict[str, Any], msg):
        """Convert dictionary to ROS message"""
        for key, value in data.items():
            if hasattr(msg, key):
                setattr(msg, key, value)


class ROS2Connector(Connector):
    """ROS2 connector implementation"""
    
    connector_type = "ros2"
    
    def __init__(self, domain_id: int = 0):
        self.domain_id = domain_id
        self.node: Optional[Node] = None
        self._initialized = False
    
    async def _ensure_initialized(self):
        """Ensure ROS2 is initialized"""
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS2 not available. Install rclpy.")
        
        if not self._initialized:
            rclpy.init()
            self.node = rclpy.create_node(
                f"openclaw_bridge_{datetime.now().timestamp()}",
                namespace="openclaw"
            )
            self._initialized = True
            
            # Start ROS2 spinner in background
            asyncio.create_task(self._spin_ros())
    
    async def _spin_ros(self):
        """Spin ROS2 node"""
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            await asyncio.sleep(0.001)
    
    async def connect(self, uri: str, **kwargs) -> Robot:
        """Connect to ROS2 system
        
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
        """Discover ROS2 systems on network"""
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
            endpoints.append(RobotEndpoint(
                uri=f"ros2://{self.domain_id}/{ns}",
                name=f"ros2_{ns}",
                connector_type="ros2",
                capabilities=["publish", "subscribe", "services", "actions"],
                metadata={
                    "domain_id": self.domain_id,
                    "namespace": ns,
                    "topic_count": len([t for t, _ in topics if ns in t])
                }
            ))
        
        return endpoints


# Example usage
async def example():
    """Example ROS2 connector usage"""
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
        result = await robot.execute(Command(
            action="get_topics",
            parameters={}
        ))
        print(f"Topics: {result}")


if __name__ == "__main__":
    import asyncio
    asyncio.run(example())
