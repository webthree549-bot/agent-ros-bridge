#!/usr/bin/env python3
"""
Mars Bridge - Connects Agent ROS Bridge to ROS2
Uses gRPC transport for mission-critical communication
"""
import os
import asyncio
import json

# Agent ROS Bridge
from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.grpc import gRPCTransport

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32


class MarsBridgeNode(Node):
    """ROS2 Node that bridges Agent ROS Bridge gRPC to ROS2"""
    
    def __init__(self):
        super().__init__('mars_bridge')
        
        # Initialize Agent ROS Bridge with gRPC
        self.bridge = Bridge()
        grpc_transport = gRPCTransport({"port": 50051})
        self.bridge.transport_manager.register(grpc_transport)
        
        # ROS2 Publishers for resources
        self.ice_pub = self.create_publisher(Int32, '/mars/resources/ice', 10)
        self.power_pub = self.create_publisher(Int32, '/mars/resources/power', 10)
        self.minerals_pub = self.create_publisher(Int32, '/mars/resources/minerals', 10)
        self.oxygen_pub = self.create_publisher(Int32, '/mars/resources/oxygen', 10)
        
        # ROS2 Publishers for missions
        self.mission_pub = self.create_publisher(String, '/mars/missions/allocate', 10)
        self.status_pub = self.create_publisher(String, '/mars/robot/status', 10)
        
        # ROS2 Subscriber for mission results
        self.result_sub = self.create_subscription(String, '/mars/missions/result', self.on_result, 10)
        
        self.get_logger().info('🚀 Mars Bridge initialized')
        
    async def start_bridge(self):
        """Start Agent ROS Bridge"""
        await self.bridge.start()
        self.get_logger().info('✓ Agent ROS Bridge gRPC started on port 50051')
        
    def publish_resources(self, ice, power, minerals, oxygen):
        """Publish resource updates to ROS2"""
        self.ice_pub.publish(Int32(data=int(ice)))
        self.power_pub.publish(Int32(data=int(power)))
        self.minerals_pub.publish(Int32(data=int(minerals)))
        self.oxygen_pub.publish(Int32(data=int(oxygen)))
        self.get_logger().info(f'Bridge: Resources updated')
        
    def allocate_mission(self, robot_id, task_type):
        """Allocate mission via bridge"""
        mission = {
            "robot_id": robot_id,
            "task": task_type,
            "timestamp": self.get_clock().now().to_msg()
        }
        msg = String()
        msg.data = json.dumps(mission)
        self.mission_pub.publish(msg)
        self.get_logger().info(f'Bridge: Mission allocated to {robot_id}')
        
    def on_result(self, msg):
        """Receive mission result from ROS2"""
        result = json.loads(msg.data)
        self.get_logger().info(f'Bridge: Mission completed by {result["robot_id"]}')


def main(args=None):
    if not os.environ.get('JWT_SECRET'):
        print("❌ JWT_SECRET required!")
        return
        
    rclpy.init(args=args)
    node = MarsBridgeNode()
    
    loop = asyncio.get_event_loop()
    loop.run_until_complete(node.start_bridge())
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()