#!/usr/bin/env python3
"""
Theater Bridge - Connects Agent ROS Bridge to ROS2
WebSocket for audience interaction
"""
import os
import asyncio
import json

# Agent ROS Bridge
from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TheaterBridgeNode(Node):
    """ROS2 Node that bridges Agent ROS Bridge WebSocket to ROS2"""
    
    def __init__(self):
        super().__init__('theater_bridge')
        
        # Initialize Agent ROS Bridge with WebSocket
        self.bridge = Bridge()
        ws_transport = WebSocketTransport({"port": 8767})
        self.bridge.transport_manager.register(ws_transport)
        
        # ROS2 Publishers for performance
        self.dialogue_pub = self.create_publisher(String, '/theater/dialogue', 10)
        self.cue_pub = self.create_publisher(String, '/theater/stage/cue', 10)
        
        # ROS2 Subscriber for audience input
        self.audience_sub = self.create_subscription(String, '/theater/audience/input', self.on_audience, 10)
        
        self.get_logger().info('🎭 Theater Bridge initialized')
        
    async def start_bridge(self):
        """Start Agent ROS Bridge"""
        await self.bridge.start()
        self.get_logger().info('✓ Agent ROS Bridge WebSocket started on port 8767')
        self.get_logger().info('👥 Waiting for audience WebSocket connections...')
        
    def publish_dialogue(self, actor, line):
        """Publish actor dialogue to ROS2"""
        dialogue = {
            "actor": actor,
            "line": line,
            "timestamp": self.get_clock().now().to_msg()
        }
        msg = String()
        msg.data = json.dumps(dialogue)
        self.dialogue_pub.publish(msg)
        self.get_logger().info(f'Bridge: {actor} spoke via ROS2')
        
    def on_audience(self, msg):
        """Receive audience input from ROS2, forward to bridge"""
        input_data = json.loads(msg.data)
        self.get_logger().info(f'Bridge: Audience input - {input_data["prompt"]}')
        # Forward to WebSocket clients


def main(args=None):
    if not os.environ.get('JWT_SECRET'):
        print("❌ JWT_SECRET required!")
        return
        
    rclpy.init(args=args)
    node = TheaterBridgeNode()
    
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