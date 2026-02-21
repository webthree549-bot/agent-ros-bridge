#!/usr/bin/env python3
"""
Art Brain ROS2 Bridge Integration
Connects Agent ROS Bridge to real ROS2 nodes
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


class ArtBridgeNode(Node):
    """
    ROS2 Node that bridges Agent ROS Bridge to ROS2 topics
    """
    
    def __init__(self):
        super().__init__('art_bridge')
        
        # Initialize Agent ROS Bridge
        self.bridge = Bridge()
        ws_transport = WebSocketTransport({"port": 8765})
        self.bridge.transport_manager.register(ws_transport)
        
        # ROS2 Publishers
        self.emotion_pub = self.create_publisher(String, '/art/emotion', 10)
        
        # ROS2 Subscribers
        self.stroke_sub = self.create_subscription(
            String, '/art/strokes', self.on_ros_stroke, 10
        )
        self.canvas_sub = self.create_subscription(
            String, '/art/canvas', self.on_ros_canvas, 10
        )
        
        self.get_logger().info('🎨 Art Bridge initialized')
        
    async def start_bridge(self):
        """Start Agent ROS Bridge"""
        await self.bridge.start()
        self.get_logger().info('✓ Agent ROS Bridge started on port 8765')
        
    def on_ros_stroke(self, msg):
        """Forward ROS2 stroke to Agent ROS Bridge"""
        stroke = json.loads(msg.data)
        self.get_logger().info(f'Bridge: Forwarding stroke from {stroke["robot_id"]}')
        # Would broadcast via bridge here
        
    def on_ros_canvas(self, msg):
        """Forward ROS2 canvas to Agent ROS Bridge"""
        canvas = json.loads(msg.data)
        self.get_logger().info(f'Bridge: Canvas has {canvas["generation"]} strokes')
        
    def publish_emotion(self, emotion):
        """Publish emotion to ROS2 topic"""
        msg = String()
        msg.data = emotion
        self.emotion_pub.publish(msg)
        self.get_logger().info(f'Bridge: Published emotion {emotion} to ROS2')


def main(args=None):
    # Check JWT
    if not os.environ.get('JWT_SECRET'):
        print("❌ JWT_SECRET required!")
        print("export JWT_SECRET=$(openssl rand -base64 32)")
        return
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create bridge node
    node = ArtBridgeNode()
    
    # Start bridge in executor
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    # Start bridge async
    loop = asyncio.get_event_loop()
    loop.run_until_complete(node.start_bridge())
    
    # Publish some emotions
    emotions = ["joy", "sadness", "wonder", "chaos"]
    for i, emotion in enumerate(emotions):
        node.get_logger().info(f'\n🎭 Generation {i+1}: {emotion}')
        node.publish_emotion(emotion)
        
        # Spin ROS2 for 2 seconds
        for _ in range(20):
            executor.spin_once(timeout_sec=0.1)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()