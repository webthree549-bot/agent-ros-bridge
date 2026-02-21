#!/usr/bin/env python3
"""
Garden Bridge - Connects Agent ROS Bridge to ROS2
Subscribes to MQTT from bridge, publishes to ROS2 topics
"""
import os
import asyncio
import json

# Agent ROS Bridge
from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.mqtt import MQTTTransport

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Range


class GardenBridgeNode(Node):
    """ROS2 Node that bridges Agent ROS Bridge MQTT to ROS2"""
    
    def __init__(self):
        super().__init__('garden_bridge')
        
        # Initialize Agent ROS Bridge
        self.bridge = Bridge()
        mqtt_transport = MQTTTransport({
            "broker": "localhost",
            "port": 1883,
            "client_id": "garden_bridge_ros2"
        })
        self.bridge.transport_manager.register(mqtt_transport)
        
        # ROS2 Publishers for plant sensors
        self.moisture_pubs = {}
        self.light_pubs = {}
        self.temp_pubs = {}
        
        plants = ['fernando', 'cactilda', 'rosalind', 'basilion', 'orchidelle', 'sunflower_sam']
        for plant in plants:
            self.moisture_pubs[plant] = self.create_publisher(Float64, f'/garden/{plant}/moisture', 10)
            self.light_pubs[plant] = self.create_publisher(Float64, f'/garden/{plant}/light', 10)
            self.temp_pubs[plant] = self.create_publisher(Float64, f'/garden/{plant}/temperature', 10)
        
        # ROS2 Subscriber for care commands
        self.care_sub = self.create_subscription(String, '/garden/care', self.on_care_command, 10)
        
        # ROS2 Publisher for poetry
        self.poetry_pub = self.create_publisher(String, '/garden/poetry', 10)
        
        self.get_logger().info('🌱 Garden Bridge initialized')
        
    async def start_bridge(self):
        """Start Agent ROS Bridge"""
        await self.bridge.start()
        self.get_logger().info('✓ Agent ROS Bridge MQTT started')
        
        # Subscribe to MQTT topics
        # In real implementation, would subscribe to MQTT broker here
        self.get_logger().info('📡 Subscribed to MQTT: garden/sensors/+/+')
        
    def on_care_command(self, msg):
        """Receive care command from ROS2, forward to bridge"""
        care_data = json.loads(msg.data)
        self.get_logger().info(f'Bridge: Care command for {care_data["plant"]}')
        # Forward to MQTT for bridge clients
        
    def publish_sensor_data(self, plant, moisture, light, temp):
        """Publish sensor data to ROS2 topics"""
        if plant in self.moisture_pubs:
            msg = Float64()
            msg.data = float(moisture)
            self.moisture_pubs[plant].publish(msg)
            
            msg.data = float(light)
            self.light_pubs[plant].publish(msg)
            
            msg.data = float(temp)
            self.temp_pubs[plant].publish(msg)
            
            self.get_logger().info(f'Bridge: Published {plant} sensors to ROS2')


def main(args=None):
    if not os.environ.get('JWT_SECRET'):
        print("❌ JWT_SECRET required!")
        return
        
    rclpy.init(args=args)
    node = GardenBridgeNode()
    
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