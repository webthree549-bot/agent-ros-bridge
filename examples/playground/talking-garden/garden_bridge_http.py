#!/usr/bin/env python3
"""
Garden Bridge + HTTP Server Combined (Fixed)
Connects Agent ROS Bridge (MQTT) to ROS2 and serves garden.html
"""
import os
import asyncio
import json
import threading

# Agent ROS Bridge
from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.mqtt import MQTTTransport

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64

# HTTP Server
from aiohttp import web


class GardenBridgeHTTPNode(Node):
    """ROS2 Node: Bridge + HTTP Server"""
    
    def __init__(self):
        super().__init__('garden_bridge_http')
        
        # Agent ROS Bridge with MQTT
        self.bridge = Bridge()
        mqtt_transport = MQTTTransport({
            "broker": "localhost",
            "port": 1883,
            "client_id": "garden_bridge_ros2"
        })
        self.bridge.transport_manager.register(mqtt_transport)
        
        # ROS2 Publishers
        self.moisture_pub = self.create_publisher(Float64, '/garden/moisture', 10)
        self.light_pub = self.create_publisher(Float64, '/garden/light', 10)
        self.poetry_pub = self.create_publisher(String, '/garden/poetry', 10)
        
        # ROS2 Subscriber
        self.care_sub = self.create_subscription(String, '/garden/care', self.on_care, 10)
        
        self.poems = []
        self.get_logger().info('🌱 Garden Bridge + HTTP initialized')
        
    async def start_bridge(self):
        await self.bridge.start()
        self.get_logger().info('✓ Agent ROS Bridge MQTT started')
        
    def on_care(self, msg):
        care = json.loads(msg.data)
        self.get_logger().info(f'Care for {care["plant"]}')
        
    def publish_poetry(self, plant, poem):
        msg = String()
        msg.data = json.dumps({"plant": plant, "poem": poem})
        self.poetry_pub.publish(msg)
        self.poems.append({"plant": plant, "poem": poem})
        self.get_logger().info(f'📜 Poetry from {plant}')


# HTTP Handlers
async def index(request):
    html_path = '/ros2_ws/garden.html'
    if os.path.exists(html_path):
        with open(html_path) as f:
            return web.Response(text=f.read(), content_type='text/html')
    return web.Response(text="garden.html not found", status=404)


async def api_poems(request):
    return web.json_response({"poems": []})


async def start_http_server():
    app = web.Application()
    app.router.add_get('/', index)
    app.router.add_get('/api/poems', api_poems)
    
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8080)
    await site.start()
    print('🌐 Garden HTTP: http://0.0.0.0:8080')


def ros2_spin_thread(executor):
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


def main(args=None):
    if not os.environ.get('JWT_SECRET'):
        print("❌ JWT_SECRET required!")
        return
        
    print("🚀 Garden Bridge + HTTP Server")
    print("=" * 60)
    
    rclpy.init(args=args)
    node = GardenBridgeHTTPNode()
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    # Start ROS2 in background thread
    ros_thread = threading.Thread(target=ros2_spin_thread, args=(executor,), daemon=True)
    ros_thread.start()
    
    loop = asyncio.get_event_loop()
    loop.run_until_complete(node.start_bridge())
    loop.run_until_complete(start_http_server())
    
    print("\n📋 Services:")
    print("   MQTT:   mqtt://localhost:1883")
    print("   HTTP:   http://localhost:8080")
    print("   ROS2:   /garden/poetry, /garden/moisture")
    print("=" * 60)
    
    # Demo poems
    poems = [
        ("Fernando", "My fronds thirst for silver dew..."),
        ("Cactilda", "I am fortress and flower..."),
        ("Rosalind", "Love me, or let me wilt beautifully...")
    ]
    
    import time
    for plant, poem in poems:
        node.get_logger().info(f'\n🌱 {plant} speaks...')
        node.publish_poetry(plant, poem)
        time.sleep(3)
    
    print("\n✨ Garden listening... HTTP server running")
    
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()