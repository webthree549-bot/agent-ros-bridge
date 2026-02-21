#!/usr/bin/env python3
"""
Garden Bridge ROS2 + HTTP Server
Connects Agent ROS Bridge to ROS2 and serves garden.html
"""
import os
import asyncio
import json
from aiohttp import web

# Agent ROS Bridge
from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64


class GardenBridgeNode(Node):
    """ROS2 Node that bridges Agent ROS Bridge to Garden topics"""
    
    def __init__(self):
        super().__init__('garden_bridge')
        
        # Initialize Agent ROS Bridge
        self.bridge = Bridge()
        ws_transport = WebSocketTransport({"port": 8765})
        self.bridge.transport_manager.register(ws_transport)
        
        # Store latest data for HTTP API
        self.plants = {}
        self.latest_poetry = None
        self.care_history = []
        
        # ROS2 Publishers
        self.care_pub = self.create_publisher(String, '/garden/care_commands', 10)
        
        # ROS2 Subscribers
        self.sensor_sub = self.create_subscription(
            String, '/garden/sensors', self.on_sensor_data, 10
        )
        self.poetry_sub = self.create_subscription(
            String, '/garden/poetry', self.on_poetry, 10
        )
        
        self.get_logger().info('🌱 Garden Bridge initialized')
        
    async def start_bridge(self):
        """Start Agent ROS Bridge"""
        await self.bridge.start()
        self.get_logger().info('✓ Agent ROS Bridge WebSocket on port 8765')
        
    def on_sensor_data(self, msg):
        """Receive sensor data from plants"""
        try:
            data = json.loads(msg.data)
            plant_id = data['plant_id']
            self.plants[plant_id] = data
            self.get_logger().debug(f"Sensor data from {plant_id}")
        except json.JSONDecodeError:
            pass
            
    def on_poetry(self, msg):
        """Receive poetry from oracle"""
        try:
            self.latest_poetry = json.loads(msg.data)
            self.get_logger().info(f"📜 New poetry: {self.latest_poetry['verses'][0][:40]}...")
        except json.JSONDecodeError:
            pass
            
    def send_care_command(self, plant, action):
        """Send care command via ROS2"""
        cmd = {
            'gardener': 'web_user',
            'plant': plant,
            'action': action,
            'reason': 'user request via web interface'
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.care_pub.publish(msg)
        self.care_history.append(cmd)
        self.get_logger().info(f"💻 Web command: {action} for {plant}")


# HTTP Server handlers
async def index_handler(request):
    """Serve garden.html"""
    html_path = '/ros2_ws/garden.html'
    if os.path.exists(html_path):
        with open(html_path) as f:
            return web.Response(text=f.read(), content_type='text/html')
    return web.Response(text="garden.html not found", status=404)


async def api_plants(request):
    """API endpoint for plant data"""
    node = request.app['garden_node']
    return web.json_response({
        'plants': node.plants,
        'count': len(node.plants)
    })


async def api_poetry(request):
    """API endpoint for latest poetry"""
    node = request.app['garden_node']
    return web.json_response({
        'poetry': node.latest_poetry,
        'history_count': len(node.care_history)
    })


async def api_care(request):
    """API endpoint to send care commands"""
    node = request.app['garden_node']
    try:
        data = await request.json()
        plant = data.get('plant')
        action = data.get('action')
        if plant and action:
            node.send_care_command(plant, action)
            return web.json_response({'status': 'ok', 'plant': plant, 'action': action})
        return web.json_response({'status': 'error', 'message': 'Missing plant or action'}, status=400)
    except Exception as e:
        return web.json_response({'status': 'error', 'message': str(e)}, status=500)


async def start_http_server(garden_node):
    """Start HTTP server for web interface"""
    app = web.Application()
    app['garden_node'] = garden_node
    
    app.router.add_get('/', index_handler)
    app.router.add_get('/api/plants', api_plants)
    app.router.add_get('/api/poetry', api_poetry)
    app.router.add_post('/api/care', api_care)
    
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8080)
    await site.start()
    print('🌐 HTTP Server started on http://localhost:8080')


def main(args=None):
    if not os.environ.get('JWT_SECRET'):
        print("❌ JWT_SECRET required!")
        print("export JWT_SECRET=$(openssl rand -base64 32)")
        return
        
    print("🚀 Starting Garden with Agent ROS Bridge + ROS2 + HTTP Server")
    print("=" * 60)
    
    # Initialize ROS2
    rclpy.init(args=args)
    node = GardenBridgeNode()
    
    # Create executor
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    # Get event loop
    loop = asyncio.get_event_loop()
    
    # Start services
    loop.run_until_complete(node.start_bridge())
    loop.run_until_complete(start_http_server(node))
    
    print("\n📋 Services:")
    print("   WebSocket: ws://localhost:8765 (Agent ROS Bridge)")
    print("   HTTP:      http://localhost:8080 (Garden interface)")
    print("   ROS2:      /garden/sensors, /garden/poetry, /garden/care_commands")
    print("\n🌱 Garden is growing...")
    print("=" * 60)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\n\n👋 Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
