#!/usr/bin/env python3
"""
Mars Bridge + HTTP Server Combined (Fixed)
Connects Agent ROS Bridge (gRPC) to ROS2 and serves dashboard.html
"""
import os
import asyncio
import json
import threading

# Agent ROS Bridge
from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.grpc import gRPCTransport

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

# HTTP Server
from aiohttp import web


class MarsBridgeHTTPNode(Node):
    """ROS2 Node: Bridge + HTTP Server"""
    
    def __init__(self):
        super().__init__('mars_bridge_http')
        
        # Agent ROS Bridge with gRPC
        self.bridge = Bridge()
        grpc_transport = gRPCTransport({"port": 50051})
        self.bridge.transport_manager.register(grpc_transport)
        
        # ROS2 Publishers
        self.ice_pub = self.create_publisher(Int32, '/mars/ice', 10)
        self.power_pub = self.create_publisher(Int32, '/mars/power', 10)
        self.mission_pub = self.create_publisher(String, '/mars/mission', 10)
        
        self.resources = {"ice": 100, "power": 100, "minerals": 50}
        self.sol = 1
        self.get_logger().info('🚀 Mars Bridge + HTTP initialized')
        
    async def start_bridge(self):
        await self.bridge.start()
        self.get_logger().info('✓ Agent ROS Bridge gRPC started')
        
    def update_resources(self, ice, power, minerals):
        self.ice_pub.publish(Int32(data=ice))
        self.power_pub.publish(Int32(data=power))
        self.resources = {"ice": ice, "power": power, "minerals": minerals}
        self.get_logger().info(f'Resources: ice={ice}, power={power}')


# HTTP Handlers
async def index(request):
    html_path = '/ros2_ws/dashboard.html'
    if os.path.exists(html_path):
        with open(html_path) as f:
            return web.Response(text=f.read(), content_type='text/html')
    return web.Response(text="dashboard.html not found", status=404)


async def api_resources(request):
    return web.json_response({"resources": {}})


async def start_http_server():
    app = web.Application()
    app.router.add_get('/', index)
    app.router.add_get('/api/resources', api_resources)
    
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8080)
    await site.start()
    print('🌐 Mars HTTP: http://0.0.0.0:8080')


def ros2_spin_thread(executor):
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


def main(args=None):
    if not os.environ.get('JWT_SECRET'):
        print("❌ JWT_SECRET required!")
        return
        
    print("🚀 Mars Bridge + HTTP Server")
    print("=" * 60)
    
    rclpy.init(args=args)
    node = MarsBridgeHTTPNode()
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    ros_thread = threading.Thread(target=ros2_spin_thread, args=(executor,), daemon=True)
    ros_thread.start()
    
    loop = asyncio.get_event_loop()
    loop.run_until_complete(node.start_bridge())
    loop.run_until_complete(start_http_server())
    
    print("\n📋 Services:")
    print("   gRPC:   localhost:50051")
    print("   HTTP:   http://localhost:8080")
    print("   ROS2:   /mars/ice, /mars/power")
    print("=" * 60)
    
    import time
    for sol in range(1, 6):
        node.sol = sol
        node.get_logger().info(f'\n🗓️  Sol {sol}')
        node.update_resources(100 + sol * 10, 100 - sol * 5, 50 + sol * 3)
        time.sleep(3)
    
    print("\n✨ Mars colony active... HTTP server running")
    
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()