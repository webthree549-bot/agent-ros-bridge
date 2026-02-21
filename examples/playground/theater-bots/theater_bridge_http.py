#!/usr/bin/env python3
"""
Theater Bridge + HTTP Server Combined (Fixed)
Connects Agent ROS Bridge (WebSocket) to ROS2 and serves stage.html
"""
import os
import asyncio
import json
import threading

# Agent ROS Bridge
from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# HTTP Server
from aiohttp import web


class TheaterBridgeHTTPNode(Node):
    """ROS2 Node: Bridge + HTTP Server"""
    
    def __init__(self):
        super().__init__('theater_bridge_http')
        
        # Agent ROS Bridge with WebSocket
        self.bridge = Bridge()
        ws_transport = WebSocketTransport({"port": 8767})
        self.bridge.transport_manager.register(ws_transport)
        
        # ROS2 Publishers
        self.dialogue_pub = self.create_publisher(String, '/theater/dialogue', 10)
        self.cue_pub = self.create_publisher(String, '/theater/cue', 10)
        
        # ROS2 Subscriber
        self.audience_sub = self.create_subscription(String, '/theater/audience', self.on_audience, 10)
        
        self.dialogues = []
        self.get_logger().info('🎭 Theater Bridge + HTTP initialized')
        
    async def start_bridge(self):
        await self.bridge.start()
        self.get_logger().info('✓ Agent ROS Bridge WebSocket started')
        
    def on_audience(self, msg):
        input_data = json.loads(msg.data)
        self.get_logger().info(f'Audience: {input_data.get("prompt", "")}')
        
    def publish_dialogue(self, actor, line):
        msg = String()
        msg.data = json.dumps({"actor": actor, "line": line})
        self.dialogue_pub.publish(msg)
        self.dialogues.append({"actor": actor, "line": line})
        self.get_logger().info(f'🎭 {actor}: {line[:40]}...')


# HTTP Handlers
async def index(request):
    html_path = '/ros2_ws/stage.html'
    if os.path.exists(html_path):
        with open(html_path) as f:
            return web.Response(text=f.read(), content_type='text/html')
    return web.Response(text="stage.html not found", status=404)


async def api_dialogues(request):
    return web.json_response({"dialogues": []})


async def start_http_server():
    app = web.Application()
    app.router.add_get('/', index)
    app.router.add_get('/api/dialogues', api_dialogues)
    
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8080)
    await site.start()
    print('🌐 Theater HTTP: http://0.0.0.0:8080')


def ros2_spin_thread(executor):
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


def main(args=None):
    if not os.environ.get('JWT_SECRET'):
        print("❌ JWT_SECRET required!")
        return
        
    print("🎭 Theater Bridge + HTTP Server")
    print("=" * 60)
    
    rclpy.init(args=args)
    node = TheaterBridgeHTTPNode()
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    ros_thread = threading.Thread(target=ros2_spin_thread, args=(executor,), daemon=True)
    ros_thread.start()
    
    loop = asyncio.get_event_loop()
    loop.run_until_complete(node.start_bridge())
    loop.run_until_complete(start_http_server())
    
    print("\n📋 Services:")
    print("   WebSocket: ws://localhost:8767")
    print("   HTTP:      http://localhost:8080")
    print("   ROS2:      /theater/dialogue")
    print("=" * 60)
    
    import time
    lines = [
        ("Hamlet", "To code or not to code, that is the question..."),
        ("Juliet", "O Romeo, Romeo, wherefore art thy WiFi?"),
        ("Feste", "Nothing to be done. The buffer is empty.")
    ]
    
    for actor, line in lines:
        node.get_logger().info(f'\n🎭 {actor} speaks...')
        node.publish_dialogue(actor, line)
        time.sleep(3)
    
    print("\n✨ Theater active... HTTP server running")
    
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()