#!/usr/bin/env python3
"""
Art Brain ROS2 Bridge + HTTP Server (Fixed)
Connects Agent ROS Bridge to ROS2 and serves canvas.html
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


class ArtBridgeNode(Node):
    """ROS2 Node that bridges Agent ROS Bridge to ROS2 topics"""
    
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
        
        self.strokes_received = []
        self.get_logger().info('🎨 Art Bridge initialized')
        
    async def start_bridge(self):
        """Start Agent ROS Bridge"""
        await self.bridge.start()
        self.get_logger().info('✓ Agent ROS Bridge WebSocket on port 8765')
        
    def on_ros_stroke(self, msg):
        """Receive stroke from ROS2 painters"""
        stroke = json.loads(msg.data)
        self.strokes_received.append(stroke)
        self.get_logger().info(f'🖌️  {stroke["robot_id"]}: {stroke["description"][:50]}...')
        
    def on_ros_canvas(self, msg):
        """Receive canvas state"""
        canvas = json.loads(msg.data)
        self.get_logger().info(f'🎨 Canvas: {canvas["generation"]} strokes from {canvas["robot_count"]} robots')
        
    def publish_emotion(self, emotion):
        """Publish emotion command to ROS2"""
        msg = String()
        msg.data = emotion
        self.emotion_pub.publish(msg)
        self.get_logger().info(f'📢 Published emotion: {emotion}')


# HTTP Handlers
async def index(request):
    """Serve canvas.html"""
    html_path = '/ros2_ws/canvas.html'
    if os.path.exists(html_path):
        with open(html_path) as f:
            return web.Response(text=f.read(), content_type='text/html')
    return web.Response(text="canvas.html not found", status=404)


async def api_strokes(request):
    """API endpoint for strokes data"""
    return web.json_response({"strokes": []})


async def start_http_server():
    """Start HTTP server for visual interface"""
    app = web.Application()
    app.router.add_get('/', index)
    app.router.add_get('/api/strokes', api_strokes)
    
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8080)
    await site.start()
    print('🌐 HTTP Server started on http://0.0.0.0:8080')
    print('   Visual interface: http://localhost:8080')


def ros2_spin_thread(executor):
    """Run ROS2 spinner in a separate thread"""
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


def main(args=None):
    # Check JWT
    if not os.environ.get('JWT_SECRET'):
        print("❌ JWT_SECRET required!")
        print("export JWT_SECRET=$(openssl rand -base64 32)")
        return
    
    print("🚀 Starting Art Studio with Agent ROS Bridge + ROS2 + HTTP Server")
    print("=" * 60)
    
    # Initialize ROS2
    rclpy.init(args=args)
    node = ArtBridgeNode()
    
    # Create executor
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    # Start ROS2 spinner in a separate thread
    ros_thread = threading.Thread(target=ros2_spin_thread, args=(executor,), daemon=True)
    ros_thread.start()
    
    # Get event loop for bridge and HTTP
    loop = asyncio.get_event_loop()
    
    # Start services
    loop.run_until_complete(node.start_bridge())
    loop.run_until_complete(start_http_server())
    
    print("\n📋 Services:")
    print("   WebSocket: ws://localhost:8765 (Agent ROS Bridge)")
    print("   HTTP:      http://localhost:8080 (Visual interface)")
    print("   ROS2:      /art/emotion, /art/strokes, /art/canvas")
    print("\n🎭 Starting emotion generations...")
    print("=" * 60)
    
    # Publish emotions
    emotions = ["joy", "sadness", "anger", "calm", "wonder", "chaos"]
    for i, emotion in enumerate(emotions):
        node.get_logger().info(f'\n🎭 Generation {i+1}: {emotion.upper()}')
        node.publish_emotion(emotion)
        # Just sleep, ROS2 is spinning in background thread
        import time
        time.sleep(3)
    
    print("\n" + "=" * 60)
    print("✨ Demo complete! HTTP server still running...")
    print("   Open http://localhost:8080 to see visual interface")
    print("   Press Ctrl+C to stop")
    print("=" * 60)
    
    try:
        # Keep HTTP server running
        loop.run_forever()
    except KeyboardInterrupt:
        print("\n\n👋 Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()