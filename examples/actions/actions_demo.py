#!/usr/bin/env python3
"""Actions Demo - ROS Actions with Web Dashboard"""

import asyncio
import logging
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("actions-demo")

try:
    from agent_ros_bridge import ROSBridge
    from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
    from agent_ros_bridge.gateway_v2.transports.grpc import GRPCServer
    from agent_ros_bridge.gateway_v2.connectors.ros2 import ROS2Connector
except ImportError:
    logger.error("agent_ros_bridge not installed")
    raise

STATIC_DIR = Path(__file__).parent

class DashboardHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            try:
                with open(STATIC_DIR / 'index.html', 'rb') as f:
                    content = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.send_header('Content-Length', len(content))
                self.end_headers()
                self.wfile.write(content)
            except FileNotFoundError:
                self.send_error(404)
        else:
            self.send_error(404)

def start_http(port=8773):
    logger.info(f"Starting HTTP on port {port}")
    server = HTTPServer(('0.0.0.0', port), DashboardHandler)
    logger.info(f"HTTP ready on port {port}")
    server.serve_forever()

async def main():
    print("=" * 60)
    print("⚡ Actions Demo")
    print("=" * 60)
    
    logger.info("Starting HTTP server...")
    threading.Thread(target=start_http, args=(8773,), daemon=True).start()
    await asyncio.sleep(1)
    
    bridge = ROSBridge(ros_version=2)
    
    ws = WebSocketTransport({'host': '0.0.0.0', 'port': 8765, 'auth': {'enabled': False}})
    bridge.transport_manager.register(ws)
    
    grpc = GRPCServer({'host': '0.0.0.0', 'port': 50051, 'auth': {'enabled': False}})
    bridge.transport_manager.register(grpc)
    
    ros2 = ROS2Connector({'domain_id': 0})
    bridge.connector_manager.register(ros2)
    
    @bridge.action("actions.navigate")
    async def navigate(x, y, theta=0.0):
        logger.info(f"Navigate to ({x}, {y})")
        await asyncio.sleep(1)
        return {"status": "success", "position": {"x": x, "y": y}}
    
    print("🌐 Dashboard: http://localhost:8773")
    print("📡 WebSocket: ws://localhost:8765")
    print("🔗 gRPC: localhost:50051")
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    try:
        await bridge.start()
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")
        await bridge.stop()

if __name__ == "__main__":
    asyncio.run(main())
