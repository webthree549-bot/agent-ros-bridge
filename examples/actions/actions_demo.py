#!/usr/bin/env python3
"""Actions Demo - ROS Actions with Web Dashboard

Demonstrates navigation and manipulation actions with a web interface.
"""

import asyncio
import json
import logging
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("actions-demo")

# Import bridge components
try:
    from agent_ros_bridge import ROSBridge
    from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
    from agent_ros_bridge.gateway_v2.transports.grpc import GRPCServer
    from agent_ros_bridge.gateway_v2.connectors.ros2 import ROS2Connector
except ImportError:
    logger.error("agent_ros_bridge not installed. Run: pip install agent_ros-bridge")
    raise

# HTTP Server for dashboard
STATIC_DIR = Path(__file__).parent

class DashboardHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        logger.info(f"{self.client_address[0]} - {format % args}")
    
    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            self.serve_file('index.html', 'text/html')
        elif self.path.endswith('.js'):
            self.serve_file(self.path[1:], 'application/javascript')
        elif self.path.endswith('.css'):
            self.serve_file(self.path[1:], 'text/css')
        else:
            self.send_error(404)
    
    def serve_file(self, filename, content_type):
        try:
            file_path = STATIC_DIR / filename
            with open(file_path, 'rb') as f:
                content = f.read()
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        except FileNotFoundError:
            self.send_error(404, f"File not found: {filename}")
        except Exception as e:
            logger.error(f"Error serving file: {e}")
            self.send_error(500)

def start_http_server(port=8773):
    """Start HTTP server for dashboard"""
    try:
        logger.info(f"Starting HTTP server on port {port}...")
        server = HTTPServer(('0.0.0.0', port), DashboardHandler)
        logger.info(f"HTTP server created, starting serve_forever on port {port}")
        server.serve_forever()
    except Exception as e:
        logger.error(f"Failed to start HTTP server: {e}")
        import traceback
        traceback.print_exc()
        raise

async def main():
    """Main entry point"""
    print("=" * 60)
    print("⚡ Actions Demo")
    print("=" * 60)
    print()
    
    # Start HTTP server FIRST (before bridge)
    logger.info("Starting HTTP dashboard server...")
    http_thread = threading.Thread(target=start_http_server, args=(8773,), daemon=True)
    http_thread.start()
    
    # Give HTTP server time to start
    await asyncio.sleep(1)
    
    # Create bridge
    bridge = ROSBridge(ros_version=2)
    
    # Register WebSocket transport
    ws_transport = WebSocketTransport({
        'host': '0.0.0.0',
        'port': 8765,
        'auth': {'enabled': False}  # Demo mode
    })
    bridge.transport_manager.register(ws_transport)
    
    # Register gRPC transport
    grpc_server = GRPCServer({
        'host': '0.0.0.0',
        'port': 50051,
        'auth': {'enabled': False}
    })
    bridge.transport_manager.register(grpc_server)
    
    # Register ROS2 connector
    ros2_connector = ROS2Connector({'domain_id': 0})
    bridge.connector_manager.register(ros2_connector)
    
    # Define actions
    @bridge.action("actions.navigate")
    async def navigate_to(x: float, y: float, theta: float = 0.0):
        """Navigate robot to position"""
        logger.info(f"Navigating to ({x}, {y}, {theta})")
        # Simulate action
        await asyncio.sleep(1)
        return {
            "status": "success",
            "position": {"x": x, "y": y, "theta": theta},
            "message": f"Navigation to ({x}, {y}) completed"
        }
    
    @bridge.action("actions.manipulate")
    async def manipulate(joint_positions: list):
        """Execute joint trajectory"""
        logger.info(f"Executing trajectory with {len(joint_positions)} waypoints")
        await asyncio.sleep(1)
        return {
            "status": "success",
            "waypoints_completed": len(joint_positions)
        }
    
    @bridge.action("actions.cancel")
    async def cancel_action():
        """Cancel current action"""
        logger.info("Cancelling current action")
        return {"status": "cancelled"}
    
    @bridge.action("actions.status")
    async def get_status():
        """Get action status"""
        return {
            "status": "idle",
            "current_action": None,
            "progress": 0.0
        }
    
    # Start bridge
    print("🌐 Dashboard: http://localhost:8773")
    print("📡 WebSocket: ws://localhost:8765")
    print("🔗 gRPC: localhost:50051")
    print()
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    try:
        await bridge.start()
    except KeyboardInterrupt:
        print("\n\n👋 Shutting down...")
        await bridge.stop()

if __name__ == "__main__":
    asyncio.run(main())
