#!/usr/bin/env python3
import asyncio
import logging
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("actions-demo")

from agent_ros_bridge import ROSBridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
from agent_ros_bridge.gateway_v2.transports.grpc import GRPCServer
from agent_ros_bridge.gateway_v2.connectors.ros2 import ROS2Connector

STATIC_DIR = Path(__file__).parent

class DashboardHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            try:
                with open(STATIC_DIR / "index.html", "rb") as f:
                    content = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(content)
            except:
                self.send_error(404)
        else:
            self.send_error(404)

def http_server():
    logger.info("HTTP starting on port 8773")
    HTTPServer(("0.0.0.0", 8773), DashboardHandler).serve_forever()

async def main():
    print("⚡ Actions Demo")
    threading.Thread(target=http_server, daemon=True).start()
    await asyncio.sleep(1)
    
    bridge = ROSBridge(ros_version=2)
    bridge.transport_manager.register(WebSocketTransport({"host": "0.0.0.0", "port": 8765, "auth": {"enabled": False}}))
    bridge.transport_manager.register(GRPCServer({"host": "0.0.0.0", "port": 50051, "auth": {"enabled": False}}))
    bridge.connector_manager.register(ROS2Connector({"domain_id": 0}))
    
    @bridge.action("actions.navigate")
    async def nav(x, y, theta=0.0):
        return {"status": "success", "position": {"x": x, "y": y}}
    
    print("🌐 http://localhost:8773")
    print("📡 ws://localhost:8765")
    await bridge.start()

if __name__ == "__main__":
    asyncio.run(main())
