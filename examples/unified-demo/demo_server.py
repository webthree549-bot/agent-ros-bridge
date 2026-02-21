#!/usr/bin/env python3
"""Unified Demo Server for Agent ROS Bridge

Web-based selector for all examples with integrated ROS2 environment.
"""

import asyncio
import json
import logging
import os
import subprocess
import sys
from pathlib import Path
from typing import Dict, Set

import websockets
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("unified-demo")

# Configuration
DEMO_PORT = 8080
BRIDGE_PORT = 8765
EXAMPLES_DIR = Path("/app/examples")
STATIC_DIR = Path("/app/static")

# Track running demos
running_demos: Dict[str, subprocess.Popen] = {}
active_ws_clients: Set[websockets.WebSocketServerProtocol] = set()

# Example definitions
EXAMPLES = {
    "talking-garden": {
        "name": "Talking Garden",
        "path": "playground/talking-garden",
        "port": 8081,
    },
    "mars-colony": {
        "name": "Mars Colony",
        "path": "playground/mars-colony",
        "port": 8082,
    },
    "theater-bots": {
        "name": "Theater Bots",
        "path": "playground/theater-bots",
        "port": 8083,
    },
    "art-studio": {
        "name": "Art Studio",
        "path": "playground/art-studio",
        "port": 8084,
    },
    "actions": {
        "name": "Actions Demo",
        "path": "actions",
        "port": 8085,
    },
    "auth": {
        "name": "Auth Demo",
        "path": "auth",
        "port": 8086,
    },
    "fleet": {
        "name": "Fleet Demo",
        "path": "fleet",
        "port": 8087,
    },
    "metrics": {
        "name": "Metrics Demo",
        "path": "metrics",
        "port": 8088,
    },
    "mqtt_iot": {
        "name": "MQTT IoT Demo",
        "path": "mqtt_iot",
        "port": 8089,
    },
    "quickstart": {
        "name": "Quickstart",
        "path": "quickstart",
        "port": 8090,
    },
}


class DemoHTTPHandler(BaseHTTPRequestHandler):
    """HTTP request handler for demo web UI"""
    
    def log_message(self, format, *args):
        logger.info(f"{self.address_string()} - {format % args}")
    
    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        
        # API endpoints
        if path == "/api/status":
            self.send_json({"running": True, "active_demos": list(running_demos.keys())})
            return
        
        # Serve static files
        if path == "/" or path == "/index.html":
            self.serve_file(STATIC_DIR / "index.html", "text/html")
        elif path.startswith("/static/"):
            file_path = STATIC_DIR / path[8:]
            if file_path.exists():
                content_type = "text/css" if path.endswith(".css") else "application/javascript"
                self.serve_file(file_path, content_type)
            else:
                self.send_error(404)
        else:
            self.send_error(404)
    
    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        
        if path.startswith("/api/start/"):
            example_id = path[11:]
            success = start_example(example_id)
            self.send_json({"success": success, "example": example_id})
        
        elif path.startswith("/api/stop/"):
            example_id = path[10:]
            success = stop_example(example_id)
            self.send_json({"success": success, "example": example_id})
        
        else:
            self.send_error(404)
    
    def serve_file(self, file_path: Path, content_type: str):
        try:
            with open(file_path, 'rb') as f:
                content = f.read()
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        except Exception as e:
            logger.error(f"Error serving file: {e}")
            self.send_error(500)
    
    def send_json(self, data: dict):
        content = json.dumps(data).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', len(content))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(content)


def start_example(example_id: str) -> bool:
    """Start an example demo"""
    if example_id in running_demos:
        logger.warning(f"Example {example_id} already running")
        return False
    
    if example_id not in EXAMPLES:
        logger.error(f"Unknown example: {example_id}")
        return False
    
    example = EXAMPLES[example_id]
    example_path = EXAMPLES_DIR / example["path"]
    
    if not example_path.exists():
        logger.error(f"Example path not found: {example_path}")
        return False
    
    try:
        # Start the example
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = "0"
        
        cmd = ["python3", "-m", "agent_ros_bridge.gateway_v2", "--port", str(example["port"])]
        
        process = subprocess.Popen(
            cmd,
            cwd=example_path,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        
        running_demos[example_id] = process
        logger.info(f"Started {example_id} on port {example['port']} (PID: {process.pid})")
        return True
        
    except Exception as e:
        logger.error(f"Failed to start {example_id}: {e}")
        return False


def stop_example(example_id: str) -> bool:
    """Stop a running example"""
    if example_id not in running_demos:
        logger.warning(f"Example {example_id} not running")
        return False
    
    process = running_demos[example_id]
    try:
        process.terminate()
        process.wait(timeout=5)
        del running_demos[example_id]
        logger.info(f"Stopped {example_id}")
        return True
    except Exception as e:
        logger.error(f"Error stopping {example_id}: {e}")
        process.kill()
        return True


async def bridge_websocket_handler(websocket, path):
    """Handle WebSocket connections to the main bridge"""
    active_ws_clients.add(websocket)
    logger.info(f"WebSocket client connected: {websocket.remote_address}")
    
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                logger.info(f"Received: {data}")
                
                # Echo back for demo
                response = {
                    "status": "ok",
                    "received": data,
                    "bridge": "Agent ROS Bridge v0.3.5",
                    "active_demos": list(running_demos.keys())
                }
                await websocket.send(json.dumps(response))
                
            except json.JSONDecodeError:
                await websocket.send(json.dumps({"status": "error", "message": "Invalid JSON"}))
    
    except websockets.exceptions.ConnectionClosed:
        logger.info(f"WebSocket client disconnected: {websocket.remote_address}")
    finally:
        active_ws_clients.discard(websocket)


async def start_http_server():
    """Start HTTP server for web UI"""
    server = HTTPServer(('0.0.0.0', DEMO_PORT), DemoHTTPHandler)
    logger.info(f"HTTP server started on http://0.0.0.0:{DEMO_PORT}")
    
    # Run in executor to not block
    loop = asyncio.get_event_loop()
    await loop.run_in_executor(None, server.serve_forever)


async def start_bridge_websocket():
    """Start WebSocket server for bridge API"""
    server = await websockets.serve(bridge_websocket_handler, '0.0.0.0', BRIDGE_PORT)
    logger.info(f"Bridge WebSocket started on ws://0.0.0.0:{BRIDGE_PORT}")
    return server


async def main():
    """Main entry point"""
    print("=" * 60)
    print("🤖 Agent ROS Bridge - Unified Demo")
    print("=" * 60)
    print()
    print(f"Web UI:    http://localhost:{DEMO_PORT}")
    print(f"WebSocket: ws://localhost:{BRIDGE_PORT}")
    print()
    print("Available examples:")
    for key, ex in EXAMPLES.items():
        print(f"  - {ex['name']} ({key})")
    print()
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    # Start HTTP and WebSocket servers
    http_task = asyncio.create_task(start_http_server())
    bridge_ws = await start_bridge_websocket()
    
    try:
        await asyncio.gather(http_task)
    except asyncio.CancelledError:
        logger.info("Shutting down...")
    finally:
        bridge_ws.close()
        # Stop all running demos
        for example_id in list(running_demos.keys()):
            stop_example(example_id)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n👋 Goodbye!")
        sys.exit(0)
