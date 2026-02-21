#!/usr/bin/env python3
"""Unified Demo Server for Agent ROS Bridge

Web-based selector for all examples with integrated ROS2 environment.
Each example gets its own homepage accessible through the unified server.
"""

import asyncio
import json
import logging
import os
import subprocess
import sys
from pathlib import Path
from typing import Dict, Set, Optional
import urllib.request
import urllib.error

import websockets
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, unquote

# Disable proxy for localhost connections (critical for unified demo)
os.environ['NO_PROXY'] = 'localhost,127.0.0.1'
os.environ['no_proxy'] = 'localhost,127.0.0.1'

# Create proxy handler that bypasses localhost
proxy_handler = urllib.request.ProxyHandler({
    'http': '',
    'https': ''
})
# Build opener with proxy handler
opener = urllib.request.build_opener(proxy_handler)
urllib.request.install_opener(opener)

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("unified-demo")

# Configuration
DEMO_PORT = 8080
BRIDGE_PORT = 8765

# Paths - work both in Docker and native
SCRIPT_DIR = Path(__file__).parent.resolve()
EXAMPLES_DIR = SCRIPT_DIR.parent / "examples" if (SCRIPT_DIR.parent / "examples").exists() else Path("/app/examples")
STATIC_DIR = SCRIPT_DIR / "static" if (SCRIPT_DIR / "static").exists() else Path("/app/static")

# Track running demos
running_demos: Dict[str, subprocess.Popen] = {}
active_ws_clients: Set[websockets.WebSocketServerProtocol] = set()

# Example definitions with homepage info
EXAMPLES = {
    "talking-garden": {
        "name": "Talking Garden",
        "path": "playground/talking-garden",
        "port": 8081,
        "homepage": "garden.html",
        "description": "AI monitors and converses with 6 IoT-enabled plants",
    },
    "mars-colony": {
        "name": "Mars Colony",
        "path": "playground/mars-colony",
        "port": 8082,
        "homepage": "dashboard.html",
        "description": "Multi-robot Mars mission with 4 robot types",
    },
    "theater-bots": {
        "name": "Theater Bots",
        "path": "playground/theater-bots",
        "port": 8083,
        "homepage": "stage.html",
        "description": "AI director controls robot actors on stage",
    },
    "art-studio": {
        "name": "Art Studio",
        "path": "playground/art-studio",
        "port": 8084,
        "homepage": "canvas.html",
        "description": "Human and robot collaborative painting",
    },
    "actions": {
        "name": "Actions Demo",
        "path": "actions",
        "port": 8085,
        "homepage": "index.html",
        "description": "ROS Actions for navigation and manipulation",
    },
    "auth": {
        "name": "Auth Demo",
        "path": "auth",
        "port": 8086,
        "homepage": "index.html",
        "description": "JWT authentication demonstration",
    },
    "fleet": {
        "name": "Fleet Demo",
        "path": "fleet",
        "port": 8087,
        "homepage": "index.html",
        "description": "Multi-robot fleet coordination",
    },
    "metrics": {
        "name": "Metrics Demo",
        "path": "metrics",
        "port": 8088,
        "homepage": "index.html",
        "description": "Prometheus metrics collection",
    },
    "mqtt_iot": {
        "name": "MQTT IoT Demo",
        "path": "mqtt_iot",
        "port": 8089,
        "homepage": "index.html",
        "description": "MQTT transport for IoT sensors",
    },
    "quickstart": {
        "name": "Quickstart",
        "path": "quickstart",
        "port": 8090,
        "homepage": "index.html",
        "description": "Basic bridge usage tutorial",
    },
}


class DemoHTTPHandler(BaseHTTPRequestHandler):
    """HTTP request handler for demo web UI and example proxying"""
    
    def log_message(self, format, *args):
        logger.info(f"{self.address_string()} - {format % args}")
    
    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        
        # Serve selector at root
        if path == "/" or path == "/index.html":
            self.serve_file(STATIC_DIR / "index.html", "text/html")
            return
        
        # Serve static assets
        if path.startswith("/static/"):
            file_path = STATIC_DIR / path[8:]
            if file_path.exists():
                content_type = "text/css" if path.endswith(".css") else "application/javascript"
                self.serve_file(file_path, content_type)
            else:
                self.send_error(404)
            return
        
        # Proxy to example homepage: /demo/{example-id}/
        if path.startswith("/demo/"):
            parts = path.split("/")
            if len(parts) >= 3:
                example_id = parts[2]
                remaining_path = "/".join(parts[3:]) or ""
                self.proxy_to_example(example_id, remaining_path)
                return
        
        # API endpoints
        if path == "/api/status":
            self.send_json({
                "running": True,
                "active_demos": list(running_demos.keys()),
                "examples": {k: {"name": v["name"], "port": v["port"], "running": k in running_demos} 
                            for k, v in EXAMPLES.items()}
            })
            return
        
        self.send_error(404)
    
    def proxy_to_example(self, example_id: str, sub_path: str):
        """Proxy request to running example's web server"""
        if example_id not in EXAMPLES:
            self.send_error(404, f"Example '{example_id}' not found")
            return
        
        example = EXAMPLES[example_id]
        target_port = example["port"]
        homepage = example["homepage"]
        
        # If example not running, show status page with start button
        if example_id not in running_demos:
            self.send_html(f"""
            <html>
            <head><title>{example['name']} - Not Running</title>
            <style>
                body {{ font-family: -apple-system, BlinkMacSystemFont, sans-serif; background: #1a1a2e; color: #fff; text-align: center; padding: 50px; }}
                h1 {{ color: #e74c3c; }}
                .btn {{ padding: 15px 30px; border: none; border-radius: 8px; color: #fff; font-size: 1.1em; cursor: pointer; text-decoration: none; display: inline-block; margin: 10px; }}
                .btn-start {{ background: linear-gradient(90deg, #2ecc71, #27ae60); }}
                .btn-back {{ background: linear-gradient(90deg, #00d4ff, #7b2cbf); }}
                .info {{ background: rgba(255,255,255,0.1); padding: 20px; border-radius: 12px; max-width: 600px; margin: 20px auto; }}
            </style></head>
            <body>
                <h1>⏹ {example['name']} is not running</h1>
                <div class="info">
                    <p>{example['description']}</p>
                    <p>Start the example to access its dashboard.</p>
                </div>
                <button class="btn btn-start" onclick="startExample()">▶ Start {example['name']}</button>
                <a href="/" class="btn btn-back">← Back to Selector</a>
                <script>
                    function startExample() {{
                        fetch('/api/start/{example_id}', {{ method: 'POST' }})
                            .then(r => r.json())
                            .then(data => {{
                                if (data.success) {{
                                    window.location.reload();
                                }} else {{
                                    alert('Failed to start: ' + data.error);
                                }}
                            }})
                            .catch(e => alert('Error: ' + e));
                    }}
                </script>
            </body>
            </html>
            """)
            return
        
        # Build target URL
        target_path = sub_path if sub_path else homepage
        target_url = f"http://localhost:{target_port}/{target_path}"
        
        try:
            # Proxy the request
            with urllib.request.urlopen(target_url, timeout=5) as response:
                content = response.read()
                content_type = response.headers.get('Content-Type', 'text/html')
                
                self.send_response(response.status)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', len(content))
                self.end_headers()
                self.wfile.write(content)
        
        except urllib.error.HTTPError as e:
            self.send_error(e.code, e.reason)
        except urllib.error.URLError as e:
            # Connection refused - example server not ready yet
            logger.warning(f"Cannot connect to {example_id} at {target_url}: {e}")
            self.send_html(f"""
            <html>
            <head><title>{example['name']} - Starting...</title>
            <style>
                body {{ font-family: -apple-system, BlinkMacSystemFont, sans-serif; background: #1a1a2e; color: #fff; text-align: center; padding: 50px; }}
                h1 {{ color: #f39c12; }}
                .btn {{ padding: 15px 30px; background: linear-gradient(90deg, #00d4ff, #7b2cbf); border: none; border-radius: 8px; color: #fff; font-size: 1.1em; cursor: pointer; text-decoration: none; display: inline-block; margin: 10px; }}
                .btn-start {{ background: linear-gradient(90deg, #2ecc71, #27ae60); }}
                .info {{ background: rgba(255,255,255,0.1); padding: 20px; border-radius: 12px; max-width: 600px; margin: 20px auto; }}
                code {{ background: rgba(0,0,0,0.3); padding: 2px 6px; border-radius: 4px; }}
            </style>
            <meta http-equiv="refresh" content="3">
            </head>
            <body>
                <h1>⏳ {example['name']} is starting...</h1>
                <div class="info">
                    <p>The example server is not ready yet.</p>
                    <p>This page will auto-refresh in 3 seconds.</p>
                    <p>If it doesn't load, the example may need to be started from the <a href="/" style="color: #00d4ff;">main selector</a>.</p>
                </div>
                <a href="/" class="btn">← Back to Selector</a>
            </body>
            </html>
            """)
        except Exception as e:
            logger.error(f"Proxy error: {e}")
            self.send_error(502, f"Example server error: {e}")
    
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
    
    def send_html(self, html: str):
        content = html.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html')
        self.send_header('Content-Length', len(content))
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
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = "0"
        
        # Find the right command based on available files
        if (example_path / "docker-compose.yml").exists():
            # Run via docker-compose (simpler for unified demo - just run Python directly)
            cmd = [
                "python3", "-m", "agent_ros_bridge.gateway_v2",
                "--port", str(example["port"]),
                "--host", "0.0.0.0"
            ]
        else:
            cmd = [
                "python3", "-m", "agent_ros_bridge.gateway_v2",
                "--port", str(example["port"]),
                "--host", "0.0.0.0"
            ]
        
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
    logger.info(f"Unified Demo Server: http://0.0.0.0:{DEMO_PORT}")
    logger.info(f"Example access: http://0.0.0.0:{DEMO_PORT}/demo/{{example-id}}/")
    
    loop = asyncio.get_event_loop()
    await loop.run_in_executor(None, server.serve_forever)


async def start_bridge_websocket():
    """Start WebSocket server for bridge API"""
    server = await websockets.serve(bridge_websocket_handler, '0.0.0.0', BRIDGE_PORT)
    logger.info(f"Bridge WebSocket: ws://0.0.0.0:{BRIDGE_PORT}")
    return server


async def main():
    """Main entry point"""
    print("=" * 60)
    print("🤖 Agent ROS Bridge - Unified Demo")
    print("=" * 60)
    print()
    print(f"Selector:     http://localhost:{DEMO_PORT}")
    print(f"Examples:     http://localhost:{DEMO_PORT}/demo/{{name}}/")
    print(f"WebSocket:    ws://localhost:{BRIDGE_PORT}")
    print()
    print("Available examples:")
    for key, ex in EXAMPLES.items():
        homepage = ex.get('homepage', 'index.html')
        print(f"  - {ex['name']}: /demo/{key}/ → {homepage}")
    print()
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    http_task = asyncio.create_task(start_http_server())
    bridge_ws = await start_bridge_websocket()
    
    try:
        await asyncio.gather(http_task)
    except asyncio.CancelledError:
        logger.info("Shutting down...")
    finally:
        bridge_ws.close()
        for example_id in list(running_demos.keys()):
            stop_example(example_id)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n👋 Goodbye!")
        sys.exit(0)
