#!/usr/bin/env python3
"""Web Dashboard Server for Agent ROS Bridge

Serves the HTML dashboard and optionally proxies WebSocket connections.
"""

import asyncio
import http.server
import socketserver
import os
import json
import logging
from pathlib import Path

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("dashboard")

# Get the directory containing this script
DASHBOARD_DIR = Path(__file__).parent / "static"


class DashboardHandler(http.server.SimpleHTTPRequestHandler):
    """HTTP request handler for dashboard"""
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(DASHBOARD_DIR), **kwargs)
    
    def log_message(self, format, *args):
        logger.info(f"{self.address_string()} - {format % args}")
    
    def do_GET(self):
        """Handle GET requests"""
        # Serve index.html for root path
        if self.path == '/':
            self.path = '/index.html'
        
        # API endpoint for bridge status
        if self.path == '/api/status':
            self.send_json({
                "status": "ok",
                "dashboard": "running",
                "version": "0.2.0"
            })
            return
        
        return super().do_GET()
    
    def send_json(self, data):
        """Send JSON response"""
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())


class ThreadedHTTPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    """Threaded HTTP server"""
    allow_reuse_address = True


def run_server(port=8080):
    """Run the dashboard HTTP server"""
    with ThreadedHTTPServer(("0.0.0.0", port), DashboardHandler) as httpd:
        logger.info(f"Dashboard server running at http://localhost:{port}")
        logger.info(f"Serving files from: {DASHBOARD_DIR}")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            logger.info("Shutting down...")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Agent ROS Bridge Dashboard")
    parser.add_argument("--port", type=int, default=8080, help="HTTP port (default: 8080)")
    parser.add_argument("--bridge-url", default="ws://localhost:8766", 
                       help="WebSocket bridge URL")
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("🎛️  Agent ROS Bridge Dashboard")
    print("=" * 60)
    print(f"Dashboard URL: http://localhost:{args.port}")
    print(f"Bridge URL: {args.bridge_url}")
    print("")
    print("Features:")
    print("  • Real-time telemetry display")
    print("  • Robot control pad (click or arrow keys)")
    print("  • Message logging")
    print("  • JWT token authentication support")
    print("")
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    run_server(args.port)
