#!/usr/bin/env python3
"""
Agent ROS Bridge - Web Dashboard Server

Simple HTTP server to serve the web dashboard.
Also includes WebSocket proxy to bridge browser -> ROS Bridge connections.
"""

import argparse
import asyncio
import http.server
import json
import socketserver
import threading
from pathlib import Path

import websockets

# Configuration
WEB_DIR = Path(__file__).parent
DEFAULT_HTTP_PORT = 8080
DEFAULT_WS_PORT = 8765
DEFAULT_BRIDGE_URL = "ws://localhost:8765"


class DashboardHandler(http.server.SimpleHTTPRequestHandler):
    """HTTP request handler for dashboard files."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEB_DIR, **kwargs)

    def end_headers(self):
        # Add CORS headers for development
        self.send_header("Access-Control-Allow-Origin", "*")
        super().end_headers()

    def log_message(self, format, *args):
        # Custom logging
        print(f"[HTTP] {self.address_string()} - {format % args}")


class WebSocketProxy:
    """
    WebSocket proxy that bridges browser connections to Agent ROS Bridge.

    Browser <---> This Proxy <---> Agent ROS Bridge
    """

    def __init__(self, listen_port=8766, bridge_url=DEFAULT_BRIDGE_URL):
        self.listen_port = listen_port
        self.bridge_url = bridge_url
        self.clients = set()

    async def handle_client(self, websocket, path):
        """Handle a browser client connection."""
        print(f"[WS] Client connected from {websocket.remote_address}")
        self.clients.add(websocket)

        bridge_ws = None
        try:
            # Connect to the actual ROS Bridge
            bridge_ws = await websockets.connect(self.bridge_url)
            print(f"[WS] Connected to bridge at {self.bridge_url}")

            # Relay messages in both directions
            await asyncio.gather(
                self.relay_to_bridge(websocket, bridge_ws),
                self.relay_to_client(websocket, bridge_ws),
            )

        except Exception as e:
            print(f"[WS] Error: {e}")
        finally:
            self.clients.discard(websocket)
            if bridge_ws:
                await bridge_ws.close()
            print("[WS] Client disconnected")

    async def relay_to_bridge(self, client_ws, bridge_ws):
        """Relay messages from browser to bridge."""
        async for message in client_ws:
            try:
                data = json.loads(message)
                print(f"[WS] Browser -> Bridge: {data.get('type', 'unknown')}")
                await bridge_ws.send(message)
            except json.JSONDecodeError:
                await bridge_ws.send(message)

    async def relay_to_client(self, client_ws, bridge_ws):
        """Relay messages from bridge to browser."""
        async for message in bridge_ws:
            try:
                data = json.loads(message)
                print(f"[WS] Bridge -> Browser: {data.get('type', 'unknown')}")
                await client_ws.send(message)
            except json.JSONDecodeError:
                await client_ws.send(message)

    async def start(self):
        """Start the WebSocket proxy server."""
        print(f"[WS] Proxy server starting on port {self.listen_port}")
        print(f"[WS] Forwarding to {self.bridge_url}")

        async with websockets.serve(self.handle_client, "localhost", self.listen_port):
            await asyncio.Future()  # Run forever


def start_http_server(port):
    """Start the HTTP server for dashboard files."""
    with socketserver.TCPServer(("", port), DashboardHandler) as httpd:
        print(f"[HTTP] Dashboard server running at http://localhost:{port}")
        print(f"[HTTP] Serving files from {WEB_DIR}")
        httpd.serve_forever()


def main():
    parser = argparse.ArgumentParser(description="Agent ROS Bridge Web Dashboard Server")
    parser.add_argument(
        "--http-port",
        type=int,
        default=DEFAULT_HTTP_PORT,
        help=f"HTTP server port (default: {DEFAULT_HTTP_PORT})",
    )
    parser.add_argument(
        "--ws-port", type=int, default=8766, help="WebSocket proxy port (default: 8766)"
    )
    parser.add_argument(
        "--bridge-url",
        default=DEFAULT_BRIDGE_URL,
        help=f"Agent ROS Bridge WebSocket URL (default: {DEFAULT_BRIDGE_URL})",
    )
    parser.add_argument(
        "--no-proxy",
        action="store_true",
        help="Disable WebSocket proxy (connect browser directly to bridge)",
    )

    args = parser.parse_args()

    print("=" * 60)
    print("Agent ROS Bridge - Web Dashboard Server")
    print("=" * 60)
    print()

    # Start HTTP server in a thread
    http_thread = threading.Thread(target=start_http_server, args=(args.http_port,), daemon=True)
    http_thread.start()

    print()
    print(f"🌐 Dashboard: http://localhost:{args.http_port}")

    if not args.no_proxy:
        print(f"📡 WS Proxy:  ws://localhost:{args.ws_port}")
        print(f"🔗 Bridge:    {args.bridge_url}")
        print()
        print("Browser connects to proxy, proxy connects to bridge")

        # Start WebSocket proxy
        proxy = WebSocketProxy(args.ws_port, args.bridge_url)
        try:
            asyncio.run(proxy.start())
        except KeyboardInterrupt:
            print("\n[WS] Proxy stopped")
    else:
        print()
        print("Browser connects directly to bridge")
        print(f"🔗 Bridge: {args.bridge_url}")

        # Keep main thread alive
        try:
            while True:
                threading.Event().wait(1)
        except KeyboardInterrupt:
            pass

    print("\nServer stopped")


if __name__ == "__main__":
    main()
