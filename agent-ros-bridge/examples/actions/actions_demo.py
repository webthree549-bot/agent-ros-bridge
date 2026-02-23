#!/usr/bin/env python3
"""Actions Demo for Agent ROS Bridge.

This demo shows how to:
1. Create a ROSBridge with WebSocket and gRPC transports
2. Connect to ROS2 (or run in simulation mode)
3. Register actions that can be triggered via the dashboard

Usage:
    # With ROS2 installed:
    python3 actions_demo.py
    
    # Without ROS2 (simulation mode):
    python3 actions_demo.py --mock
"""

import asyncio
import argparse
import logging
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("actions-demo")

from agent_ros_bridge import ROSBridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
from agent_ros_bridge.gateway_v2.transports.grpc import GRPCServer

STATIC_DIR = Path(__file__).parent


class DashboardHandler(BaseHTTPRequestHandler):
    """Simple HTTP handler for serving the dashboard."""
    
    def log_message(self, format, *args):
        # Suppress default logging
        pass
    
    def do_GET(self):
        if self.path == "/":
            try:
                with open(STATIC_DIR / "index.html", "rb") as f:
                    content = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(content)
            except FileNotFoundError:
                self.send_error(404, "index.html not found")
        else:
            self.send_error(404)


def http_server():
    """Run HTTP server in background thread."""
    logger.info("HTTP server starting on port 8773")
    HTTPServer(("0.0.0.0", 8773), DashboardHandler).serve_forever()


def create_ros_connector(ros_version: int = 2, mock: bool = False):
    """Create appropriate ROS connector based on version and availability.
    
    Args:
        ros_version: 1 for ROS1, 2 for ROS2
        mock: If True, don't try to create real connector
        
    Returns:
        ROS connector instance or None if not available
    """
    if mock:
        logger.info("Running in mock mode (no ROS connection)")
        return None
    
    if ros_version == 2:
        try:
            from agent_ros_bridge.gateway_v2.connectors.ros2 import ROS2Connector
            connector = ROS2Connector({"domain_id": 0})
            logger.info("ROS2 connector created successfully")
            return connector
        except RuntimeError as e:
            logger.warning(f"ROS2 not available: {e}")
            logger.info("Run with --mock flag to skip ROS connection")
            return None
    else:
        try:
            from agent_ros_bridge.gateway_v2.connectors.ros1 import ROS1Connector
            connector = ROS1Connector({"node_name": "agent_ros_bridge_demo"})
            logger.info("ROS1 connector created successfully")
            return connector
        except RuntimeError as e:
            logger.warning(f"ROS1 not available: {e}")
            logger.info("Run with --mock flag to skip ROS connection")
            return None


async def main():
    parser = argparse.ArgumentParser(description="Agent ROS Bridge Actions Demo")
    parser.add_argument(
        "--ros-version",
        type=int,
        choices=[1, 2],
        default=2,
        help="ROS version to use (1 or 2, default: 2)"
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Run without ROS connection (simulation mode)"
    )
    args = parser.parse_args()
    
    print("=" * 60)
    print("⚡ Agent ROS Bridge - Actions Demo")
    print("=" * 60)
    
    # Start HTTP server
    logger.info("Starting HTTP dashboard server on port 8773...")
    threading.Thread(target=http_server, daemon=True).start()
    await asyncio.sleep(0.5)
    
    # Create bridge
    bridge = ROSBridge(ros_version=args.ros_version)
    
    # Add transports
    bridge.transport_manager.register(
        WebSocketTransport({
            "host": "0.0.0.0",
            "port": 8765,
            "auth": {"enabled": False}
        })
    )
    bridge.transport_manager.register(
        GRPCServer({
            "host": "0.0.0.0",
            "port": 50051,
            "auth": {"enabled": False}
        })
    )
    
    # Add ROS connector (if available)
    ros_connector = create_ros_connector(args.ros_version, args.mock)
    if ros_connector:
        bridge.connector_manager.register(ros_connector)
    
    # Register demo actions
    @bridge.action("navigate")
    async def navigate(x: float, y: float, theta: float = 0.0):
        """Navigate to a position."""
        logger.info(f"Navigation requested: x={x}, y={y}, theta={theta}")
        # In a real implementation, this would publish to /cmd_vel or call a navigation action
        return {
            "status": "success",
            "action": "navigate",
            "target": {"x": x, "y": y, "theta": theta},
            "message": "Navigation command sent"
        }
    
    @bridge.action("move_arm")
    async def move_arm(position: str):
        """Move robotic arm to a position."""
        logger.info(f"Arm movement requested: position={position}")
        valid_positions = ["home", "ready", "grip", "drop"]
        if position not in valid_positions:
            return {
                "status": "error",
                "message": f"Invalid position. Valid: {valid_positions}"
            }
        return {
            "status": "success",
            "action": "move_arm",
            "position": position
        }
    
    @bridge.action("grasp")
    async def grasp(state: str):
        """Control the gripper."""
        logger.info(f"Grasp requested: state={state}")
        valid_states = ["open", "close", "half"]
        if state not in valid_states:
            return {
                "status": "error",
                "message": f"Invalid state. Valid: {valid_states}"
            }
        return {
            "status": "success",
            "action": "grasp",
            "state": state
        }
    
    @bridge.action("patrol")
    async def patrol(area: str, rounds: int = 1):
        """Start a patrol route."""
        logger.info(f"Patrol requested: area={area}, rounds={rounds}")
        return {
            "status": "success",
            "action": "patrol",
            "area": area,
            "rounds": rounds
        }
    
    # Print info
    print()
    print("🌐 Dashboard: http://localhost:8773")
    print("📡 WebSocket: ws://localhost:8765")
    print("🔗 gRPC:      localhost:50051")
    print()
    print("Registered actions:")
    for action in bridge.get_registered_actions():
        print(f"  • {action}")
    print()
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    try:
        await bridge.start()
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")
        await bridge.stop()


if __name__ == "__main__":
    asyncio.run(main())
