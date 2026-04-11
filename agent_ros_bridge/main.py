#!/usr/bin/env python3
"""
Agent ROS Bridge - Unified Entry Point

Consolidated CLI for gateway, dashboard, and robot API modes.

Usage:
    agent-ros-bridge [command] [options]

Commands:
    start       Start the bridge gateway (default)
    dashboard   Start the web dashboard only
    robot       Start robot API mode
    status      Check bridge status
    config      Manage configuration
    skill       Manage OpenClaw skills

Examples:
    agent-ros-bridge                          # Start gateway with defaults
    agent-ros-bridge start --port 8765        # Start gateway on port 8765
    agent-ros-bridge dashboard                # Start dashboard only
    agent-ros-bridge robot --device-id bot1   # Start robot API mode
    agent-ros-bridge status                   # Check status
"""

import argparse
import asyncio
import logging
import os
import signal
import sys
import webbrowser
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path


# Configure logging
def setup_logging(level: str = "INFO") -> logging.Logger:
    """Setup logging configuration."""
    logging.basicConfig(
        level=getattr(logging, level.upper()),
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    return logging.getLogger("agent_ros_bridge")


@dataclass
class BridgeConfig:
    """Bridge configuration."""
    websocket_port: int = 8765
    grpc_port: int = 50051
    mqtt_port: int = 1883
    host: str = "0.0.0.0"
    config_file: str | None = None
    debug: bool = False
    jwt_secret: str | None = None


def get_version() -> str:
    """Get package version."""
    try:
        from . import __version__
        return __version__
    except ImportError:
        return "0.6.7"


def create_parser() -> argparse.ArgumentParser:
    """Create argument parser with subcommands."""
    parser = argparse.ArgumentParser(
        prog="agent-ros-bridge",
        description="Universal ROS1/ROS2 bridge for AI agents to control robots",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                          # Start gateway with defaults
  %(prog)s start --port 8765        # Start gateway on custom port
  %(prog)s dashboard                # Start web dashboard only
  %(prog)s robot --device-id bot1   # Start robot API mode
  %(prog)s status                   # Check bridge status

For more help: %(prog)s [command] --help
        """,
    )

    parser.add_argument(
        "--version", "-v",
        action="version",
        version=f"%(prog)s {get_version()}"
    )

    parser.add_argument(
        "--debug", "-d",
        action="store_true",
        help="Enable debug logging"
    )

    # Subcommands
    subparsers = parser.add_subparsers(dest="command", help="Command to run")

    # Start command (default gateway mode)
    start_parser = subparsers.add_parser(
        "start",
        help="Start the bridge gateway",
        description="Start the Agent ROS Bridge gateway with WebSocket, gRPC, and MQTT transports."
    )
    start_parser.add_argument(
        "--config", "-c",
        type=str,
        help="Path to configuration file (YAML or JSON)"
    )
    start_parser.add_argument(
        "--port", "-p",
        type=int,
        default=8765,
        help="WebSocket port (default: 8765)"
    )
    start_parser.add_argument(
        "--grpc-port", "-g",
        type=int,
        default=50051,
        help="gRPC port (default: 50051)"
    )
    start_parser.add_argument(
        "--mqtt-port", "-m",
        type=int,
        default=1883,
        help="MQTT port (default: 1883)"
    )
    start_parser.add_argument(
        "--host", "-H",
        type=str,
        default="0.0.0.0",
        help="Host to bind to (default: 0.0.0.0)"
    )
    start_parser.add_argument(
        "--jwt-secret", "-j",
        type=str,
        default=None,
        help="JWT secret (or set JWT_SECRET env var)"
    )
    start_parser.add_argument(
        "--no-ros2",
        action="store_true",
        help="Disable ROS2 connector"
    )

    # Dashboard command
    dashboard_parser = subparsers.add_parser(
        "dashboard",
        help="Start the web dashboard",
        description="Start the web dashboard for monitoring robot state."
    )
    dashboard_parser.add_argument(
        "--port", "-p",
        type=int,
        default=8080,
        help="Dashboard port (default: 8080)"
    )
    dashboard_parser.add_argument(
        "--host",
        type=str,
        default="127.0.0.1",
        help="Host to bind to (default: 127.0.0.1)"
    )
    dashboard_parser.add_argument(
        "--bridge-port",
        type=int,
        default=8765,
        help="Bridge WebSocket port (default: 8765)"
    )
    dashboard_parser.add_argument(
        "--no-browser",
        action="store_true",
        help="Don't open browser automatically"
    )

    # Robot command
    robot_parser = subparsers.add_parser(
        "robot",
        help="Start robot API mode",
        description="Start in robot API mode for direct robot control."
    )
    robot_parser.add_argument(
        "--device-id", "-i",
        type=str,
        required=True,
        help="Robot device ID"
    )
    robot_parser.add_argument(
        "--device-type", "-t",
        type=str,
        default="mobile_robot",
        choices=["mobile_robot", "manipulator", "drone", "humanoid", "sensor_array"],
        help="Robot type (default: mobile_robot)"
    )
    robot_parser.add_argument(
        "--port", "-p",
        type=int,
        default=8765,
        help="Bridge port to connect to (default: 8765)"
    )
    robot_parser.add_argument(
        "--auto-discover",
        action="store_true",
        help="Auto-discover robot capabilities"
    )

    # Status command
    status_parser = subparsers.add_parser(
        "status",
        help="Check bridge status",
        description="Check the status of the running bridge."
    )
    status_parser.add_argument(
        "--host",
        type=str,
        default="localhost",
        help="Bridge host (default: localhost)"
    )
    status_parser.add_argument(
        "--port", "-p",
        type=int,
        default=8765,
        help="Bridge port (default: 8765)"
    )

    # Config command
    config_parser = subparsers.add_parser(
        "config",
        help="Manage configuration",
        description="Manage Agent ROS Bridge configuration."
    )
    config_parser.add_argument(
        "--init",
        action="store_true",
        help="Initialize default configuration"
    )
    config_parser.add_argument(
        "--validate",
        type=str,
        help="Validate configuration file"
    )
    config_parser.add_argument(
        "--output", "-o",
        type=str,
        default="config/bridge.yaml",
        help="Output file for --init (default: config/bridge.yaml)"
    )

    return parser


async def run_gateway(args: argparse.Namespace, logger: logging.Logger) -> int:
    """Run the bridge gateway."""
    try:
        from . import Bridge
        from .gateway_v2.config import ConfigLoader
        from .gateway_v2.transports.mqtt_transport import MQTTTransport
        from .gateway_v2.transports.websocket import WebSocketTransport
    except ImportError as e:
        logger.error(f"Failed to import gateway modules: {e}")
        return 1

    # Optional imports
    try:
        from .gateway_v2.transports.grpc_transport import GRPCTransport
    except ImportError:
        GRPCTransport = None
        logger.debug("gRPC transport not available")

    try:
        from .gateway_v2.connectors.ros2_connector import ROS2Connector
    except ImportError:
        ROS2Connector = None
        logger.debug("ROS2 connector not available")

    logger.info("=" * 60)
    logger.info(f"Agent ROS Bridge v{get_version()}")
    logger.info("Starting gateway...")
    logger.info("=" * 60)

    # Load configuration
    config = None
    if args.config:
        try:
            config = ConfigLoader.from_file(args.config)
            logger.info(f"Configuration loaded from: {args.config}")
        except Exception as e:
            logger.warning(f"Failed to load config file: {e}, using defaults")

    if config is None:
        config = BridgeConfig()

    # Apply command line overrides
    config.websocket_port = args.port
    config.grpc_port = args.grpc_port
    config.mqtt_port = args.mqtt_port
    config.host = args.host

    # Handle JWT secret
    jwt_secret = args.jwt_secret or os.environ.get("JWT_SECRET")
    if not jwt_secret:
        logger.warning("No JWT secret provided - authentication disabled")

    # Create bridge
    bridge = Bridge()

    # Register transports
    transports = []

    # WebSocket
    ws_transport = WebSocketTransport(
        "websocket",
        {"host": config.host, "port": config.websocket_port}
    )
    bridge.transport_manager.register(ws_transport)
    transports.append(f"WebSocket ({config.host}:{config.websocket_port})")

    # gRPC
    if GRPCTransport:
        grpc_transport = GRPCTransport(
            {"host": config.host, "port": config.grpc_port}
        )
        bridge.transport_manager.register(grpc_transport)
        transports.append(f"gRPC ({config.host}:{config.grpc_port})")

    # MQTT
    mqtt_transport = MQTTTransport(
        {"host": config.host, "port": config.mqtt_port}
    )
    bridge.transport_manager.register(mqtt_transport)
    transports.append(f"MQTT ({config.host}:{config.mqtt_port})")

    # Register ROS2 connector
    if ROS2Connector and not args.no_ros2:
        ros2_connector = ROS2Connector()
        bridge.connector_registry.register(ros2_connector)
        transports.append("ROS2 Connector")

    logger.info(f"Registered transports: {', '.join(transports)}")

    # Setup signal handlers
    loop = asyncio.get_event_loop()

    def signal_handler(sig):
        logger.info(f"Received signal {sig.name}, shutting down...")
        asyncio.create_task(bridge.stop())

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda s=sig: signal_handler(s))

    # Start bridge
    try:
        await bridge.start()
        logger.info("Gateway started successfully")
        logger.info("Press Ctrl+C to stop")

        # Keep running
        while getattr(bridge, 'running', True):
            await asyncio.sleep(1)

    except Exception as e:
        logger.error(f"Gateway error: {e}")
        return 1
    finally:
        await bridge.stop()

    logger.info("Gateway stopped")
    return 0


def run_dashboard(args: argparse.Namespace, logger: logging.Logger) -> int:
    """Run the web dashboard."""
    logger.info("=" * 60)
    logger.info(f"Agent ROS Bridge Dashboard v{get_version()}")
    logger.info("=" * 60)

    # Simple HTML dashboard
    DASHBOARD_HTML = """<!DOCTYPE html>
<html>
<head>
    <title>Agent ROS Bridge Dashboard</title>
    <meta charset="utf-8">
    <style>
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: #1a1a2e;
            color: #eee;
            margin: 0;
            padding: 20px;
        }
        .header {
            text-align: center;
            padding: 20px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            border-radius: 10px;
            margin-bottom: 20px;
        }
        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
        }
        .card {
            background: #16213e;
            border-radius: 10px;
            padding: 20px;
            border: 1px solid #0f3460;
        }
        .card h3 {
            margin-top: 0;
            color: #e94560;
        }
        .status-indicator {
            display: inline-block;
            width: 12px;
            height: 12px;
            border-radius: 50%;
            margin-right: 8px;
        }
        .status-online { background: #00d9ff; box-shadow: 0 0 10px #00d9ff; }
        .status-offline { background: #e94560; }
        .metric {
            display: flex;
            justify-content: space-between;
            padding: 10px 0;
            border-bottom: 1px solid #0f3460;
        }
        .metric:last-child { border-bottom: none; }
        .metric-label { color: #a0a0a0; }
        .metric-value { font-weight: bold; color: #00d9ff; }
    </style>
</head>
<body>
    <div class="header">
        <h1>🤖 Agent ROS Bridge Dashboard</h1>
        <p>Real-time robot monitoring and control</p>
    </div>

    <div class="status-grid">
        <div class="card">
            <h3><span class="status-indicator status-online"></span>Bridge Status</h3>
            <div class="metric">
                <span class="metric-label">Status</span>
                <span class="metric-value" id="bridge-status">Connecting...</span>
            </div>
            <div class="metric">
                <span class="metric-label">WebSocket</span>
                <span class="metric-value" id="websocket-status">--</span>
            </div>
            <div class="metric">
                <span class="metric-label">Connected Robots</span>
                <span class="metric-value" id="robot-count">0</span>
            </div>
        </div>

        <div class="card">
            <h3>📊 System Metrics</h3>
            <div class="metric">
                <span class="metric-label">Uptime</span>
                <span class="metric-value" id="uptime">--</span>
            </div>
            <div class="metric">
                <span class="metric-label">Messages/sec</span>
                <span class="metric-value" id="msg-rate">--</span>
            </div>
            <div class="metric">
                <span class="metric-label">Latency</span>
                <span class="metric-value" id="latency">--</span>
            </div>
        </div>
    </div>

    <script>
        // Connect to bridge WebSocket
        const ws = new WebSocket('ws://localhost:BRIDGE_PORT');

        ws.onopen = () => {
            document.getElementById('bridge-status').textContent = 'Connected';
            document.getElementById('websocket-status').textContent = '✅ Online';
        };

        ws.onclose = () => {
            document.getElementById('bridge-status').textContent = 'Disconnected';
            document.getElementById('websocket-status').textContent = '❌ Offline';
        };

        ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if (data.type === 'telemetry') {
                document.getElementById('robot-count').textContent = data.robot_count || 0;
            }
        };
    </script>
</body>
</html>""".replace("BRIDGE_PORT", str(args.bridge_port))

    class DashboardHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(DASHBOARD_HTML.encode())

        def log_message(self, format, *args):
            logger.debug(f"Dashboard: {format % args}")

    # Start HTTP server
    server = HTTPServer((args.host, args.port), DashboardHandler)
    logger.info(f"Dashboard running at http://{args.host}:{args.port}")

    # Open browser
    if not args.no_browser:
        webbrowser.open(f"http://{args.host}:{args.port}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logger.info("Dashboard stopped")
    finally:
        server.shutdown()

    return 0


async def run_robot_mode(args: argparse.Namespace, logger: logging.Logger) -> int:
    """Run in robot API mode."""
    try:
        from .agentic import RobotAgent
    except ImportError as e:
        logger.error(f"Failed to import RobotAgent: {e}")
        return 1

    logger.info("=" * 60)
    logger.info(f"Agent ROS Bridge Robot Mode v{get_version()}")
    logger.info("=" * 60)

    # Create robot agent
    try:
        robot = RobotAgent(
            device_id=args.device_id,
            device_type=args.device_type,
            require_confirmation=True,
        )
        logger.info(f"Robot agent created: {args.device_id} ({args.device_type})")
    except Exception as e:
        logger.error(f"Failed to create robot agent: {e}")
        return 1

    # Auto-discover if requested
    if args.auto_discover:
        logger.info("Auto-discovering robot capabilities...")
        try:
            capabilities = robot.discover_capabilities()
            logger.info(f"Discovered capabilities: {capabilities}")
        except Exception as e:
            logger.warning(f"Auto-discovery failed: {e}")

    # Print status
    status = robot.get_status()
    logger.info(f"Robot status: {status}")

    logger.info("Robot mode active. Use Ctrl+C to exit.")

    # Keep running
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        logger.info("Robot mode stopped")

    return 0


def run_status(args: argparse.Namespace, logger: logging.Logger) -> int:
    """Check bridge status."""
    import socket

    logger.info(f"Checking bridge status at {args.host}:{args.port}...")

    # Try to connect to WebSocket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        result = sock.connect_ex((args.host, args.port))
        sock.close()

        if result == 0:
            print(f"✅ Bridge is running at {args.host}:{args.port}")
            print(f"   WebSocket: ws://{args.host}:{args.port}")
            return 0
        else:
            print(f"❌ Bridge is not responding at {args.host}:{args.port}")
            return 1
    except Exception as e:
        print(f"❌ Failed to check status: {e}")
        return 1


def run_config(args: argparse.Namespace, logger: logging.Logger) -> int:
    """Manage configuration."""
    if args.init:
        # Generate default config
        version = get_version()
        default_config = f"""# Agent ROS Bridge Configuration
# Generated by: agent-ros-bridge config --init

bridge:
  name: "Agent ROS Bridge"
  version: "{version}"

transports:
  websocket:
    enabled: true
    host: "0.0.0.0"
    port: 8765
  
  grpc:
    enabled: true
    host: "0.0.0.0"
    port: 50051
  
  mqtt:
    enabled: true
    host: "0.0.0.0"
    port: 1883

connectors:
  ros2:
    enabled: true
    domain_id: 0

safety:
  autonomous_mode: false
  human_in_the_loop: true
  shadow_mode_enabled: true
  min_confidence_for_auto: 0.95

logging:
  level: "INFO"
  format: "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
"""

        # Write config file
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with open(output_path, 'w') as f:
            f.write(default_config)

        print(f"✅ Default configuration written to: {args.output}")
        return 0

    if args.validate:
        # Validate config file
        try:
            from .gateway_v2.config import ConfigLoader
            ConfigLoader.from_file(args.validate)
            print(f"✅ Configuration file is valid: {args.validate}")
            return 0
        except Exception as e:
            print(f"❌ Configuration file is invalid: {e}")
            return 1

    # No action specified
    print("Usage: agent-ros-bridge config [--init | --validate <file>]")
    return 0


async def main_async() -> int:
    """Main async entry point."""
    parser = create_parser()
    args = parser.parse_args()

    # Setup logging
    log_level = "DEBUG" if args.debug else "INFO"
    logger = setup_logging(log_level)

    # Default command is "start"
    command = args.command or "start"

    try:
        if command == "start":
            return await run_gateway(args, logger)
        elif command == "dashboard":
            return run_dashboard(args, logger)
        elif command == "robot":
            return await run_robot_mode(args, logger)
        elif command == "status":
            return run_status(args, logger)
        elif command == "config":
            return run_config(args, logger)
        else:
            logger.error(f"Unknown command: {command}")
            return 1
    except KeyboardInterrupt:
        logger.info("Shutdown requested")
        return 0
    except Exception as e:
        logger.error(f"Error: {e}")
        if args.debug:
            import traceback
            traceback.print_exc()
        return 1


def main() -> int:
    """Main entry point."""
    try:
        return asyncio.run(main_async())
    except KeyboardInterrupt:
        print("\nShutdown requested")
        return 0


if __name__ == "__main__":
    sys.exit(main())
