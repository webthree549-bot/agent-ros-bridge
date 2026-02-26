#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Agent ROS Bridge - CLI Entry Point

Usage:
    python -m agent_ros_bridge [options]
    
Examples:
    python -m agent_ros_bridge                          # Start with default config
    python -m agent_ros_bridge --config config.yaml     # Start with custom config
    python -m agent_ros_bridge --port 8765              # Override WebSocket port
    python -m agent_ros_bridge --debug                  # Enable debug logging
"""

import argparse
import asyncio
import logging
import sys
import signal
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from agent_ros_bridge import Bridge, ConfigLoader
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport
from agent_ros_bridge.gateway_v2.transports.mqtt_transport import MQTTTransport
from agent_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector


def setup_logging(level: str = "INFO") -> None:
    """Setup logging configuration"""
    logging.basicConfig(
        level=getattr(logging, level.upper()),
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )


def parse_args() -> argparse.Namespace:
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        prog="agent_ros_bridge",
        description="Universal ROS1/ROS2 bridge for AI agents to control robots",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                              # Start with default config
  %(prog)s --config config.yaml         # Start with custom config
  %(prog)s --port 8765                  # Override WebSocket port
  %(prog)s --debug                      # Enable debug logging
  %(prog)s --config config.yaml --port 8765 --debug
        """
    )
    
    parser.add_argument(
        "--config", "-c",
        type=str,
        help="Path to configuration file (YAML or JSON)"
    )
    
    parser.add_argument(
        "--port", "-p",
        type=int,
        default=None,
        help="WebSocket port (overrides config file)"
    )
    
    parser.add_argument(
        "--grpc-port",
        type=int,
        default=None,
        help="gRPC port (overrides config file)"
    )
    
    parser.add_argument(
        "--host",
        type=str,
        default=None,
        help="Host to bind to (default: 0.0.0.0)"
    )
    
    parser.add_argument(
        "--debug", "-d",
        action="store_true",
        help="Enable debug logging"
    )
    
    parser.add_argument(
        "--version", "-v",
        action="version",
        version=f"%(prog)s 0.5.0"
    )
    
    return parser.parse_args()


async def main() -> int:
    """Main entry point"""
    args = parse_args()
    
    # Setup logging
    log_level = "DEBUG" if args.debug else "INFO"
    setup_logging(log_level)
    logger = logging.getLogger("agent_ros_bridge")
    
    logger.info("=" * 60)
    logger.info("Agent ROS Bridge v0.5.0")
    logger.info("Universal ROS1/ROS2 bridge for AI agents")
    logger.info("=" * 60)
    
    # Load configuration
    try:
        config = ConfigLoader.from_file_or_env(args.config)
        logger.info(f"Configuration loaded from: {args.config or 'defaults'}")
    except Exception as e:
        logger.error(f"Failed to load configuration: {e}")
        return 1
    
    # Apply command line overrides
    if args.port:
        config.transports["websocket"].port = args.port
        logger.info(f"WebSocket port overridden: {args.port}")
    
    if args.grpc_port:
        config.transports["grpc"].port = args.grpc_port
        logger.info(f"gRPC port overridden: {args.grpc_port}")
    
    if args.host:
        for transport in config.transports.values():
            transport.host = args.host
        logger.info(f"Host overridden: {args.host}")
    
    # Create bridge
    bridge = Bridge(config)
    
    # Register transports based on config
    if config.transports.get("websocket") and config.transports["websocket"].enabled:
        ws_config = config.transports["websocket"]
        if ws_config.port > 0:
            transport = WebSocketTransport("websocket", {
                "host": ws_config.host,
                "port": ws_config.port,
                "tls_cert": ws_config.tls_cert,
                "tls_key": ws_config.tls_key,
                **ws_config.options
            })
            bridge.transport_manager.register(transport)
            logger.info(f"WebSocket transport registered on {ws_config.host}:{ws_config.port}")
    
    if config.transports.get("grpc") and config.transports["grpc"].enabled:
        grpc_config = config.transports["grpc"]
        if grpc_config.port > 0:
            transport = GRPCTransport("grpc", {
                "host": grpc_config.host,
                "port": grpc_config.port,
                **grpc_config.options
            })
            bridge.transport_manager.register(transport)
            logger.info(f"gRPC transport registered on {grpc_config.host}:{grpc_config.port}")
    
    if config.transports.get("mqtt") and config.transports["mqtt"].enabled:
        mqtt_config = config.transports["mqtt"]
        if mqtt_config.port > 0:
            transport = MQTTTransport("mqtt", {
                "host": mqtt_config.host,
                "port": mqtt_config.port,
                **mqtt_config.options
            })
            bridge.transport_manager.register(transport)
            logger.info(f"MQTT transport registered on {mqtt_config.host}:{mqtt_config.port}")
    
    # Register ROS2 connector if enabled
    if config.connectors.get("ros2") and config.connectors["ros2"].enabled:
        connector = ROS2Connector("ros2", config.connectors["ros2"].options)
        bridge.connector_registry.register(connector)
        logger.info("ROS2 connector registered")
    
    # Setup signal handlers for graceful shutdown
    loop = asyncio.get_event_loop()
    
    def signal_handler(sig):
        logger.info(f"Received signal {sig.name}, shutting down...")
        asyncio.create_task(bridge.stop())
    
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda s=sig: signal_handler(s))
    
    # Start bridge
    try:
        await bridge.start()
        logger.info("Bridge started successfully")
        logger.info("Press Ctrl+C to stop")
        
        # Keep running until stopped
        while bridge.running:
            await asyncio.sleep(1)
            
    except Exception as e:
        logger.error(f"Bridge error: {e}")
        return 1
    finally:
        await bridge.stop()
    
    logger.info("Bridge stopped")
    return 0


if __name__ == "__main__":
    try:
        exit_code = asyncio.run(main())
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\nShutdown requested")
        sys.exit(0)
