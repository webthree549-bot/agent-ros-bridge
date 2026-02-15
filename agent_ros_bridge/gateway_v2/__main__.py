#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Agent ROS Bridge - Main Entry Point

Universal ROS bridge with multi-protocol support.
"""

import asyncio
import argparse
import logging
import sys
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent.parent.parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from agent_ros_bridge.gateway_v2.core import Bridge
from agent_ros_bridge.gateway_v2.config import ConfigLoader, BridgeConfig
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport
from agent_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector
from agent_ros_bridge.gateway_v2.plugins.greenhouse_plugin import GreenhousePlugin

logger = logging.getLogger("agent_ros_bridge")


def setup_logging(log_level: str = "INFO"):
    """Configure logging"""
    logging.basicConfig(
        level=getattr(logging, log_level.upper()),
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout)
        ]
    )


def create_bridge_from_config(config: BridgeConfig) -> Bridge:
    """Create and configure bridge from config"""
    bridge = Bridge(config.__dict__)
    
    # Register transports
    for name, transport_cfg in config.transports.items():
        if not transport_cfg.enabled or transport_cfg.port == 0:
            continue
        
        transport_config = {
            "host": transport_cfg.host,
            "port": transport_cfg.port,
            "tls_cert": transport_cfg.tls_cert,
            "tls_key": transport_cfg.tls_key,
            **transport_cfg.options
        }
        
        if name == "websocket":
            transport = WebSocketTransport(transport_config)
            bridge.transport_manager.register(transport)
            logger.info(f"Registered WebSocket transport on port {transport_cfg.port}")
        
        elif name == "grpc":
            transport = GRPCTransport(transport_config)
            bridge.transport_manager.register(transport)
            logger.info(f"Registered gRPC transport on port {transport_cfg.port}")
        
        # Add more transports as needed
    
    # Register connectors
    for name, connector_cfg in config.connectors.items():
        if not connector_cfg.enabled:
            continue
        
        if name == "ros2":
            domain_id = connector_cfg.options.get("domain_id", 0)
            connector = ROS2Connector(domain_id=domain_id)
            bridge.connector_registry.register(connector)
            logger.info(f"Registered ROS2 connector (domain {domain_id})")
        
        # Add more connectors as needed
    
    # Load plugins
    for plugin_cfg in config.plugins:
        if not plugin_cfg.enabled:
            continue
        
        if plugin_cfg.name == "greenhouse":
            plugin = GreenhousePlugin()
            asyncio.create_task(bridge.plugin_manager.load_plugin(plugin))
            logger.info(f"Loaded greenhouse plugin")
    
    return bridge


async def run_bridge(config: BridgeConfig):
    """Run the bridge"""
    setup_logging(config.log_level)
    
    logger.info("=" * 60)
    logger.info("  Agent ROS Bridge")
    logger.info("  Universal ROS Bridge")
    logger.info("=" * 60)
    
    # Create bridge
    bridge = create_bridge_from_config(config)
    
    # Start
    async with bridge.run():
        logger.info("")
        logger.info("Bridge is running!")
        logger.info("")
        
        # Print connection info
        for name, transport in bridge.transport_manager.transports.items():
            if transport.running:
                logger.info(f"  {name}: Available")
        
        logger.info("")
        logger.info("Press Ctrl+C to stop")
        logger.info("")
        
        # Run forever
        try:
            while bridge.running:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            pass


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="Agent ROS Bridge - Universal ROS Bridge",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run with default configuration
  %(prog)s

  # Run with custom config file
  %(prog)s --config ./my_config.yaml

  # Run with specific transports
  %(prog)s --websocket-port 8765 --grpc-port 50051

  # Quick demo mode
  %(prog)s --demo

Environment Variables:
  AGENT_ROS_BRIDGE_CONFIG       Path to config file
  AGENT_ROS_BRIDGE_LOG_LEVEL    Logging level (DEBUG, INFO, WARNING, ERROR)
  AGENT_ROS_BRIDGE_WEBSOCKET_PORT  WebSocket port
  AGENT_ROS_BRIDGE_GRPC_PORT    gRPC port
  AGENT_ROS_BRIDGE_JWT_SECRET   JWT secret for authentication
        """
    )
    
    parser.add_argument(
        "--config", "-c",
        help="Path to configuration file (YAML or JSON)"
    )
    
    parser.add_argument(
        "--log-level",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level"
    )
    
    parser.add_argument(
        "--websocket-port",
        type=int,
        help="WebSocket port (overrides config)"
    )
    
    parser.add_argument(
        "--grpc-port",
        type=int,
        help="gRPC port (overrides config)"
    )
    
    parser.add_argument(
        "--tcp-port",
        type=int,
        help="TCP socket port (overrides config)"
    )
    
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Run in demo mode (WebSocket + greenhouse plugin)"
    )
    
    parser.add_argument(
        "--version", "-v",
        action="version",
        version="Agent ROS Bridge 0.3.2"
    )
    
    args = parser.parse_args()
    
    # Load configuration
    if args.demo:
        # Demo mode - simple configuration
        config = BridgeConfig()
        config.name = "demo_bridge"
        config.transports["websocket"].port = 8765
    else:
        config = ConfigLoader.from_file_or_env(args.config)
    
    # Apply command-line overrides
    if args.log_level:
        config.log_level = args.log_level
    
    if args.websocket_port is not None:
        config.transports["websocket"].port = args.websocket_port
    
    if args.grpc_port is not None:
        config.transports["grpc"].port = args.grpc_port
    
    if args.tcp_port is not None:
        config.transports["tcp"].port = args.tcp_port
    
    # Run bridge
    try:
        asyncio.run(run_bridge(config))
    except KeyboardInterrupt:
        logger.info("\nShutdown requested")
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
