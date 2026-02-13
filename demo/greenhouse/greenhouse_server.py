#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Greenhouse TCP Server - Demo application launcher

This launches the generic OpenClaw TCP Server with the greenhouse plugin loaded.
It demonstrates how to build application-specific functionality on top of the
generic ROS bridge.
"""
import sys
import os

# Add project root to path
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from openclaw_ros_bridge.communication.openclaw_tcp_server import openclaw_server
from demo.greenhouse.greenhouse_plugin import register_with_server
from openclaw_ros_bridge.base.logger import get_logger

logger = get_logger(__name__)


def main():
    """Start TCP server with greenhouse plugin"""
    import signal
    
    def signal_handler(sig, frame):
        logger.info("Shutting down greenhouse server...")
        openclaw_server.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Register greenhouse plugin with the server
    register_with_server(openclaw_server)
    
    # Start server
    if openclaw_server.start():
        logger.info("=" * 50)
        logger.info("Greenhouse Demo Server Running")
        logger.info("=" * 50)
        logger.info("Available commands:")
        logger.info("  - read_sensor (sensor: env)")
        logger.info("  - write_actuator (actuator: fan|valve, value: true|false)")
        logger.info("  - get_greenhouse_status")
        logger.info("  - get_status")
        logger.info("  - ping")
        logger.info("=" * 50)
        logger.info("Press Ctrl+C to stop")
        
        try:
            import time
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    else:
        logger.error("Failed to start server")
        sys.exit(1)


if __name__ == "__main__":
    main()
