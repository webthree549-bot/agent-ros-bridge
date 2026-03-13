#!/usr/bin/env python3
"""Simple WebSocket server for agent_ros_bridge"""

import asyncio
import sys

sys.path.insert(0, "/Users/webthree/.openclaw/workspace")

from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport


async def main():
    print("=" * 60)
    print("🚀 Agent ROS Bridge - WebSocket Server")
    print("=" * 60)

    bridge = Bridge()

    # Register WebSocket transport on port 8766 (no auth for testing)
    bridge.transport_manager.register(
        WebSocketTransport({"host": "0.0.0.0", "port": 8766, "auth": {"enabled": False}})
    )

    print("📡 WebSocket: ws://localhost:8766")
    print("Press Ctrl+C to stop")
    print("=" * 60)

    await bridge.start()

    # Keep running until interrupted
    try:
        while True:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        pass
    finally:
        await bridge.stop()


if __name__ == "__main__":
    asyncio.run(main())
