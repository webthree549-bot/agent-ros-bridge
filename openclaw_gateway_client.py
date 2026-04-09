#!/usr/bin/env python3
"""Simple OpenClaw client for Agent ROS Bridge Gateway.

Direct WebSocket connection to ros2_jazzy container.
Usage: python openclaw_gateway_client.py
"""

import asyncio
import json
import uuid
from datetime import UTC, datetime

import websockets


class OpenClawGatewayClient:
    """Minimal client for OpenClaw to talk to ROS2 container."""

    def __init__(self, uri: str = "ws://localhost:8765"):
        self.uri = uri
        self.ws = None
        self.connected = False

    async def connect(self):
        """Connect to gateway."""
        try:
            self.ws = await websockets.connect(self.uri)
            self.connected = True
            print(f"✅ Connected to {self.uri}")
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False

    async def disconnect(self):
        """Disconnect from gateway."""
        if self.ws:
            await self.ws.close()
            self.connected = False
            print("Disconnected")

    async def send_command(self, action: str, parameters: dict = None, timeout_ms: int = 5000):
        """Send command to gateway."""
        if not self.connected:
            print("Not connected!")
            return None

        msg = {
            "header": {
                "message_id": str(uuid.uuid4()),
                "timestamp": datetime.now(UTC).isoformat(),
                "source": "openclaw",
                "target": "ros2_jazzy"
            },
            "command": {
                "action": action,
                "parameters": parameters or {},
                "timeout_ms": timeout_ms,
                "priority": 5
            }
        }

        await self.ws.send(json.dumps(msg))
        print(f"📤 Sent: {action}")

        # Wait for response
        try:
            response = await asyncio.wait_for(self.ws.recv(), timeout=timeout_ms/1000)
            data = json.loads(response)
            print(f"📥 Received: {json.dumps(data, indent=2)}")
            return data
        except asyncio.TimeoutError:
            print("⏱️ Timeout waiting for response")
            return None

    async def subscribe_telemetry(self, topic: str):
        """Subscribe to telemetry topic."""
        return await self.send_command("subscribe", {"topic": topic})


async def main():
    """Test connection to gateway."""
    client = OpenClawGatewayClient("ws://localhost:8765")

    if not await client.connect():
        return

    try:
        # Test 1: Get status
        print("\n--- Test 1: Get Status ---")
        await client.send_command("get_status")

        # Test 2: List robots
        print("\n--- Test 2: List Robots ---")
        await client.send_command("list_robots")

        # Test 3: Try navigation (will fail without ROS2, but shows protocol)
        print("\n--- Test 3: Navigate To ---")
        await client.send_command("navigate_to", {"x": 1.0, "y": 2.0, "theta": 0.0})

    except Exception as e:
        print(f"Error: {e}")
    finally:
        await client.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
