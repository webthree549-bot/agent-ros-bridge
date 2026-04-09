#!/usr/bin/env python3
"""Test Agent ROS Bridge Gateway commands."""
import asyncio
import json
import uuid
from datetime import UTC, datetime

import websockets


async def send_command(ws, action, params=None):
    """Send command and get response."""
    cmd = {
        "header": {
            "message_id": str(uuid.uuid4()),
            "timestamp": datetime.now(UTC).isoformat(),
            "source": "openclaw",
            "target": "ros2_jazzy"
        },
        "command": {
            "action": action,
            "parameters": params or {},
            "timeout_ms": 5000,
            "priority": 5
        }
    }
    await ws.send(json.dumps(cmd))
    print(f"📤 Sent: {action}")
    
    try:
        response = await asyncio.wait_for(ws.recv(), timeout=5)
        data = json.loads(response)
        print(f"📥 Response:\n{json.dumps(data, indent=2)}\n")
        return data
    except asyncio.TimeoutError:
        print("⏱️ Timeout (no response)\n")
        return None


async def main():
    uri = "ws://127.0.0.1:8765"
    print(f"Connecting to {uri}...\n")
    
    async with websockets.connect(uri) as ws:
        print("✅ Connected!\n")
        
        # Test 1: Discover
        print("=== Test 1: Discover ===")
        await send_command(ws, "discover")
        
        # Test 2: Fleet list
        print("=== Test 2: List Fleets ===")
        await send_command(ws, "fleet.list")
        
        # Test 3: Fleet robots
        print("=== Test 3: Fleet Robots ===")
        await send_command(ws, "fleet.robots", {"fleet": "default"})


if __name__ == "__main__":
    asyncio.run(main())
