#!/usr/bin/env python3
"""
JWT Client Example - Connect to Agent ROS Bridge

This script demonstrates how to:
1. Generate a JWT token
2. Connect to the bridge via WebSocket
3. Send commands to control the robot
"""

import asyncio
import json
import os
import sys

import websockets

# Add workspace to path
sys.path.insert(0, '/Users/webthree/.openclaw/workspace')

from agent_ros_bridge.gateway_v2.auth import AuthConfig, Authenticator


async def generate_token():
    """Generate a JWT token for authentication."""
    # Get secret from environment or use demo secret
    jwt_secret = os.getenv("JWT_SECRET", "demo-secret-for-testing")

    config = AuthConfig(
        jwt_secret=jwt_secret,
        jwt_expiry_hours=1
    )
    auth = Authenticator(config)

    token = auth.create_token(
        user_id="demo_client",
        roles=["robot_operator"],
        metadata={"source": "client_example"}
    )

    return token


async def connect_and_control():
    """Connect to bridge and control robot."""

    # Generate token
    print("🔐 Generating JWT token...")
    token = await generate_token()
    print(f"   Token: {token[:50]}...")
    print()

    # WebSocket URL
    ws_port = os.getenv("WEBSOCKET_PORT", "8766")
    uri = f"ws://localhost:{ws_port}?token={token}"

    print(f"🔗 Connecting to {uri[:50]}...")
    print()

    try:
        async with websockets.connect(uri) as ws:
            print("✅ Connected to Agent ROS Bridge!")
            print()

            # 1. List available robots
            print("🤖 Listing robots...")
            await ws.send(json.dumps({
                "command": "list_robots"
            }))
            response = await ws.recv()
            print(f"   Response: {response}")
            print()

            # 2. Get topics
            print("📡 Getting ROS topics...")
            await ws.send(json.dumps({
                "command": "get_topics"
            }))
            response = await ws.recv()
            data = json.loads(response)
            if data.get("topics"):
                for topic in data["topics"][:5]:
                    print(f"   • {topic['name']}")
            print()

            # 3. Subscribe to odometry
            print("📊 Subscribing to /odom...")
            await ws.send(json.dumps({
                "command": "subscribe",
                "topic": "/odom",
                "msg_type": "nav_msgs/Odometry"
            }))

            # Read a few messages
            for i in range(3):
                response = await ws.recv()
                data = json.loads(response)
                if "data" in data:
                    pose = data["data"].get("pose", {}).get("pose", {}).get("position", {})
                    print(f"   Position {i+1}: x={pose.get('x', 0):.3f}, y={pose.get('y', 0):.3f}")
            print()

            # 4. Send movement command
            print("🚀 Sending move command...")
            await ws.send(json.dumps({
                "command": "publish",
                "topic": "/cmd_vel",
                "msg_type": "geometry_msgs/Twist",
                "message": {
                    "linear": {"x": 0.2, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
                }
            }))
            response = await ws.recv()
            print(f"   Response: {response}")
            print()

            # Wait a bit
            await asyncio.sleep(2)

            # 5. Stop
            print("🛑 Stopping robot...")
            await ws.send(json.dumps({
                "command": "publish",
                "topic": "/cmd_vel",
                "msg_type": "geometry_msgs/Twist",
                "message": {
                    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
                }
            }))
            response = await ws.recv()
            print(f"   Response: {response}")

    except websockets.exceptions.InvalidStatusCode as e:
        print(f"❌ Connection failed: {e}")
        print("   Make sure the bridge is running:")
        print("   ./start_bridge.sh")
    except Exception as e:
        print(f"❌ Error: {e}")


if __name__ == "__main__":
    print("=" * 60)
    print("🔐 JWT Client Example")
    print("=" * 60)
    print()
    print("This example demonstrates:")
    print("  1. JWT token generation")
    print("  2. WebSocket connection with auth")
    print("  3. Sending commands to the bridge")
    print()

    # Check if bridge is running
    import subprocess
    result = subprocess.run(
        ["lsof", "-Pi", ":8766", "-sTCP:LISTEN"],
        capture_output=True,
        text=True
    )
    if not result.stdout.strip():
        print("⚠️  Bridge doesn't appear to be running on port 8766")
        print("   Start it first with: ./start_bridge.sh")
        print()
        sys.exit(1)

    asyncio.run(connect_and_control())

    print()
    print("=" * 60)
    print("✅ Demo complete!")
    print("=" * 60)
