#!/usr/bin/env python3
"""
Send navigation goals to the TurtleBot3 robot via Agent ROS Bridge.

Usage:
    python3 send_nav_goal.py --x 1.0 --y 1.0 --theta 0.0
    python3 send_nav_goal.py --forward 1.0
    python3 send_nav_goal.py --rotate 90
"""

import argparse
import asyncio
import json

import websockets


async def send_navigation_goal(
    x: float, y: float, theta: float, bridge_url: str = "ws://localhost:8766"
):
    """Send a navigation goal to the robot."""

    print(f"Connecting to Agent ROS Bridge at {bridge_url}...")

    try:
        async with websockets.connect(bridge_url) as ws:
            print("✅ Connected to bridge")

            # Authenticate (using test token for simulation)
            auth_msg = {"type": "auth", "token": "test-secret-for-simulation"}
            await ws.send(json.dumps(auth_msg))
            print("🔐 Authenticated")

            # Discover available robots
            discover_msg = {"type": "command", "command": {"action": "discover"}}
            await ws.send(json.dumps(discover_msg))

            try:
                response = await asyncio.wait_for(ws.recv(), timeout=5.0)
                print(f"📡 Discovery response: {response[:200]}...")
            except TimeoutError:
                print("⚠️  No discovery response (may be empty)")

            # Send navigation goal
            nav_msg = {
                "type": "command",
                "command": {
                    "action": "robot.execute",
                    "parameters": {
                        "robot_id": "turtlebot3",
                        "command": {
                            "action": "navigate_to_pose",
                            "parameters": {"x": x, "y": y, "theta": theta},
                        },
                    },
                },
            }

            print(f"🚀 Sending navigation goal: x={x}, y={y}, theta={theta}")
            await ws.send(json.dumps(nav_msg))

            # Wait for response
            try:
                response = await asyncio.wait_for(ws.recv(), timeout=10.0)
                print(f"✅ Response: {response}")
            except TimeoutError:
                print("⏱️  No response received (command may still be executing)")

            print("🎯 Navigation goal sent!")

    except websockets.exceptions.ConnectionRefused:
        print(f"❌ Could not connect to {bridge_url}")
        print("   Make sure the Agent ROS Bridge is running:")
        print("   ./simulation-environment.sh start-bridge")
    except Exception as e:
        print(f"❌ Error: {e}")


async def send_simple_command(
    action: str, parameters: dict = None, bridge_url: str = "ws://localhost:8766"
):
    """Send a simple command to the robot."""

    print(f"Connecting to Agent ROS Bridge at {bridge_url}...")

    try:
        async with websockets.connect(bridge_url) as ws:
            print("✅ Connected to bridge")

            # Authenticate
            auth_msg = {"type": "auth", "token": "test-secret-for-simulation"}
            await ws.send(json.dumps(auth_msg))

            # Send command
            cmd = {"type": "command", "command": {"action": action, "parameters": parameters or {}}}

            print(f"🚀 Sending command: {action}")
            await ws.send(json.dumps(cmd))

            try:
                response = await asyncio.wait_for(ws.recv(), timeout=5.0)
                print(f"✅ Response: {response}")
            except TimeoutError:
                print("⏱️  No response received")

    except Exception as e:
        print(f"❌ Error: {e}")


def main():
    parser = argparse.ArgumentParser(description="Send navigation goals to TurtleBot3")
    parser.add_argument("--x", type=float, default=1.0, help="X coordinate (meters)")
    parser.add_argument("--y", type=float, default=1.0, help="Y coordinate (meters)")
    parser.add_argument("--theta", type=float, default=0.0, help="Rotation (radians)")
    parser.add_argument("--forward", type=float, help="Move forward by N meters")
    parser.add_argument("--rotate", type=float, help="Rotate by N degrees")
    parser.add_argument("--bridge", default="ws://localhost:8766", help="Bridge WebSocket URL")
    parser.add_argument("--discover", action="store_true", help="Just discover robots")

    args = parser.parse_args()

    if args.discover:
        # Just discover
        asyncio.run(send_simple_command("discover", bridge_url=args.bridge))
    elif args.forward:
        # Move forward
        asyncio.run(send_simple_command("move_forward", {"distance": args.forward}, args.bridge))
    elif args.rotate:
        # Rotate
        radians = args.rotate * 3.14159 / 180
        asyncio.run(send_simple_command("rotate", {"angle": radians}, args.bridge))
    else:
        # Send navigation goal
        asyncio.run(send_navigation_goal(args.x, args.y, args.theta, args.bridge))


if __name__ == "__main__":
    main()
