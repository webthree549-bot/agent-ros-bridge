#!/usr/bin/env python3
"""Example: OpenClaw Integration with Agent ROS Bridge.

This example demonstrates how to integrate Agent ROS Bridge with the
OpenClaw AI agent framework via ClawHub skills.

Usage:
    # Basic usage with simulated robot
    python openclaw_example.py

    # Show skill path
    python openclaw_example.py --skill-path

    # Package skill
    python openclaw_example.py --package
"""

import argparse
import asyncio
import os
import sys
from pathlib import Path

# Add parent to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport


async def basic_example():
    """Basic example showing OpenClaw adapter usage."""
    print("=" * 70)
    print("Agent ROS Bridge + OpenClaw Integration Example")
    print("=" * 70)

    # Create bridge
    bridge = Bridge()

    # Register WebSocket transport
    bridge.transport_manager.register(WebSocketTransport({"port": 8765}))

    # Get OpenClaw adapter
    adapter = bridge.get_openclaw_adapter()

    print("\n1. OpenClaw Skill Path:")
    print("-" * 40)
    skill_path = adapter.get_skill_path()
    if skill_path:
        print(f"  Skill location: {skill_path}")
        print(f"  SKILL.md exists: {(skill_path / 'SKILL.md').exists()}")
    else:
        print("  Skill path not found (expected in development)")

    print("\n2. Available OpenClaw Tools (Extension Mode):")
    print("-" * 40)
    tools = adapter.get_tools()
    for tool in tools[:10]:  # Show first 10
        dangerous = "⚠️" if tool.get("dangerous") else "  "
        print(f"  {dangerous} {tool['name']}")
    if len(tools) > 10:
        print(f"  ... and {len(tools) - 10} more tools")

    print("\n3. RosClaw-Compatible Tools:")
    print("-" * 40)
    rosclaw_tools = adapter.to_rosclaw_compatible_format()
    print(f"  Total RosClaw-compatible tools: {len(rosclaw_tools)}")
    for tool in rosclaw_tools[:5]:
        print(f"    - {tool['function']['name']}")

    # Start bridge
    print("\n4. Starting Bridge...")
    async with bridge.run():
        print("   Bridge running on ws://localhost:8765")

        print("\n5. OpenClaw Integration Summary:")
        print("-" * 40)
        print("  ✓ ClawHub skill available for distribution")
        print("  ✓ Extension mode with 17+ tools")
        print("  ✓ RosClaw-compatible interface")
        print("  ✓ ROS1 and ROS2 support")
        print("  ✓ Fleet management capabilities")
        print("  ✓ Safety features (emergency stop)")

        print("\n6. Usage with OpenClaw:")
        print("-" * 40)
        print("""
  # Option 1: ClawHub Skill (Recommended)
  # Package and upload to ClawHub:
  cd skills/agent-ros-bridge
  python scripts/package_skill.py
  # Upload dist/agent-ros-bridge.skill to ClawHub

  # Users install via:
  npx clawhub@latest install agent-ros-bridge

  # Then in OpenClaw:
  "Move forward 1 meter"
  "Navigate to the kitchen"
  "Check the battery"

  # Option 2: Extension Mode (Direct)
  from agent_ros_bridge import Bridge
  bridge = Bridge()
  adapter = bridge.get_openclaw_adapter()
  result = await adapter.execute_tool("ros2_publish", {...})
        """)

        print("\n" + "=" * 70)
        print("Example complete. Press Ctrl+C to exit.")
        print("=" * 70)

        # Keep running
        while bridge.running:
            await asyncio.sleep(1)


def show_skill_path():
    """Show the skill path for packaging."""
    bridge = Bridge()
    adapter = bridge.get_openclaw_adapter()

    skill_path = adapter.get_skill_path()
    if skill_path:
        print(f"OpenClaw skill path: {skill_path}")
        print("\nTo package:")
        print(f"  cd {skill_path}")
        print("  python scripts/package_skill.py")
    else:
        print("Skill path not found")
        sys.exit(1)


def package_skill():
    """Package the OpenClaw skill."""
    bridge = Bridge()
    adapter = bridge.get_openclaw_adapter()

    skill_path = adapter.get_skill_path()
    if not skill_path:
        print("Skill path not found")
        sys.exit(1)

    output_dir = Path(__file__).parent.parent.parent / "dist"
    output_dir.mkdir(exist_ok=True)

    try:
        output_file = adapter.package_skill(output_dir)
        print(f"Packaged skill: {output_file}")
        print(f"Size: {output_file.stat().st_size} bytes")

        # List contents
        import zipfile

        print("\nContents:")
        with zipfile.ZipFile(output_file, "r") as zf:
            for name in zf.namelist():
                print(f"  {name}")
    except Exception as e:
        print(f"Packaging failed: {e}")
        sys.exit(1)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="OpenClaw Integration Example")
    parser.add_argument("--skill-path", action="store_true", help="Show skill path")
    parser.add_argument("--package", action="store_true", help="Package skill")

    args = parser.parse_args()

    # Set JWT secret if not set
    if not os.environ.get("JWT_SECRET"):
        os.environ["JWT_SECRET"] = "example-secret-for-demo-only"

    if args.skill_path:
        show_skill_path()
    elif args.package:
        package_skill()
    else:
        try:
            asyncio.run(basic_example())
        except KeyboardInterrupt:
            print("\n\nExiting...")


if __name__ == "__main__":
    main()
