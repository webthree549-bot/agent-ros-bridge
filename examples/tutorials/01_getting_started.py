#!/usr/bin/env python3
"""
Tutorial 1: Getting Started with Agent ROS Bridge

This tutorial demonstrates basic robot control using natural language.
"""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from agent_ros_bridge.gateway_v2.core import AgentROSBridge


async def tutorial_1_basic_commands():
    """Tutorial 1: Basic natural language commands."""
    print("=" * 60)
    print("Tutorial 1: Getting Started with Agent ROS Bridge")
    print("=" * 60)
    print()
    
    # Initialize bridge
    print("Step 1: Initialize the bridge")
    print("-" * 40)
    bridge = AgentROSBridge({
        "websocket_port": 8765,
        "jwt_secret": "demo-secret"
    })
    await bridge.start()
    print("✅ Bridge started on port 8765")
    print()
    
    # Example 1: Simple movement
    print("Step 2: Execute natural language commands")
    print("-" * 40)
    
    commands = [
        "move forward 1 meter",
        "turn left 90 degrees",
        "move to position x=2, y=3",
        "stop",
    ]
    
    for cmd in commands:
        print(f"\n📝 Command: '{cmd}'")
        try:
            result = await bridge.execute_nl(cmd)
            print(f"✅ Result: {result}")
        except Exception as e:
            print(f"❌ Error: {e}")
    
    print()
    print("=" * 60)
    print("Tutorial 1 Complete!")
    print("=" * 60)
    
    await bridge.stop()


async def tutorial_2_context_awareness():
    """Tutorial 2: Context-aware conversations."""
    print()
    print("=" * 60)
    print("Tutorial 2: Context-Aware Conversations")
    print("=" * 60)
    print()
    
    bridge = AgentROSBridge({
        "websocket_port": 8766,
        "jwt_secret": "demo-secret"
    })
    await bridge.start()
    
    # Simulate conversation with context
    session_id = "tutorial-session-001"
    
    conversation = [
        ("go to the kitchen", None),
        ("now go to the living room", None),  # Should understand "now"
        ("go back", None),  # Should remember "kitchen"
        ("what was my first command?", None),  # Should recall history
    ]
    
    print("Simulating context-aware conversation:")
    print("-" * 40)
    
    for cmd, robot_id in conversation:
        print(f"\n📝 User: '{cmd}'")
        try:
            result = await bridge.execute_nl(cmd, robot_id, session_id)
            print(f"🤖 Robot: {result}")
        except Exception as e:
            print(f"❌ Error: {e}")
    
    print()
    print("=" * 60)
    print("Tutorial 2 Complete!")
    print("=" * 60)
    
    await bridge.stop()


async def tutorial_3_fleet_management():
    """Tutorial 3: Multi-robot fleet management."""
    print()
    print("=" * 60)
    print("Tutorial 3: Fleet Management")
    print("=" * 60)
    print()
    
    bridge = AgentROSBridge({
        "websocket_port": 8767,
        "jwt_secret": "demo-secret"
    })
    await bridge.start()
    
    # Register mock robots
    print("Registering robots in fleet:")
    print("-" * 40)
    
    robots = [
        {"id": "robot-001", "name": "TurtleBot-1", "type": "turtlebot4", "battery": 85},
        {"id": "robot-002", "name": "TurtleBot-2", "type": "turtlebot4", "battery": 92},
        {"id": "robot-003", "name": "ArmBot-1", "type": "universal_robots", "battery": 78},
    ]
    
    for robot in robots:
        print(f"  ✅ Registered {robot['name']} ({robot['id']})")
    
    print()
    print("Fleet commands:")
    print("-" * 40)
    
    fleet_commands = [
        "all robots move to charging station",
        "robot with highest battery go to zone A",
        "find closest robot to position x=10, y=20",
        "status of all robots",
    ]
    
    for cmd in fleet_commands:
        print(f"\n📝 Command: '{cmd}'")
        try:
            result = await bridge.execute_nl(cmd)
            print(f"✅ Result: {result}")
        except Exception as e:
            print(f"❌ Error: {e}")
    
    print()
    print("=" * 60)
    print("Tutorial 3 Complete!")
    print("=" * 60)
    
    await bridge.stop()


async def main():
    """Run all tutorials."""
    print("\n" + "=" * 60)
    print("Agent ROS Bridge - Interactive Tutorials")
    print("=" * 60)
    print()
    
    try:
        await tutorial_1_basic_commands()
        await tutorial_2_context_awareness()
        await tutorial_3_fleet_management()
        
        print()
        print("=" * 60)
        print("🎉 All Tutorials Complete!")
        print("=" * 60)
        print()
        print("Next steps:")
        print("  1. Try the LangChain integration: examples/langchain_demo.py")
        print("  2. Try the MCP server: python -m agent_ros_bridge.frameworks.mcp.server")
        print("  3. Read the documentation: docs/")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Tutorials interrupted by user")
    except Exception as e:
        print(f"\n\n❌ Error: {e}")
        raise


if __name__ == "__main__":
    asyncio.run(main())
