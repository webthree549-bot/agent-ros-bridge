"""Example: Using Agent ROS Bridge with AutoGPT.

This example shows how to expose bridge actions as AutoGPT commands.
"""

import asyncio
import os
from agent_ros_bridge import Bridge

os.environ["JWT_SECRET"] = "your-secret-key-here"


async def main():
    """Main example."""
    
    bridge = Bridge()
    
    # Get AutoGPT adapter
    adapter = bridge.get_autogpt_adapter()
    
    # Discover commands
    commands = adapter.get_commands()
    
    print("✅ AutoGPT Adapter Ready")
    print(f"   Available commands: {len(commands)}")
    
    for cmd in commands:
        print(f"   - {cmd['name']}: {cmd['description']}")
    
    # Example: Execute a command
    if commands:
        result = await adapter.execute_command(
            commands[0]['name'],
            param1="value1"
        )
        print(f"\n🤖 Command result: {result}")
    
    await bridge.stop()


if __name__ == "__main__":
    asyncio.run(main())
