"""Example: Using Agent ROS Bridge with MCP (Claude Desktop).

This example shows how to start an MCP server for Claude Desktop integration.
"""

import asyncio
import os

from agent_ros_bridge import Bridge

os.environ["JWT_SECRET"] = "your-secret-key-here"


async def main():
    """Main example."""
    bridge = Bridge()

    # Get MCP server (stdio mode for Claude Desktop)
    mcp = bridge.get_mcp_server(mode="stdio")

    print("✅ MCP Server Ready")
    print("   Mode: stdio")
    print("   Claude Desktop can now control robots!")
    print("\n   Add this to your Claude Desktop config:")
    print("   {")
    print('     "mcpServers": {')
    print('       "agent-ros-bridge": {')
    print('         "command": "python",')
    print('         "args": ["-m", "agent_ros_bridge.integrations.mcp_transport"]')
    print("       }")
    print("     }")
    print("   }")

    # Start MCP server (blocks until stopped)
    print("\n🚀 Starting MCP server...")
    await mcp.start()


if __name__ == "__main__":
    asyncio.run(main())
