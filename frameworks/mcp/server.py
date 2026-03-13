"""MCP (Model Context Protocol) server for Agent ROS Bridge.

Provides MCP tools for Claude and other MCP-compatible clients.
"""

import asyncio
import json
from typing import Any

from mcp.server import Server
from mcp.types import TextContent, Tool


class AgentROSMCPBridge:
    """MCP bridge for Agent ROS.

    Provides MCP tools for robot control that can be used by Claude Desktop
    and other MCP-compatible clients.
    """

    def __init__(self, bridge_client):
        self.bridge_client = bridge_client
        self.server = Server("agent-ros-bridge")
        self._setup_tools()

    def _setup_tools(self):
        """Register MCP tools."""

        @self.server.list_tools()
        async def list_tools() -> list[Tool]:
            """List available tools."""
            return [
                Tool(
                    name="ros2_publish",
                    description="Publish a message to a ROS2 topic",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "topic": {
                                "type": "string",
                                "description": "ROS topic name (e.g., '/cmd_vel')",
                            },
                            "message_type": {
                                "type": "string",
                                "description": "ROS message type (e.g., 'geometry_msgs/Twist')",
                            },
                            "data": {
                                "type": "object",
                                "description": "Message data as JSON object",
                            },
                        },
                        "required": ["topic", "message_type", "data"],
                    },
                ),
                Tool(
                    name="ros2_subscribe",
                    description="Subscribe to a ROS2 topic and get latest message",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "topic": {"type": "string", "description": "ROS topic name"},
                            "message_type": {"type": "string", "description": "ROS message type"},
                            "timeout": {
                                "type": "number",
                                "description": "Timeout in seconds",
                                "default": 5.0,
                            },
                        },
                        "required": ["topic", "message_type"],
                    },
                ),
                Tool(
                    name="ros2_action",
                    description="Execute a ROS2 action",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "action_name": {"type": "string", "description": "Action server name"},
                            "action_type": {"type": "string", "description": "Action type"},
                            "goal": {"type": "object", "description": "Goal parameters"},
                            "timeout": {
                                "type": "number",
                                "description": "Timeout in seconds",
                                "default": 30.0,
                            },
                        },
                        "required": ["action_name", "action_type", "goal"],
                    },
                ),
                Tool(
                    name="robot_command",
                    description="Execute a natural language robot command",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "command": {
                                "type": "string",
                                "description": "Natural language command (e.g., 'move forward 1 meter')",
                            },
                            "robot_id": {
                                "type": "string",
                                "description": "Optional robot ID",
                                "default": None,
                            },
                        },
                        "required": ["command"],
                    },
                ),
                Tool(
                    name="list_robots",
                    description="List all connected robots",
                    inputSchema={"type": "object", "properties": {}},
                ),
                Tool(
                    name="get_robot_info",
                    description="Get information about a specific robot",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "robot_id": {"type": "string", "description": "Robot identifier"}
                        },
                        "required": ["robot_id"],
                    },
                ),
            ]

        @self.server.call_tool()
        async def call_tool(name: str, arguments: dict[str, Any]) -> list[TextContent]:
            """Handle tool calls."""
            try:
                if name == "ros2_publish":
                    result = self.bridge_client.publish(
                        arguments["topic"], arguments["message_type"], arguments["data"]
                    )
                    return [TextContent(type="text", text=json.dumps(result, indent=2))]

                elif name == "ros2_subscribe":
                    result = self.bridge_client.subscribe(
                        arguments["topic"], arguments["message_type"], arguments.get("timeout", 5.0)
                    )
                    return [TextContent(type="text", text=json.dumps(result, indent=2))]

                elif name == "ros2_action":
                    result = self.bridge_client.execute_action(
                        arguments["action_name"],
                        arguments["action_type"],
                        arguments["goal"],
                        arguments.get("timeout", 30.0),
                    )
                    return [TextContent(type="text", text=json.dumps(result, indent=2))]

                elif name == "robot_command":
                    result = self.bridge_client.execute_natural_language(
                        arguments["command"], arguments.get("robot_id")
                    )
                    return [TextContent(type="text", text=json.dumps(result, indent=2))]

                elif name == "list_robots":
                    result = self.bridge_client.list_robots()
                    return [TextContent(type="text", text=json.dumps(result, indent=2))]

                elif name == "get_robot_info":
                    result = self.bridge_client.get_robot_info(arguments["robot_id"])
                    return [TextContent(type="text", text=json.dumps(result, indent=2))]

                else:
                    return [TextContent(type="text", text=f"Unknown tool: {name}")]

            except Exception as e:
                return [TextContent(type="text", text=f"Error: {str(e)}")]

    async def run(self):
        """Run the MCP server."""
        from mcp.server.stdio import stdio_server

        async with stdio_server() as (read_stream, write_stream):
            await self.server.run(
                read_stream, write_stream, self.server.create_initialization_options()
            )


def main():
    """Main entry point for MCP server."""
    import sys

    from agent_ros_bridge.frameworks.langchain import AgentROSBridgeClient

    # Get configuration from environment or arguments
    bridge_url = "http://localhost:8765"
    token = None

    # Parse arguments
    if len(sys.argv) > 1:
        bridge_url = sys.argv[1]
    if len(sys.argv) > 2:
        token = sys.argv[2]

    # Create client
    client = AgentROSBridgeClient(bridge_url, token)

    # Create and run MCP bridge
    bridge = AgentROSMCPBridge(client)
    asyncio.run(bridge.run())


if __name__ == "__main__":
    main()
