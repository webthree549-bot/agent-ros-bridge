"""MCP (Model Context Protocol) Server for Agent ROS Bridge.

This module provides an MCP server interface for Agent ROS Bridge,
allowing AI agents like Claude Desktop to control ROS robots through
standard MCP tools and resources.

Usage:
    python -m agent_ros_bridge.mcp_server

Or configure in Claude Desktop:
    {
      "mcpServers": {
        "ros": {
          "command": "python3",
          "args": ["-m", "agent_ros_bridge.mcp_server"]
        }
      }
    }
"""

import asyncio
import json
import os
from typing import Any, Dict, List, Optional
from pathlib import Path

try:
    from mcp.server import Server
    from mcp.types import Tool, Resource, TextContent
except ImportError:
    raise ImportError(
        "MCP not installed. Run: pip install agent-ros-bridge[mcp]"
    )

from .. import ROSBridge
from ..gateway_v2.transports.websocket import WebSocketTransport
from ..gateway_v2.transports.grpc import GRPCServer
from ..gateway_v2.connectors.ros2 import ROS2Connector


class ROSMCPBridge:
    """MCP server bridge for ROS.
    
    Exposes ROS actions as MCP tools and ROS topics as MCP resources.
    """
    
    def __init__(self, bridge: Optional[ROSBridge] = None):
        """Initialize MCP bridge.
        
        Args:
            bridge: Existing ROSBridge instance or None to create new
        """
        self.bridge = bridge or self._create_default_bridge()
        self.app = Server("agent-ros-bridge")
        self._setup_handlers()
    
    def _create_default_bridge(self) -> ROSBridge:
        """Create default ROS bridge with standard configuration."""
        bridge = ROSBridge(ros_version=2)
        
        # Register transports
        bridge.transport_manager.register(
            WebSocketTransport({
                'host': '0.0.0.0',
                'port': 8765,
                'auth': {'enabled': False}  # Auth handled at MCP layer
            })
        )
        
        bridge.transport_manager.register(
            GRPCServer({
                'host': '0.0.0.0',
                'port': 50051,
                'auth': {'enabled': False}
            })
        )
        
        # Register ROS2 connector
        bridge.connector_manager.register(
            ROS2Connector({'domain_id': 0})
        )
        
        return bridge
    
    def _setup_handlers(self):
        """Set up MCP request handlers."""
        
        @self.app.list_tools()
        async def list_tools() -> List[Tool]:
            """List available ROS actions as MCP tools."""
            tools = []
            
            # Get registered actions from bridge
            for action_name in self.bridge.get_registered_actions():
                tool = Tool(
                    name=f"ros_{action_name}",
                    description=f"Execute ROS action: {action_name}",
                    inputSchema={
                        "type": "object",
                        "properties": self._get_action_schema(action_name)
                    }
                )
                tools.append(tool)
            
            return tools
        
        @self.app.call_tool()
        async def call_tool(name: str, arguments: Dict[str, Any]) -> List[TextContent]:
            """Execute a ROS action via MCP tool call."""
            # Extract action name (remove ros_ prefix)
            action_name = name.replace("ros_", "")
            
            try:
                result = await self.bridge.call_action(action_name, **arguments)
                return [TextContent(type="text", text=json.dumps(result, indent=2))]
            except Exception as e:
                return [TextContent(type="text", text=f"Error: {str(e)}")]
        
        @self.app.list_resources()
        async def list_resources() -> List[Resource]:
            """List available ROS topics as MCP resources."""
            resources = []
            
            for topic in self.bridge.get_available_topics():
                resource = Resource(
                    uri=f"ros://{topic}",
                    name=topic,
                    mimeType="application/json"
                )
                resources.append(resource)
            
            return resources
        
        @self.app.read_resource()
        async def read_resource(uri: str) -> str:
            """Read current value of a ROS topic."""
            # Parse URI: ros://topic_name
            topic = uri.replace("ros://", "")
            
            try:
                data = await self.bridge.get_topic_data(topic)
                return json.dumps(data, indent=2)
            except Exception as e:
                return json.dumps({"error": str(e)})
    
    def _get_action_schema(self, action_name: str) -> Dict[str, Any]:
        """Get JSON schema for action parameters."""
        # Default schema - bridge should provide better introspection
        return {
            "parameters": {
                "type": "object",
                "description": f"Parameters for {action_name}"
            }
        }
    
    async def start(self):
        """Start MCP server."""
        # Start ROS bridge
        asyncio.create_task(self.bridge.start())
        
        # Start MCP server
        await self.app.run()


def main():
    """Main entry point for MCP server."""
    import sys
    
    print("=" * 60)
    print("🤖 Agent ROS Bridge - MCP Server")
    print("=" * 60)
    print()
    
    # Check JWT secret
    jwt_secret = os.environ.get('JWT_SECRET')
    if not jwt_secret:
        print("⚠️  Warning: JWT_SECRET not set. Using demo mode.")
        print("   Set JWT_SECRET for production use.")
        print()
    
    print("Starting MCP server...")
    print("Configure in Claude Desktop with:")
    print()
    print('  {')
    print('    "mcpServers": {')
    print('      "ros": {')
    print('        "command": "python3",')
    print('        "args": ["-m", "agent_ros_bridge.mcp_server"]')
    print('      }')
    print('    }')
    print('  }')
    print()
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    bridge = ROSMCPBridge()
    
    try:
        asyncio.run(bridge.start())
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")
        sys.exit(0)


if __name__ == "__main__":
    main()
