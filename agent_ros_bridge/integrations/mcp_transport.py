"""MCP Server Transport - Model Context Protocol integration for gateway_v2."""

import json
import logging
import asyncio
from typing import Dict, List, Any, Optional

logger = logging.getLogger(__name__)


class MCPServerTransport:
    """MCP (Model Context Protocol) server transport.
    
    Implements MCP protocol for Claude Desktop and other MCP clients.
    
    Example:
        from agent_ros_bridge.gateway_v2.core import Bridge
        from agent_ros_bridge.integrations.mcp_transport import MCPServerTransport
        
        bridge = Bridge()
        mcp = MCPServerTransport(bridge)
        
        # Start MCP server
        await mcp.start()
    """
    
    def __init__(self, bridge, mode: str = "stdio"):
        self.bridge = bridge
        self.mode = mode  # "stdio" or "sse"
        self.tools: List[Dict] = []
        self.running = False
        logger.info(f"MCPServerTransport initialized (mode: {mode})")
    
    async def start(self):
        """Start MCP server."""
        self.running = True
        
        # Discover tools from bridge
        await self._discover_tools()
        
        if self.mode == "stdio":
            await self._run_stdio()
        elif self.mode == "sse":
            await self._run_sse()
        else:
            raise ValueError(f"Unknown mode: {self.mode}")
    
    async def _discover_tools(self):
        """Discover available tools from bridge."""
        if self.bridge and hasattr(self.bridge, 'get_actions'):
            actions = self.bridge.get_actions()
            for action in actions:
                self.tools.append({
                    "name": action,
                    "description": f"Execute {action} on ROS robot",
                    "inputSchema": {
                        "type": "object",
                        "properties": {}
                    }
                })
    
    async def _run_stdio(self):
        """Run in stdio mode (for Claude Desktop)."""
        import sys
        
        # Send initialization
        await self._send_message({
            "jsonrpc": "2.0",
            "id": 0,
            "result": {
                "protocolVersion": "2024-11-05",
                "capabilities": {
                    "tools": {}
                },
                "serverInfo": {
                    "name": "agent-ros-bridge",
                    "version": "0.5.0"
                }
            }
        })
        
        # Handle incoming messages
        while self.running:
            try:
                line = await asyncio.get_event_loop().run_in_executor(
                    None, sys.stdin.readline
                )
                if not line:
                    break
                
                message = json.loads(line)
                response = await self._handle_message(message)
                if response:
                    await self._send_message(response)
                    
            except json.JSONDecodeError:
                logger.error("Invalid JSON received")
            except Exception as e:
                logger.error(f"Error handling message: {e}")
    
    async def _run_sse(self):
        """Run in SSE mode (for web clients)."""
        # Implementation for Server-Sent Events
        # This would require an HTTP server
        logger.info("SSE mode not yet implemented")
    
    async def _handle_message(self, message: Dict) -> Optional[Dict]:
        """Handle incoming MCP message."""
        method = message.get("method")
        msg_id = message.get("id")
        
        if method == "tools/list":
            return {
                "jsonrpc": "2.0",
                "id": msg_id,
                "result": {"tools": self.tools}
            }
        
        elif method == "tools/call":
            params = message.get("params", {})
            tool_name = params.get("name")
            arguments = params.get("arguments", {})
            
            result = await self._execute_tool(tool_name, arguments)
            return {
                "jsonrpc": "2.0",
                "id": msg_id,
                "result": result
            }
        
        elif method == "initialize":
            return {
                "jsonrpc": "2.0",
                "id": msg_id,
                "result": {
                    "protocolVersion": "2024-11-05",
                    "capabilities": {"tools": {}},
                    "serverInfo": {
                        "name": "agent-ros-bridge",
                        "version": "0.5.0"
                    }
                }
            }
        
        return None
    
    async def _execute_tool(self, tool_name: str, arguments: Dict) -> Dict:
        """Execute a tool via bridge."""
        try:
            if not self.bridge:
                return {
                    "content": [{"type": "text", "text": "Bridge not available"}],
                    "isError": True
                }
            
            if hasattr(self.bridge, 'execute_action'):
                result = await self.bridge.execute_action(tool_name, arguments)
                return {
                    "content": [{"type": "text", "text": json.dumps(result)}],
                    "isError": False
                }
            else:
                return {
                    "content": [{"type": "text", "text": "Bridge doesn't support execute_action"}],
                    "isError": True
                }
                
        except Exception as e:
            return {
                "content": [{"type": "text", "text": str(e)}],
                "isError": True
            }
    
    async def _send_message(self, message: Dict):
        """Send message to client."""
        import sys
        print(json.dumps(message), flush=True)
    
    async def stop(self):
        """Stop MCP server."""
        self.running = False
        logger.info("MCPServerTransport stopped")
