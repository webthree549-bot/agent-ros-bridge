"""
MCP (Model Context Protocol) Adapter for Agent ROS Bridge

Enables integration with MCP-compatible LLMs (Claude, GPT, etc.)
Competes with ros-mcp-server while maintaining our unique features.

MCP: https://modelcontextprotocol.io/
"""

import json
import asyncio
from typing import Any, Dict, List, Optional, Callable
from dataclasses import dataclass
from abc import ABC, abstractmethod


@dataclass
class MCPTool:
    """MCP Tool definition"""
    name: str
    description: str
    input_schema: Dict[str, Any]
    handler: Callable


class MCPAdapter:
    """
    Model Context Protocol adapter for Agent ROS Bridge.
    
    Allows MCP-compatible clients (Claude Desktop, etc.) to control
    robots through Agent ROS Bridge while maintaining our safety
    and human-in-the-loop features.
    
    Example:
        adapter = MCPAdapter(agent)
        await adapter.serve()  # Start MCP server
    """
    
    def __init__(self, robot_agent: 'RobotAgent'):
        """
        Initialize MCP adapter.
        
        Args:
            robot_agent: Configured RobotAgent instance
        """
        self.robot_agent = robot_agent
        self.tools: Dict[str, MCPTool] = {}
        self._register_default_tools()
    
    def _register_default_tools(self):
        """Register default MCP tools"""
        self.register_tool(
            name="execute_robot_command",
            description="Execute a natural language command on the robot",
            input_schema={
                "type": "object",
                "properties": {
                    "command": {
                        "type": "string",
                        "description": "Natural language command (e.g., 'go to the kitchen')"
                    },
                    "require_confirmation": {
                        "type": "boolean",
                        "description": "Whether to require human approval",
                        "default": True
                    }
                },
                "required": ["command"]
            },
            handler=self._handle_execute_command
        )
        
        self.register_tool(
            name="get_robot_status",
            description="Get current robot status and observations",
            input_schema={
                "type": "object",
                "properties": {}
            },
            handler=self._handle_get_status
        )
        
        self.register_tool(
            name="list_robot_capabilities",
            description="List available robot capabilities",
            input_schema={
                "type": "object",
                "properties": {}
            },
            handler=self._handle_list_capabilities
        )
        
        self.register_tool(
            name="navigate_to",
            description="Navigate robot to a location",
            input_schema={
                "type": "object",
                "properties": {
                    "location": {
                        "type": "string",
                        "description": "Target location (e.g., 'kitchen', 'office')"
                    },
                    "coordinates": {
                        "type": "array",
                        "description": "Optional [x, y] coordinates",
                        "items": {"type": "number"}
                    }
                },
                "required": ["location"]
            },
            handler=self._handle_navigate
        )
        
        self.register_tool(
            name="pick_object",
            description="Pick up an object",
            input_schema={
                "type": "object",
                "properties": {
                    "object": {
                        "type": "string",
                        "description": "Object to pick up (e.g., 'red cup', 'book')"
                    },
                    "location": {
                        "type": "string",
                        "description": "Optional location hint"
                    }
                },
                "required": ["object"]
            },
            handler=self._handle_pick_object
        )
        
        self.register_tool(
            name="move_manipulator",
            description="Move robot arm to position",
            input_schema={
                "type": "object",
                "properties": {
                    "x": {"type": "number", "description": "X coordinate"},
                    "y": {"type": "number", "description": "Y coordinate"},
                    "z": {"type": "number", "description": "Z coordinate"},
                },
                "required": ["x", "y", "z"]
            },
            handler=self._handle_move_arm
        )
        
        self.register_tool(
            name="emergency_stop",
            description="Immediately stop the robot",
            input_schema={
                "type": "object",
                "properties": {}
            },
            handler=self._handle_emergency_stop
        )
    
    def register_tool(
        self,
        name: str,
        description: str,
        input_schema: Dict[str, Any],
        handler: Callable
    ):
        """Register a new MCP tool"""
        self.tools[name] = MCPTool(
            name=name,
            description=description,
            input_schema=input_schema,
            handler=handler
        )
    
    def get_tools(self) -> List[Dict[str, Any]]:
        """Get list of tools in MCP format"""
        return [
            {
                "name": tool.name,
                "description": tool.description,
                "inputSchema": tool.input_schema
            }
            for tool in self.tools.values()
        ]
    
    async def call_tool(self, name: str, arguments: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute an MCP tool call.
        
        Args:
            name: Tool name
            arguments: Tool arguments
            
        Returns:
            MCP-compliant result
        """
        if name not in self.tools:
            return {
                "content": [
                    {
                        "type": "text",
                        "text": f"Error: Unknown tool '{name}'"
                    }
                ],
                "isError": True
            }
        
        tool = self.tools[name]
        
        try:
            result = await tool.handler(arguments)
            return {
                "content": [
                    {
                        "type": "text",
                        "text": json.dumps(result, indent=2)
                    }
                ],
                "isError": False
            }
        except Exception as e:
            return {
                "content": [
                    {
                        "type": "text",
                        "text": f"Error executing tool: {str(e)}"
                    }
                ],
                "isError": True
            }
    
    # Tool handlers
    
    async def _handle_execute_command(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle execute_robot_command tool"""
        command = args.get("command", "")
        require_confirmation = args.get("require_confirmation", True)
        
        # Temporarily override confirmation setting
        original_setting = self.robot_agent.require_confirmation
        self.robot_agent.require_confirmation = require_confirmation
        
        try:
            result = self.robot_agent.execute(command)
            
            return {
                "success": result.success,
                "command": result.task,
                "ai_confidence": result.ai_confidence,
                "steps_executed": len(result.steps),
                "human_approvals": result.human_approvals,
                "safety_violations": result.safety_violations,
                "duration_seconds": result.duration_seconds,
                "message": result.message
            }
        finally:
            self.robot_agent.require_confirmation = original_setting
    
    async def _handle_get_status(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle get_robot_status tool"""
        obs = self.robot_agent.observe()
        device = self.robot_agent.device
        
        return {
            "device_id": self.robot_agent.device_id,
            "device_type": self.robot_agent.device_type,
            "position": obs.robot_position,
            "battery": obs.battery_level,
            "nearby_objects": obs.nearby_objects,
            "capabilities": [
                cap.name for cap in device.get_capabilities()
            ] if device else []
        }
    
    async def _handle_list_capabilities(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle list_robot_capabilities tool"""
        device = self.robot_agent.device
        
        if not device:
            return {"capabilities": []}
        
        caps = device.get_capabilities()
        return {
            "capabilities": [
                {
                    "name": cap.name,
                    "description": cap.description,
                    "safety_critical": cap.safety_critical
                }
                for cap in caps
            ]
        }
    
    async def _handle_navigate(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle navigate_to tool"""
        location = args.get("location", "")
        coordinates = args.get("coordinates")
        
        command = f"Navigate to {location}"
        if coordinates:
            command += f" at coordinates {coordinates}"
        
        result = self.robot_agent.execute(command)
        
        return {
            "success": result.success,
            "location": location,
            "coordinates": coordinates,
            "message": result.message,
            "human_approvals_required": result.human_approvals > 0
        }
    
    async def _handle_pick_object(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle pick_object tool"""
        obj = args.get("object", "")
        location = args.get("location", "")
        
        command = f"Pick up the {obj}"
        if location:
            command += f" from {location}"
        
        result = self.robot_agent.execute(command)
        
        return {
            "success": result.success,
            "object": obj,
            "message": result.message,
            "safety_violations": result.safety_violations
        }
    
    async def _handle_move_arm(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle move_manipulator tool"""
        x, y, z = args.get("x", 0), args.get("y", 0), args.get("z", 0)
        
        command = f"Move arm to position ({x}, {y}, {z})"
        result = self.robot_agent.execute(command)
        
        return {
            "success": result.success,
            "position": [x, y, z],
            "message": result.message
        }
    
    async def _handle_emergency_stop(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle emergency_stop tool"""
        # Execute stop capability if available
        device = self.robot_agent.device
        
        if device and device.has_capability("stop"):
            result = device.execute_capability("stop", {})
            return {
                "success": result.get("success", True),
                "message": "Emergency stop executed",
                "robot_stopped": True
            }
        
        return {
            "success": False,
            "message": "Stop capability not available",
            "robot_stopped": False
        }
    
    # Server implementations
    
    async def serve_stdio(self):
        """
        Serve MCP over stdio (for Claude Desktop, etc.)
        
        Usage:
            adapter = MCPAdapter(agent)
            asyncio.run(adapter.serve_stdio())
        """
        import sys
        
        while True:
            try:
                line = await asyncio.get_event_loop().run_in_executor(
                    None, sys.stdin.readline
                )
                
                if not line:
                    break
                
                request = json.loads(line)
                response = await self._handle_request(request)
                
                print(json.dumps(response), flush=True)
                
            except json.JSONDecodeError:
                print(json.dumps({
                    "jsonrpc": "2.0",
                    "error": {"code": -32700, "message": "Parse error"},
                    "id": None
                }), flush=True)
            except Exception as e:
                print(json.dumps({
                    "jsonrpc": "2.0",
                    "error": {"code": -32603, "message": str(e)},
                    "id": None
                }), flush=True)
    
    async def _handle_request(self, request: Dict[str, Any]) -> Dict[str, Any]:
        """Handle MCP JSON-RPC request"""
        method = request.get("method", "")
        params = request.get("params", {})
        req_id = request.get("id")
        
        if method == "initialize":
            return {
                "jsonrpc": "2.0",
                "result": {
                    "protocolVersion": "2024-11-05",
                    "capabilities": {
                        "tools": {}
                    },
                    "serverInfo": {
                        "name": "agent-ros-bridge-mcp",
                        "version": "0.6.4"
                    }
                },
                "id": req_id
            }
        
        elif method == "tools/list":
            return {
                "jsonrpc": "2.0",
                "result": {"tools": self.get_tools()},
                "id": req_id
            }
        
        elif method == "tools/call":
            name = params.get("name", "")
            arguments = params.get("arguments", {})
            result = await self.call_tool(name, arguments)
            return {
                "jsonrpc": "2.0",
                "result": result,
                "id": req_id
            }
        
        else:
            return {
                "jsonrpc": "2.0",
                "error": {
                    "code": -32601,
                    "message": f"Method not found: {method}"
                },
                "id": req_id
            }


class ClaudeDesktopIntegration:
    """
    Integration with Claude Desktop via MCP.
    
    Usage:
        1. Install Claude Desktop
        2. Add to claude_desktop_config.json
        3. Start chatting with your robot!
    """
    
    @staticmethod
    def get_config(device_id: str = "bot1", device_type: str = "mobile_robot") -> Dict[str, Any]:
        """
        Get Claude Desktop configuration.
        
        Add this to your Claude Desktop config:
        ~/Library/Application Support/Claude/claude_desktop_config.json
        """
        return {
            "mcpServers": {
                "agent-ros-bridge": {
                    "command": "python",
                    "args": [
                        "-m",
                        "agent_ros_bridge.mcp",
                        device_id,
                        device_type
                    ],
                    "env": {
                        "PYTHONPATH": "${PYTHONPATH}"
                    }
                }
            }
        }
    
    @staticmethod
    def print_setup_instructions():
        """Print setup instructions for Claude Desktop"""
        print("""
🤖 Claude Desktop + Agent ROS Bridge Setup
==========================================

1. Install Claude Desktop from https://claude.ai/download

2. Open Claude Desktop settings:
   - Mac: Cmd + , (or Menu → Settings)
   - Windows/Linux: Edit → Settings

3. Click "Developer" → "Edit Config"

4. Add this configuration:

{
  "mcpServers": {
    "agent-ros-bridge": {
      "command": "python",
      "args": ["-m", "agent_ros_bridge.mcp", "bot1", "mobile_robot"]
    }
  }
}

5. Save and restart Claude Desktop

6. Look for the 🔨 icon (tools) in Claude's interface

7. Try these commands:
   - "Navigate to the kitchen"
   - "What's the robot's current status?"
   - "Pick up the red cup from the table"
   - "Emergency stop!"

Claude will show you the AI proposal and ask for confirmation
before executing (human-in-the-loop safety).

==========================================
        """)


class OpenAIGPTIntegration:
    """
    Integration with OpenAI GPT via function calling.
    
    This integrates Agent ROS Bridge with GPT's tool use capabilities
    while maintaining human confirmation for safety.
    """
    
    def __init__(self, robot_agent: 'RobotAgent', api_key: str = None):
        """
        Initialize GPT integration.
        
        Args:
            robot_agent: Configured RobotAgent instance
            api_key: OpenAI API key (defaults to OPENAI_API_KEY env var)
        """
        import os
        self.robot_agent = robot_agent
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        
        if not self.api_key:
            raise ValueError("OpenAI API key required. Set OPENAI_API_KEY env var.")
        
        self.tools = self._define_tools()
    
    def _define_tools(self) -> List[Dict[str, Any]]:
        """Define tools in OpenAI function calling format"""
        return [
            {
                "type": "function",
                "function": {
                    "name": "execute_robot_command",
                    "description": "Execute a natural language command on the robot",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "command": {
                                "type": "string",
                                "description": "Natural language command (e.g., 'go to the kitchen')"
                            }
                        },
                        "required": ["command"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "get_robot_status",
                    "description": "Get current robot status and observations",
                    "parameters": {
                        "type": "object",
                        "properties": {}
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "navigate_to",
                    "description": "Navigate robot to a specific location",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "location": {
                                "type": "string",
                                "description": "Target location (e.g., 'kitchen', 'office')"
                            }
                        },
                        "required": ["location"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "emergency_stop",
                    "description": "Immediately stop the robot (use in emergencies)",
                    "parameters": {
                        "type": "object",
                        "properties": {}
                    }
                }
            }
        ]


class CustomMCPClient:
    """
    Example of a custom MCP client implementation.
    
    This shows how to build your own client that connects to
    the Agent ROS Bridge MCP server.
    """
    
    def __init__(self, server_command: List[str]):
        """
        Initialize custom MCP client.
        
        Args:
            server_command: Command to start MCP server
                           e.g., ["python", "-m", "agent_ros_bridge.mcp", "bot1"]
        """
        self.server_command = server_command
        self.tools: List[Dict[str, Any]] = []
        self.initialized = False
        self.process = None


class MCPServer:
    """
    Standalone MCP server for Agent ROS Bridge.
    
    Can be run independently to expose robots to MCP clients.
    """
    
    def __init__(
        self,
        device_id: str,
        device_type: str = "mobile_robot",
        llm_provider: str = "moonshot",
    ):
        """
        Initialize MCP server.
        
        Args:
            device_id: Robot device ID
            device_type: Type of device
            llm_provider: LLM provider for intent parsing
        """
        from agent_ros_bridge.agentic import RobotAgent
        
        self.robot_agent = RobotAgent(
            device_id=device_id,
            device_type=device_type,
            llm_provider=llm_provider,
        )
        
        self.adapter = MCPAdapter(self.robot_agent)
    
    async def run(self):
        """Run MCP server over stdio"""
        print(
            f"Starting MCP server for {self.robot_agent.device_type} "
            f"({self.robot_agent.device_id})",
            file=__import__('sys').stderr
        )
        await self.adapter.serve_stdio()


# Example usage
if __name__ == "__main__":
    import sys
    
    print("=" * 70, file=sys.stderr)
    print("🤖 Agent ROS Bridge - MCP Server", file=sys.stderr)
    print("=" * 70, file=sys.stderr)
    print(file=sys.stderr)
    print("Usage:", file=sys.stderr)
    print("  python -m agent_ros_bridge.mcp <device_id> <device_type>", file=sys.stderr)
    print(file=sys.stderr)
    print("Example:", file=sys.stderr)
    print("  python -m agent_ros_bridge.mcp bot1 mobile_robot", file=sys.stderr)
    print(file=sys.stderr)
    print("Compatible with Claude Desktop, GPT, and other MCP clients.", file=sys.stderr)
    print("=" * 70, file=sys.stderr)
    print(file=sys.stderr)
    
    # Demo mode
    device_id = sys.argv[1] if len(sys.argv) > 1 else "bot1"
    device_type = sys.argv[2] if len(sys.argv) > 2 else "mobile_robot"
    
    print(f"Starting demo MCP server for {device_id} ({device_type})...", file=sys.stderr)
    print("(This is a demo - actual server would connect to real robot)", file=sys.stderr)
    print(file=sys.stderr)
    
    # Show available tools
    from unittest.mock import Mock
    mock_agent = Mock()
    mock_agent.device_id = device_id
    mock_agent.device_type = device_type
    mock_agent.require_confirmation = True
    mock_agent.device = Mock()
    mock_agent.device.get_capabilities.return_value = []
    
    adapter = MCPAdapter(mock_agent)
    
    print("Available MCP Tools:", file=sys.stderr)
    for tool in adapter.get_tools():
        print(f"  - {tool['name']}: {tool['description']}", file=sys.stderr)
    print(file=sys.stderr)
    
    print("To use with Claude Desktop, add to claude_desktop_config.json:", file=sys.stderr)
    print(json.dumps({
        "mcpServers": {
            "agent-ros-bridge": {
                "command": "python",
                "args": ["-m", "agent_ros_bridge.mcp", device_id, device_type]
            }
        }
    }, indent=2), file=sys.stderr)
