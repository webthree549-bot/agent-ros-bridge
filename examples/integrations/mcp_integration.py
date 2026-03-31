"""
MCP Client Integration Examples

Demonstrates how to integrate Agent ROS Bridge with:
1. Claude Desktop (via MCP)
2. OpenAI GPT (via function calling)
3. Custom MCP clients
"""

import asyncio
import json
import os
from typing import Any, Dict, List

from agent_ros_bridge.agentic import RobotAgent
from agent_ros_bridge.mcp import MCPAdapter


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
    
    def __init__(self, robot_agent: RobotAgent, api_key: str = None):
        """
        Initialize GPT integration.
        
        Args:
            robot_agent: Configured RobotAgent instance
            api_key: OpenAI API key (defaults to OPENAI_API_KEY env var)
        """
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
    
    async def chat(self, user_message: str, conversation_history: List[Dict] = None) -> str:
        """
        Chat with GPT about robot control.
        
        Args:
            user_message: User's natural language message
            conversation_history: Previous messages (optional)
            
        Returns:
            GPT's response with any tool executions
        """
        try:
            import openai
        except ImportError:
            raise ImportError("Install openai: pip install openai")
        
        client = openai.AsyncOpenAI(api_key=self.api_key)
        
        messages = conversation_history or []
        messages.append({"role": "user", "content": user_message})
        
        # Get GPT's response with potential tool calls
        response = await client.chat.completions.create(
            model="gpt-4o",
            messages=messages,
            tools=self.tools,
            tool_choice="auto"
        )
        
        message = response.choices[0].message
        
        # If GPT wants to use tools
        if message.tool_calls:
            # Add assistant's tool call to conversation
            messages.append({
                "role": "assistant",
                "content": message.content or "",
                "tool_calls": [
                    {
                        "id": tc.id,
                        "type": "function",
                        "function": {
                            "name": tc.function.name,
                            "arguments": tc.function.arguments
                        }
                    }
                    for tc in message.tool_calls
                ]
            })
            
            # Execute tools and get results
            for tool_call in message.tool_calls:
                function_name = tool_call.function.name
                function_args = json.loads(tool_call.function.arguments)
                
                print(f"\n🤖 GPT wants to execute: {function_name}")
                print(f"   Arguments: {function_args}")
                
                # HUMAN CONFIRMATION for safety
                if function_name == "emergency_stop":
                    # Always execute emergency stop immediately
                    result = await self._execute_tool(function_name, function_args)
                else:
                    # Ask for human confirmation
                    print(f"\n⚠️  Human confirmation required:")
                    print(f"   GPT proposes: {function_name}({function_args})")
                    
                    confirmation = input("   Approve? (y/n/modify): ").strip().lower()
                    
                    if confirmation == "y":
                        result = await self._execute_tool(function_name, function_args)
                    elif confirmation == "n":
                        result = {"status": "rejected", "reason": "Human rejection"}
                    else:
                        # Modified command
                        result = await self._execute_tool(function_name, function_args)
                
                # Add tool result to conversation
                messages.append({
                    "role": "tool",
                    "tool_call_id": tool_call.id,
                    "content": json.dumps(result)
                })
            
            # Get GPT's response to tool results
            final_response = await client.chat.completions.create(
                model="gpt-4o",
                messages=messages,
                tools=self.tools
            )
            
            return final_response.choices[0].message.content
        
        return message.content
    
    async def _execute_tool(self, function_name: str, function_args: Dict) -> Dict:
        """Execute a tool and return result"""
        if function_name == "execute_robot_command":
            command = function_args.get("command", "")
            result = self.robot_agent.execute(command)
            return {
                "success": result.success,
                "message": result.message,
                "ai_confidence": result.ai_confidence,
                "human_approvals": result.human_approvals,
                "safety_violations": result.safety_violations
            }
        
        elif function_name == "get_robot_status":
            obs = self.robot_agent.observe()
            return {
                "position": obs.robot_position,
                "battery": obs.battery_level,
                "nearby_objects": obs.nearby_objects,
                "current_task": obs.current_task
            }
        
        elif function_name == "navigate_to":
            location = function_args.get("location", "")
            result = self.robot_agent.execute(f"Navigate to {location}")
            return {
                "success": result.success,
                "location": location,
                "message": result.message
            }
        
        elif function_name == "emergency_stop":
            device = self.robot_agent.device
            if device and device.has_capability("stop"):
                device.execute_capability("stop", {})
                return {"success": True, "message": "Emergency stop executed"}
            return {"success": False, "message": "Stop capability not available"}
        
        return {"error": f"Unknown function: {function_name}"}


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
        self.tools = []
        self.initialized = False
    
    async def connect(self):
        """Connect to MCP server and initialize"""
        import subprocess
        
        # Start server process
        self.process = await asyncio.create_subprocess_exec(
            *self.server_command,
            stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        
        # Send initialize request
        init_request = {
            "jsonrpc": "2.0",
            "method": "initialize",
            "id": 1
        }
        
        await self._send_request(init_request)
        response = await self._read_response()
        
        if "result" in response:
            self.initialized = True
            print("✅ Connected to MCP server")
            
            # Get available tools
            await self._refresh_tools()
        else:
            raise ConnectionError(f"Failed to initialize: {response}")
    
    async def _refresh_tools(self):
        """Refresh list of available tools"""
        request = {
            "jsonrpc": "2.0",
            "method": "tools/list",
            "id": 2
        }
        
        await self._send_request(request)
        response = await self._read_response()
        
        if "result" in response:
            self.tools = response["result"]["tools"]
            print(f"📋 Available tools: {[t['name'] for t in self.tools]}")
    
    async def call_tool(self, name: str, arguments: Dict[str, Any]) -> Dict[str, Any]:
        """Call an MCP tool"""
        if not self.initialized:
            raise RuntimeError("Not connected to MCP server")
        
        request = {
            "jsonrpc": "2.0",
            "method": "tools/call",
            "params": {
                "name": name,
                "arguments": arguments
            },
            "id": 3
        }
        
        await self._send_request(request)
        response = await self._read_response()
        
        return response.get("result", {})
    
    async def _send_request(self, request: Dict):
        """Send JSON-RPC request to server"""
        request_json = json.dumps(request) + "\n"
        self.process.stdin.write(request_json.encode())
        await self.process.stdin.drain()
    
    async def _read_response(self) -> Dict:
        """Read JSON-RPC response from server"""
        line = await self.process.stdout.readline()
        return json.loads(line.decode().strip())
    
    async def disconnect(self):
        """Disconnect from MCP server"""
        if self.process:
            self.process.terminate()
            await self.process.wait()
            print("🔌 Disconnected from MCP server")


async def demo_claude_desktop():
    """Demo: Claude Desktop setup"""
    print("=" * 70)
    print("🤖 Agent ROS Bridge - Claude Desktop Integration Demo")
    print("=" * 70)
    print()
    
    ClaudeDesktopIntegration.print_setup_instructions()
    
    # Print config
    config = ClaudeDesktopIntegration.get_config("bot1", "mobile_robot")
    print("\n📄 Claude Desktop Config (claude_desktop_config.json):")
    print(json.dumps(config, indent=2))
    print()


async def demo_openai_gpt():
    """Demo: OpenAI GPT integration with human confirmation"""
    print("=" * 70)
    print("🤖 Agent ROS Bridge - OpenAI GPT Integration Demo")
    print("=" * 70)
    print()
    
    # Check for API key
    if not os.getenv("OPENAI_API_KEY"):
        print("⚠️  Set OPENAI_API_KEY environment variable to run this demo")
        print("   export OPENAI_API_KEY='your-key-here'")
        return
    
    # Create mock robot agent for demo
    from unittest.mock import Mock
    mock_agent = Mock()
    mock_agent.device_id = "bot1"
    mock_agent.device_type = "mobile_robot"
    
    mock_result = Mock()
    mock_result.success = True
    mock_result.message = "Successfully navigated to kitchen"
    mock_result.ai_confidence = 0.94
    mock_result.human_approvals = 1
    mock_result.safety_violations = 0
    
    mock_agent.execute.return_value = mock_result
    mock_agent.observe.return_value = Mock(
        robot_position=(1.0, 2.0, 0.0),
        battery_level=85.0,
        nearby_objects=["cup", "table"],
        current_task="idle",
        recent_commands=[],
        obstacles_detected=[]
    )
    mock_agent.device = Mock()
    mock_agent.device.has_capability.return_value = True
    
    # Create GPT integration
    gpt = OpenAIGPTIntegration(mock_agent)
    
    print("💬 GPT Integration ready!")
    print("   Tools available:")
    for tool in gpt.tools:
        print(f"     - {tool['function']['name']}")
    print()
    print("   Example conversation:")
    print('   User: "Go to the kitchen"')
    print('   GPT: [calls navigate_to tool]')
    print('   System: [asks for human confirmation]')
    print('   Human: "y" (approved)')
    print('   Robot: [executes command]')
    print()
    print("Note: This demo requires OPENAI_API_KEY to actually run")


async def demo_custom_client():
    """Demo: Custom MCP client"""
    print("=" * 70)
    print("🤖 Agent ROS Bridge - Custom MCP Client Demo")
    print("=" * 70)
    print()
    
    print("This shows how to build a custom MCP client that connects to")
    print("the Agent ROS Bridge MCP server.")
    print()
    
    print("Example usage:")
    print("```python")
    print("client = CustomMCPClient([")
    print("    'python', '-m', 'agent_ros_bridge.mcp', 'bot1', 'mobile_robot'")
    print("])")
    print("")
    print("await client.connect()")
    print("result = await client.call_tool('get_robot_status', {})")
    print("result = await client.call_tool('navigate_to', {'location': 'kitchen'})")
    print("await client.disconnect()")
    print("```")
    print()
    
    print("Features:")
    print("  ✅ JSON-RPC communication")
    print("  ✅ Tool discovery")
    print("  ✅ Async/await support")
    print("  ✅ Error handling")


async def main():
    """Main demo"""
    import sys
    
    print("Agent ROS Bridge - MCP Client Integration Examples")
    print()
    print("Choose demo:")
    print("  1. Claude Desktop setup")
    print("  2. OpenAI GPT integration")
    print("  3. Custom MCP client")
    print()
    
    # Default to showing all demos
    await demo_claude_desktop()
    print("\n" + "=" * 70 + "\n")
    await demo_openai_gpt()
    print("\n" + "=" * 70 + "\n")
    await demo_custom_client()
    
    print("\n" + "=" * 70)
    print("For more information, see: docs/MCP_INTEGRATION.md")
    print("=" * 70)


if __name__ == "__main__":
    asyncio.run(main())
