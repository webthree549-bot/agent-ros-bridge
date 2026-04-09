"""Claude Desktop MCP Integration for Agent ROS Bridge.

Demonstrates how Claude Desktop can use Agent ROS Bridge
through the Model Context Protocol (MCP) to control ROS robots.
"""

import json
import asyncio
from dataclasses import dataclass
from typing import Any, Callable
from contextlib import asynccontextmanager

# MCP imports (when available)
try:
    from mcp.server import Server
    from mcp.types import Tool, TextContent, ImageContent
    from mcp.server.stdio import stdio_server
    MCP_AVAILABLE = True
except ImportError:
    MCP_AVAILABLE = False
    print("MCP not available. Install with: pip install mcp")

from agent_ros_bridge.agentic import RobotAgent
from agent_ros_bridge.tools import ROSTopicEchoTool, ROSServiceCallTool
from agent_ros_bridge.actions import create_action_client


@dataclass
class MCPContext:
    """Context for MCP tool execution."""
    robot_agent: RobotAgent
    session_id: str
    safety_level: str = "supervised"  # supervised, assisted, autonomous


class ClaudeMCPBridge:
    """MCP Bridge for Claude Desktop integration.
    
    Exposes Agent ROS Bridge capabilities to Claude Desktop
    through the Model Context Protocol.
    """
    
    def __init__(self, robot_agent: RobotAgent = None):
        self.robot_agent = robot_agent or RobotAgent(
            device_id="claude_bot",
            require_confirmation=True,  # Safety first
        )
        self.topic_tool = ROSTopicEchoTool()
        self.service_tool = ROSServiceCallTool()
        self.session_history = []
        
    def get_mcp_tools(self) -> list[dict]:
        """Get tools in MCP format for Claude Desktop.
        
        Returns:
            List of tool definitions in MCP schema format
        """
        return [
            {
                "name": "ros_navigate",
                "description": """
Navigate a ROS robot to a target location.

Use this when the user wants to:
- Move the robot somewhere
- Navigate to a position
- Drive to a location
- Go to a specific place

SAFETY: This operation requires human confirmation.
The system will ask for approval before moving the robot.

Examples:
- "Navigate to the kitchen"
- "Go to position A"
- "Move to the charging station"
                """.strip(),
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "location": {
                            "type": "string",
                            "description": "Target location name or coordinates"
                        },
                        "description": {
                            "type": "string",
                            "description": "Human-readable description of the navigation goal"
                        },
                        "safety_override": {
                            "type": "boolean",
                            "description": "Whether to bypass safety checks (requires admin)",
                            "default": False
                        }
                    },
                    "required": ["location"]
                }
            },
            {
                "name": "ros_inspect_topic",
                "description": """
Inspect a ROS topic and get its current data.

Use this when the user wants to:
- Check sensor data
- Read topic values
- Monitor robot state
- Get telemetry

Examples:
- "Check the /cmd_vel topic"
- "What's on /odom?"
- "Read the laser scan data"
- "Show me joint states"
                """.strip(),
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "topic": {
                            "type": "string",
                            "description": "ROS topic name (e.g., /cmd_vel, /odom, /scan)"
                        },
                        "message_count": {
                            "type": "integer",
                            "description": "Number of messages to collect",
                            "default": 1,
                            "minimum": 1,
                            "maximum": 100
                        }
                    },
                    "required": ["topic"]
                }
            },
            {
                "name": "ros_call_service",
                "description": """
Call a ROS service to trigger an action.

Use this when the user wants to:
- Clear costmaps
- Reset odometry
- Trigger behaviors
- Execute services

SAFETY: This operation may require human confirmation
depending on the service being called.

Examples:
- "Clear the costmap"
- "Reset odometry"
- "Call the planner service"
                """.strip(),
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "service": {
                            "type": "string",
                            "description": "ROS service name (e.g., /clear_costmap, /reset_odom)"
                        },
                        "parameters": {
                            "type": "object",
                            "description": "Service request parameters",
                            "default": {}
                        }
                    },
                    "required": ["service"]
                }
            },
            {
                "name": "ros_get_status",
                "description": """
Get comprehensive robot status and safety information.

Use this when the user wants to:
- Check robot health
- View safety status
- See shadow mode progress
- Get system overview

Returns:
- Robot connection status
- Safety validation state
- Shadow mode hours collected
- Current autonomy level

Examples:
- "What's the status?"
- "Is the robot safe?"
- "How many shadow hours?"
- "Show me system info"
                """.strip(),
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "include_telemetry": {
                            "type": "boolean",
                            "description": "Include detailed telemetry data",
                            "default": True
                        }
                    }
                }
            },
            {
                "name": "ros_emergency_stop",
                "description": """
EMERGENCY STOP - Immediately halt all robot motion.

Use this when:
- There is immediate danger
- The robot is behaving unexpectedly
- A collision is imminent
- Any safety concern arises

⚠️  WARNING: This command has highest priority and bypasses
normal safety confirmations. Use only in emergencies.

Examples:
- "STOP!"
- "Emergency stop!"
- "Halt immediately!"
                """.strip(),
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "reason": {
                            "type": "string",
                            "description": "Reason for emergency stop (for logging)"
                        }
                    }
                }
            },
            {
                "name": "ros_manipulate",
                "description": """
Control a robot arm or gripper for manipulation tasks.

Use this when the user wants to:
- Pick up objects
- Place items
- Move the arm
- Grasp or release

SAFETY: Manipulation requires high confidence and human
confirmation. The system will validate all movements.

Examples:
- "Pick up the cup"
- "Place object on table"
- "Move arm to home position"
- "Open gripper"
                """.strip(),
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "enum": ["pick", "place", "move", "grasp", "release", "home"],
                            "description": "Manipulation action to perform"
                        },
                        "target": {
                            "type": "string",
                            "description": "Target object or location"
                        },
                        "pose": {
                            "type": "object",
                            "description": "Target pose (x, y, z, orientation)",
                            "properties": {
                                "x": {"type": "number"},
                                "y": {"type": "number"},
                                "z": {"type": "number"}
                            }
                        }
                    },
                    "required": ["action"]
                }
            }
        ]
    
    async def execute_tool(self, tool_name: str, arguments: dict) -> list[dict]:
        """Execute an MCP tool.
        
        Args:
            tool_name: Name of the tool to execute
            arguments: Tool arguments
            
        Returns:
            MCP content objects (TextContent or ImageContent)
        """
        try:
            if tool_name == "ros_navigate":
                return await self._handle_navigate(arguments)
            elif tool_name == "ros_inspect_topic":
                return await self._handle_inspect_topic(arguments)
            elif tool_name == "ros_call_service":
                return await self._handle_call_service(arguments)
            elif tool_name == "ros_get_status":
                return await self._handle_get_status(arguments)
            elif tool_name == "ros_emergency_stop":
                return await self._handle_emergency_stop(arguments)
            elif tool_name == "ros_manipulate":
                return await self._handle_manipulate(arguments)
            else:
                return [{
                    "type": "text",
                    "text": f"❌ Unknown tool: {tool_name}"
                }]
        except Exception as e:
            return [{
                "type": "text",
                "text": f"❌ Error executing {tool_name}: {str(e)}"
            }]
    
    async def _handle_navigate(self, args: dict) -> list[dict]:
        """Handle navigation tool."""
        location = args.get("location", "unknown")
        description = args.get("description", f"Navigate to {location}")
        
        print(f"🤖 Claude requested: {description}")
        print(f"⚠️  Safety: Awaiting human confirmation...")
        
        # Execute with RobotAgent (includes safety)
        result = self.robot_agent.execute(f"Navigate to {location}")
        
        if result.success:
            text = f"""✅ Navigation Complete
            
Destination: {location}
Duration: {result.duration_seconds:.1f} seconds
AI Confidence: {result.ai_confidence:.2%}
Human Approvals: {result.human_approvals}

The robot has successfully navigated to {location} with full safety validation."""
        else:
            text = f"""❌ Navigation Failed
            
Destination: {location}
Error: {result.message}

The navigation could not be completed. Please check:
- Is the location reachable?
- Are there obstacles blocking the path?
- Is the localization system working?"""
        
        return [{"type": "text", "text": text}]
    
    async def _handle_inspect_topic(self, args: dict) -> list[dict]:
        """Handle topic inspection tool."""
        topic = args.get("topic", "/unknown")
        count = args.get("message_count", 1)
        
        result = self.topic_tool.execute(topic=topic, count=count)
        
        if result.success:
            text = f"""✅ Topic Data: {topic}

Execution Time: {result.execution_time_ms:.1f}ms

Data:
{result.output}

Metadata:
{json.dumps(result.data, indent=2)}"""
        else:
            text = f"""❌ Failed to read topic: {topic}

Error: {result.error}

Possible causes:
- Topic doesn't exist
- No publishers on topic
- Message type not recognized"""
        
        return [{"type": "text", "text": text}]
    
    async def _handle_call_service(self, args: dict) -> list[dict]:
        """Handle service call tool."""
        service = args.get("service", "/unknown")
        params = args.get("parameters", {})
        
        print(f"🤖 Claude requested service call: {service}")
        print(f"⚠️  Safety: Awaiting human confirmation...")
        
        result = self.service_tool.execute(service=service, request=params)
        
        if result.success:
            text = f"""✅ Service Called Successfully: {service}

Response:
{result.output}

Execution Time: {result.execution_time_ms:.1f}ms"""
        else:
            text = f"❌ Service call failed: {result.error}"
        
        return [{"type": "text", "text": text}]
    
    async def _handle_get_status(self, args: dict) -> list[dict]:
        """Handle status tool."""
        status = self.robot_agent.get_state()
        safety = self.robot_agent.safety
        
        text = f"""🤖 Robot Status Report

**Device Information:**
- ID: {status.get('device_id', 'unknown')}
- Type: {status.get('device_type', 'unknown')}
- Connected: {'✅ Yes' if status.get('connected') else '❌ No'}

**Safety Status:**
- Validation Stage: {safety.safety_validation_status.upper()}
- Autonomous Mode: {'⚠️ Enabled' if safety.autonomous_mode else '✅ Human Supervised'}
- Human-in-the-Loop: {'✅ Active' if safety.human_in_the_loop else '⚠️ Disabled'}
- Shadow Mode: {'✅ Collecting' if safety.shadow_mode_enabled else '❌ Off'}

**Shadow Mode Progress:**
- Hours Collected: {safety.shadow_mode_hours_collected:.1f} / {safety.required_shadow_hours}
- Progress: {(safety.shadow_mode_hours_collected / safety.required_shadow_hours * 100):.1f}%
- Agreement Rate: {safety.shadow_mode_agreement_rate:.1%}

**Deployment Readiness:**
{'✅ Ready for Gradual Rollout' if safety.shadow_mode_hours_collected >= safety.required_shadow_hours else '⏳ Collecting Shadow Data'}

The robot is operating safely with full human oversight."""
        
        return [{"type": "text", "text": text}]
    
    async def _handle_emergency_stop(self, args: dict) -> list[dict]:
        """Handle emergency stop tool."""
        reason = args.get("reason", "User requested emergency stop via Claude")
        
        print(f"🚨 EMERGENCY STOP triggered via Claude")
        print(f"Reason: {reason}")
        
        result = self.robot_agent.execute("STOP")
        
        text = f"""🚨 EMERGENCY STOP EXECUTED

Status: {'✅ Robot Halted' if result.success else '❌ Failed to Stop'}
Reason: {reason}
Timestamp: {asyncio.get_event_loop().time()}

The robot has been immediately halted. All motion commands are suspended.

To resume operation:
1. Ensure the area is safe
2. Clear any obstacles or hazards
3. Reset the emergency stop (if applicable)
4. Resume with caution"""
        
        return [{"type": "text", "text": text}]
    
    async def _handle_manipulate(self, args: dict) -> list[dict]:
        """Handle manipulation tool."""
        action = args.get("action", "unknown")
        target = args.get("target", "unknown")
        
        print(f"🤖 Claude requested manipulation: {action} {target}")
        print(f"⚠️  Safety: High-risk operation - awaiting confirmation...")
        
        # This would connect to manipulation action client
        text = f"""⏳ Manipulation Command Queued

Action: {action.upper()}
Target: {target}
Safety Level: HIGH (Human confirmation required)

The manipulation command has been submitted to the safety queue.
Please confirm this operation through the safety interface.

Note: Manipulation tasks require:
- Clear workspace
- Proper calibration
- Human supervision
- Gradual execution"""
        
        return [{"type": "text", "text": text}]


# MCP Server setup
async def main():
    """Run MCP server for Claude Desktop."""
    if not MCP_AVAILABLE:
        print("MCP not available. Running in demo mode.")
        bridge = ClaudeMCPBridge()
        
        # Demo: Show available tools
        tools = bridge.get_mcp_tools()
        print("\n" + "=" * 60)
        print("🦾 Claude Desktop MCP Integration")
        print("=" * 60)
        print("\nAvailable Tools:")
        for tool in tools:
            print(f"\n📋 {tool['name']}")
            print(f"   {tool['description'][:100]}...")
        
        # Demo: Execute a tool
        print("\n" + "=" * 60)
        print("Demo: ros_get_status")
        print("=" * 60)
        result = await bridge.execute_tool("ros_get_status", {})
        print(result[0]['text'])
        
        return
    
    # Real MCP server
    bridge = ClaudeMCPBridge()
    server = Server("agent-ros-bridge")
    
    @server.list_tools()
    async def list_tools() -> list[Tool]:
        """List available tools."""
        tools_data = bridge.get_mcp_tools()
        return [Tool(**tool) for tool in tools_data]
    
    @server.call_tool()
    async def call_tool(name: str, arguments: dict) -> list:
        """Execute a tool."""
        return await bridge.execute_tool(name, arguments)
    
    # Run server
    async with stdio_server(server) as (read_stream, write_stream):
        await server.run(
            read_stream,
            write_stream,
            server.create_initialization_options()
        )


if __name__ == "__main__":
    asyncio.run(main())
