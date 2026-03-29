# MCP Integration Guide

Agent ROS Bridge supports the Model Context Protocol (MCP), enabling seamless integration with Claude Desktop, OpenAI GPT, and other MCP-compatible clients.

## Table of Contents

1. [Quick Start](#quick-start)
2. [Claude Desktop Integration](#claude-desktop-integration)
3. [OpenAI GPT Integration](#openai-gpt-integration)
4. [Custom MCP Clients](#custom-mcp-clients)
5. [Available Tools](#available-tools)
6. [Safety Features](#safety-features)
7. [Troubleshooting](#troubleshooting)

---

## Quick Start

### Installation

```bash
pip install agent-ros-bridge
```

### Start MCP Server

```bash
# For mobile robot
python -m agent_ros_bridge.mcp bot1 mobile_robot

# For drone
python -m agent_ros_bridge.mcp drone1 drone

# For robot arm
python -m agent_ros_bridge.mcp arm1 manipulator
```

---

## Claude Desktop Integration

Claude Desktop provides the easiest way to interact with your robot using natural language.

### Setup

1. **Install Claude Desktop**
   - Download from [claude.ai/download](https://claude.ai/download)
   - Available for Mac, Windows, and Linux

2. **Open Settings**
   - Mac: `Cmd + ,` or Menu → Settings
   - Windows/Linux: Edit → Settings

3. **Enable Developer Mode**
   - Click "Developer" in left sidebar
   - Click "Edit Config"

4. **Add MCP Configuration**

   Create or edit `claude_desktop_config.json`:

   **Mac:** `~/Library/Application Support/Claude/claude_desktop_config.json`
   
   **Windows:** `%APPDATA%/Claude/claude_desktop_config.json`
   
   **Linux:** `~/.config/Claude/claude_desktop_config.json`

   ```json
   {
     "mcpServers": {
       "agent-ros-bridge": {
         "command": "python",
         "args": [
           "-m",
           "agent_ros_bridge.mcp",
           "bot1",
           "mobile_robot"
         ]
       }
     }
   }
   ```

5. **Restart Claude Desktop**

6. **Verify Connection**
   - Look for the 🔨 (tools) icon in Claude's interface
   - Click it to see available robot tools

### Using Claude with Your Robot

Once connected, you can chat with Claude about your robot:

**Example Conversations:**

```
You: Navigate to the kitchen

Claude: I'll navigate the robot to the kitchen.
[Claude shows the AI proposal with confidence: 0.94]

⚠️ Human confirmation required:
   AI proposes: navigate_to({"location": "kitchen"})
   
Approve? (Claude will ask you to confirm)

You: Yes

Claude: ✅ Command executed successfully!
   The robot is now navigating to the kitchen.
   Duration: 12.3 seconds
   No safety violations detected.
```

```
You: What's the robot's current status?

Claude: Here's the current robot status:
   
   🤖 Device: bot1 (mobile_robot)
   📍 Position: (5.2, 3.1, 0.0)
   🔋 Battery: 87%
   🎯 Current Task: navigating
   📦 Nearby Objects: ["cup", "table", "chair"]
```

```
You: Pick up the red cup from the table

Claude: I'll pick up the red cup from the table.
[AI proposal with confidence: 0.91]

⚠️ Human confirmation required:
   AI proposes: pick_object({"object": "red cup", "location": "table"})

You: Yes

Claude: ✅ Object picked up successfully!
   The robot has grasped the red cup from the table.
   Grip force: 0.5N (safe)
```

### Advanced Configuration

#### Custom Robot Parameters

```json
{
  "mcpServers": {
    "agent-ros-bridge": {
      "command": "python",
      "args": [
        "-m",
        "agent_ros_bridge.mcp",
        "my_robot",
        "mobile_robot",
        "--llm-provider",
        "openai"
      ],
      "env": {
        "OPENAI_API_KEY": "your-key-here",
        "ROS_MASTER_URI": "http://localhost:11311"
      }
    }
  }
}
```

#### Multiple Robots

```json
{
  "mcpServers": {
    "robot-1": {
      "command": "python",
      "args": ["-m", "agent_ros_bridge.mcp", "bot1", "mobile_robot"]
    },
    "robot-2": {
      "command": "python",
      "args": ["-m", "agent_ros_bridge.mcp", "arm1", "manipulator"]
    },
    "drone": {
      "command": "python",
      "args": ["-m", "agent_ros_bridge.mcp", "drone1", "drone"]
    }
  }
}
```

---

## OpenAI GPT Integration

For programmatic integration with GPT-4 or other OpenAI models, use the function calling API.

### Setup

```python
import asyncio
import os
from agent_ros_bridge.agentic import RobotAgent
from agent_ros_bridge.mcp import OpenAIGPTIntegration

# Set API key
os.environ["OPENAI_API_KEY"] = "your-key-here"

# Create robot agent
agent = RobotAgent(
    device_id="bot1",
    device_type="mobile_robot",
    llm_provider="openai"
)

# Create GPT integration
gpt = OpenAIGPTIntegration(agent)

# Chat with GPT about robot control
async def main():
    response = await gpt.chat("Navigate to the kitchen")
    print(response)

asyncio.run(main())
```

### Example: Interactive GPT Session

```python
import asyncio
from agent_ros_bridge.mcp import OpenAIGPTIntegration

async def interactive_session():
    agent = RobotAgent(device_id="bot1", device_type="mobile_robot")
    gpt = OpenAIGPTIntegration(agent)
    
    conversation = []
    
    print("🤖 GPT Robot Control Session")
    print("Type 'exit' to quit\n")
    
    while True:
        user_input = input("You: ")
        
        if user_input.lower() == "exit":
            break
        
        # GPT processes the request
        response = await gpt.chat(user_input, conversation)
        
        # If GPT wants to execute a command, it will ask for confirmation
        print(f"GPT: {response}\n")
        
        # Save to conversation history
        conversation.append({"role": "user", "content": user_input})
        conversation.append({"role": "assistant", "content": response})

asyncio.run(interactive_session())
```

### Example Output

```
🤖 GPT Robot Control Session
Type 'exit' to quit

You: Go to the kitchen and check what's there

GPT: I'll navigate the robot to the kitchen and then check the surroundings.

🤖 GPT wants to execute: navigate_to
   Arguments: {"location": "kitchen"}

⚠️  Human confirmation required:
   GPT proposes: navigate_to({"location": "kitchen"})
   
Approve? (y/n/modify): y

✅ Command approved and executed
   The robot has reached the kitchen.

🤖 GPT wants to execute: get_robot_status
   Arguments: {}

⚠️  Human confirmation required:
   GPT proposes: get_robot_status({})
   
Approve? (y/n/modify): y

✅ In the kitchen, I can see:
   - A red cup on the counter
   - A book on the table
   - No people present
   
   Battery level: 85%

GPT: The robot has navigated to the kitchen and identified a red cup on the counter and a book on the table. The area appears clear of people.
```

---

## Custom MCP Clients

Build your own MCP client for specialized applications.

### Basic Client

```python
import asyncio
import json
from agent_ros_bridge.mcp import CustomMCPClient

async def main():
    # Create client
    client = CustomMCPClient([
        "python", "-m", "agent_ros_bridge.mcp",
        "bot1", "mobile_robot"
    ])
    
    # Connect to server
    await client.connect()
    
    # Get robot status
    status = await client.call_tool("get_robot_status", {})
    print(f"Robot position: {status['position']}")
    
    # Navigate to location
    result = await client.call_tool("navigate_to", {
        "location": "kitchen"
    })
    print(f"Navigation: {result['message']}")
    
    # Emergency stop (if needed)
    # await client.call_tool("emergency_stop", {})
    
    # Disconnect
    await client.disconnect()

asyncio.run(main())
```

### Advanced Client with Tool Discovery

```python
class SmartMCPClient(CustomMCPClient):
    """Client that discovers and validates tools"""
    
    async def execute_command(self, natural_language: str):
        """Execute natural language command with validation"""
        
        # First, get current status
        status = await self.call_tool("get_robot_status", {})
        
        # Determine best tool for the command
        if "navigate" in natural_language.lower():
            tool_name = "navigate_to"
            # Extract location (simplified)
            location = natural_language.split("to")[-1].strip()
            args = {"location": location}
        elif "pick" in natural_language.lower():
            tool_name = "pick_object"
            # Extract object (simplified)
            args = {"object": "unknown"}  # Would use NLP here
        else:
            tool_name = "execute_robot_command"
            args = {"command": natural_language}
        
        # Validate tool exists
        if not any(t["name"] == tool_name for t in self.tools):
            raise ValueError(f"Tool {tool_name} not available")
        
        # Execute
        return await self.call_tool(tool_name, args)

# Usage
client = SmartMCPClient(["python", "-m", "agent_ros_bridge.mcp", "bot1"])
await client.connect()
result = await client.execute_command("Go to the kitchen")
```

---

## Available Tools

### execute_robot_command
Execute arbitrary natural language command.

**Input:**
```json
{
  "command": "Go to the kitchen and pick up the red cup",
  "require_confirmation": true
}
```

**Output:**
```json
{
  "success": true,
  "command": "Go to the kitchen and pick up the red cup",
  "ai_confidence": 0.94,
  "steps_executed": 2,
  "human_approvals": 2,
  "safety_violations": 0,
  "duration_seconds": 18.5,
  "message": "Successfully navigated to kitchen and picked up red cup"
}
```

### get_robot_status
Get current robot observations and state.

**Input:** `{}`

**Output:**
```json
{
  "device_id": "bot1",
  "device_type": "mobile_robot",
  "position": [5.2, 3.1, 0.0],
  "battery": 87,
  "nearby_objects": ["cup", "table"],
  "capabilities": ["navigate_to", "rotate", "stop"]
}
```

### navigate_to
Navigate robot to specific location.

**Input:**
```json
{
  "location": "kitchen",
  "coordinates": [10.5, 5.2]
}
```

**Output:**
```json
{
  "success": true,
  "location": "kitchen",
  "coordinates": [10.5, 5.2],
  "message": "Successfully navigated to kitchen",
  "human_approvals_required": true
}
```

### pick_object
Pick up an object.

**Input:**
```json
{
  "object": "red cup",
  "location": "table"
}
```

**Output:**
```json
{
  "success": true,
  "object": "red cup",
  "message": "Successfully picked up red cup",
  "safety_violations": 0
}
```

### move_manipulator
Move robot arm to position.

**Input:**
```json
{
  "x": 0.5,
  "y": 0.2,
  "z": 0.3
}
```

**Output:**
```json
{
  "success": true,
  "position": [0.5, 0.2, 0.3],
  "message": "Arm moved to target position"
}
```

### emergency_stop
Immediately stop the robot (no confirmation required).

**Input:** `{}`

**Output:**
```json
{
  "success": true,
  "message": "Emergency stop executed",
  "robot_stopped": true
}
```

---

## Safety Features

### Human-in-the-Loop

All non-emergency commands require human confirmation:

1. **AI Proposal:** LLM generates action with confidence score
2. **Human Review:** Operator sees proposal and can approve/reject/modify
3. **Execution:** Only approved commands execute
4. **Logging:** All decisions logged for shadow mode learning

### Emergency Stop

Emergency stop tools execute immediately without confirmation:

```python
# This executes immediately
await client.call_tool("emergency_stop", {})
```

### Safety Validation

Every command validated against:
- Hardware limits (velocity, acceleration, workspace)
- Collision detection
- Task feasibility
- AI confidence thresholds

### Shadow Mode

In shadow mode, the system:
1. Logs AI proposals
2. Logs human decisions
3. Measures agreement rate
4. Uses disagreements for model improvement

---

## Troubleshooting

### Claude Desktop: "Tool not found"

**Problem:** Claude doesn't show robot tools

**Solutions:**
1. Check config file path is correct for your OS
2. Verify JSON syntax is valid
3. Restart Claude Desktop completely
4. Check Claude Desktop logs:
   - Mac: `~/Library/Logs/Claude/`
   - Check for MCP server errors

### OpenAI: "API key not found"

**Problem:** OPENAI_API_KEY not set

**Solution:**
```bash
export OPENAI_API_KEY="your-key-here"
```

Or set in Python:
```python
import os
os.environ["OPENAI_API_KEY"] = "your-key-here"
```

### MCP Server: "Connection refused"

**Problem:** Can't connect to MCP server

**Solutions:**
1. Verify Python is installed: `python --version`
2. Install agent-ros-bridge: `pip install agent-ros-bridge`
3. Check server is running: `python -m agent_ros_bridge.mcp bot1 mobile_robot`
4. Verify device type is valid

### Commands not executing

**Problem:** Commands fail or timeout

**Solutions:**
1. Check robot is connected and responsive
2. Verify ROS is running: `ros2 topic list`
3. Check safety limits aren't being violated
4. Review shadow mode logs for errors

### Human confirmation not appearing

**Problem:** Commands execute without asking

**Cause:** High AI confidence (>0.8) auto-approves

**Solution:** Adjust confidence threshold:
```python
agent = RobotAgent(
    device_id="bot1",
    device_type="mobile_robot",
    confidence_threshold=0.95  # Higher = more confirmations
)
```

---

## Examples

See `examples/mcp_integration.py` for complete working examples:

```bash
python examples/mcp_integration.py
```

This will show:
- Claude Desktop setup instructions
- OpenAI GPT integration demo
- Custom MCP client examples

---

## Next Steps

- [Shadow Mode Configuration](../SHADOW_MODE.md)
- [Safety Validation](../SAFETY.md)
- [Hardware Abstraction](../HARDWARE.md)
- [API Reference](../API_REFERENCE.md)

---

## Support

- GitHub Issues: https://github.com/webthree549-bot/agent-ros-bridge/issues
- Documentation: https://docs.agent-ros-bridge.ai
- Discord: https://discord.gg/agent-ros-bridge
