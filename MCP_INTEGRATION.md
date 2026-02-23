# Agent ROS Bridge as MCP Server

## What is MCP?

**Model Context Protocol (MCP)** - Anthropic's standard for connecting AI assistants to external tools.

- **Tools**: Functions AI can call (like ROS actions)
- **Resources**: Data AI can read (like ROS topics)
- **Prompts**: Templates for common tasks

**Standardizing how AI agents interact with external systems.**

---

## Current vs MCP Architecture

### Current (Custom Protocol)
```
AI Agent → WebSocket/gRPC → Agent ROS Bridge → ROS → Robot
         (Custom JSON)
```

### MCP Integration
```
Claude Desktop → MCP → Agent ROS Bridge MCP Server → ROS → Robot
                 (Standard protocol)
```

**Benefits:**
- Works with Claude Desktop out of the box
- Standard tool discovery
- Built-in safety/capability negotiation
- Part of growing MCP ecosystem

---

## How It Would Work

### 1. MCP Server Definition

```python
# mcp_server.py
from mcp.server import Server
from agent_ros_bridge import ROSBridge

app = Server("agent-ros-bridge")

@app.tool()
async def navigate_to(x: float, y: float) -> str:
    """Navigate robot to coordinates"""
    result = await bridge.call_action("navigate", x=x, y=y)
    return f"Navigated to ({x}, {y})"

@app.resource("ros://topic/battery")
async def get_battery() -> str:
    """Current battery level"""
    return await bridge.get_topic("/battery/status")
```

### 2. Claude Desktop Integration

**claude_desktop_config.json:**
```json
{
  "mcpServers": {
    "ros-bridge": {
      "command": "python3",
      "args": ["-m", "agent_ros_bridge.mcp_server"],
      "env": {
        "ROS_MASTER_URI": "http://localhost:11311",
        "JWT_SECRET": "..."
      }
    }
  }
}
```

### 3. User Experience

User in Claude Desktop:
```
User: "Navigate the robot to position (5, 3)"

Claude: I'll navigate the robot to coordinates (5, 3).
[Calls navigate_to tool]

Result: Successfully navigated to (5, 3). 
Current position confirmed.
```

**No custom code needed - just natural language!**

---

## MCP vs Current Approach

| Feature | Current | MCP |
|---------|---------|-----|
| AI Agent Support | Custom integration | Claude Desktop, any MCP client |
| Tool Discovery | Manual | Automatic |
| Safety | JWT auth | Built-in + JWT |
| Ecosystem | Isolated | Growing MCP ecosystem |
| Setup | Write code | Configure JSON |

---

## Implementation Strategy

### Phase 1: Basic MCP Server
- Expose ROS actions as MCP tools
- Expose ROS topics as MCP resources
- JWT authentication integration

### Phase 2: Advanced Features
- ROS node lifecycle as MCP prompts
- Multi-robot coordination tools
- Safety constraints as MCP prompts

### Phase 3: Ecosystem
- Claude Desktop marketplace
- Integration with other MCP clients
- Standard ROS tool definitions

---

## Strategic Value

### Immediate
- **Claude Desktop users** can control robots without coding
- **Lower barrier** to entry for AI-robotics
- **Standard integration** pattern

### Long-term
- **MCP becomes standard** for AI-tool interaction
- **Agent ROS Bridge** is the ROS adapter
- **Network effects** with MCP ecosystem

---

## Why This Makes Sense

1. **MCP is winning** - Anthropic pushing hard, growing adoption
2. **Natural fit** - ROS actions = MCP tools, ROS topics = MCP resources
3. **Easier for users** - No custom code, just configuration
4. **Future-proof** - Riding the MCP wave

---

## Architecture Decision

**Option A: Replace current protocol with MCP**
- Pros: Single standard
- Cons: Breaking change, limits flexibility

**Option B: Add MCP as alternative transport**
- Pros: Backwards compatible, choice of protocols
- Cons: More code to maintain

**Option C: MCP as higher-level abstraction**
- Keep WebSocket/gRPC for performance
- Add MCP server wrapper
- Best of both worlds

**Recommendation: Option C**

---

## Implementation

```python
# New module: agent_ros_bridge/mcp_server.py

from mcp.server import Server
from .core import ROSBridge

class ROSMCPBridge:
    def __init__(self, bridge: ROSBridge):
        self.bridge = bridge
        self.app = Server("agent-ros-bridge")
        self._register_tools()
    
    def _register_tools(self):
        """Auto-discover ROS actions and register as MCP tools"""
        for action in self.bridge.get_actions():
            self.app.tool()(self._wrap_action(action))
    
    def run(self):
        """Start MCP server"""
        self.app.run()
```

---

## Summary

**MCP integration would:**
- Make Agent ROS Bridge accessible to Claude Desktop users
- Lower barrier to entry (no coding required)
- Position us in the growing MCP ecosystem
- Provide standard AI-robotics integration pattern

**This is a strong strategic addition to the roadmap.**

---

*MCP could become the primary interface for casual users, while WebSocket/gRPC remain for power users.*
