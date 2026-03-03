# ADR-0005: AI Framework Integration Strategy

## Status

**Accepted**

## Context

AI agents need to control robots through various frameworks:
- **LangChain** — Popular LLM chaining framework
- **AutoGPT** — Autonomous agent framework
- **MCP (Model Context Protocol)** — Anthropic's protocol for Claude Desktop
- **Custom agents** — Direct API usage

Each framework has different integration patterns:
- LangChain uses `BaseTool` classes
- AutoGPT uses command-based plugins
- MCP uses JSON-RPC over stdio/SSE

We needed a strategy to support multiple frameworks without coupling the core bridge to any specific one.

## Decision

We will provide **adapter-based integrations** for each major AI framework, maintaining a clean separation between the core bridge and framework-specific code.

### Architecture

```
┌─────────────────────────────────────────┐
│           AI Frameworks                 │
│  LangChain · AutoGPT · MCP · Custom    │
└──────────┬────────┬──────────┬──────────┘
           │        │          │
    ┌──────▼──┐ ┌───▼────┐ ┌──▼─────┐
    │LangChain│ │AutoGPT │ │  MCP   │
    │ Adapter │ │Adapter │ │Transport
    └────┬────┘ └───┬────┘ └──┬─────┘
         │          │         │
         └──────────┼─────────┘
                    │
         ┌──────────▼──────────┐
         │   Bridge Core       │
         │  (Framework Agnostic)│
         └─────────────────────┘
```

### Adapter Implementations

#### 1. LangChain Adapter

```python
class ROSBridgeTool(BaseTool):
    """LangChain tool for robot control."""
    name = "ros_bridge"
    description = "Control robots via ROS"
    
    def __init__(self, bridge: Bridge):
        self.bridge = bridge
    
    def _run(self, command: str) -> str:
        # Execute robot command
        return self.bridge.execute(command)
```

Usage:
```python
from agent_ros_bridge import Bridge
from agent_ros_bridge.integrations.langchain_adapter import ROSBridgeTool

bridge = Bridge()
tool = ROSBridgeTool(bridge)

# Use with any LangChain agent
agent = initialize_agent([tool], llm)
```

#### 2. AutoGPT Adapter

```python
class AutoGPTAdapter:
    """AutoGPT plugin adapter."""
    
    def __init__(self, bridge: Bridge):
        self.bridge = bridge
        self.commands = {
            "ros_move": self.cmd_move,
            "ros_get_status": self.cmd_get_status,
        }
    
    def cmd_move(self, direction: str, distance: float) -> str:
        return self.bridge.move(direction, distance)
```

#### 3. MCP Transport

```python
class MCPServerTransport:
    """MCP server for Claude Desktop integration."""
    
    def __init__(self, bridge: Bridge):
        self.bridge = bridge
        self.tools = self._discover_tools()
    
    async def handle_initialize(self, params):
        return {
            "tools": [
                {
                    "name": "ros_publish",
                    "description": "Publish to ROS topic",
                    "inputSchema": {...}
                }
            ]
        }
```

### Integration Points

The Bridge class provides factory methods:

```python
class Bridge:
    def get_langchain_tool(self) -> ROSBridgeTool:
        """Get LangChain-compatible tool."""
        from .integrations.langchain_adapter import ROSBridgeTool
        return ROSBridgeTool(self)
    
    def get_autogpt_adapter(self) -> AutoGPTAdapter:
        """Get AutoGPT adapter."""
        from .integrations.autogpt_adapter import AutoGPTAdapter
        return AutoGPTAdapter(self)
    
    def get_mcp_server(self) -> MCPServerTransport:
        """Get MCP server."""
        from .integrations.mcp_transport import MCPServerTransport
        return MCPServerTransport(self)
```

## Consequences

### Positive

- **Framework Agnostic** — Core bridge doesn't depend on any AI framework
- **User Choice** — Users choose their preferred framework
- **Extensibility** — Easy to add new framework adapters
- **Isolation** — Framework issues don't affect core bridge
- **Testing** — Can test bridge without AI frameworks installed

### Negative

- **Maintenance** — Multiple adapters to maintain
- **Documentation** — Must document each integration separately
- **Feature Parity** — Some features may not be available in all adapters

### Neutral

- **Dependencies** — Optional dependencies for each framework
- **Version Tracking** — Must track framework API changes

## Alternatives Considered

### Single Framework Only

**Rejected:** Would limit user base. Different users prefer different frameworks.

### Direct Integration

**Rejected:** Coupling bridge to specific frameworks would create bloat and maintenance issues.

### Generic API Only

**Rejected:** While we provide generic API, framework-specific adapters significantly improve developer experience.

## Implementation History

- **v0.4.x** — LangChain adapter only
- **v0.5.0** — AutoGPT adapter added
- **v0.5.0** — MCP transport added
- **v0.5.0** — All adapters properly integrated with Bridge class

## References

- [LangChain Adapter](../../agent_ros_bridge/integrations/langchain_adapter.py)
- [AutoGPT Adapter](../../agent_ros_bridge/integrations/autogpt_adapter.py)
- [MCP Transport](../../agent_ros_bridge/integrations/mcp_transport.py)
- [Integration Examples](../../examples/v0.5.0_integrations/)
