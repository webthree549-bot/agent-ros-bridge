"""AI Agent Integrations for Agent ROS Bridge v0.5.0.

This package contains integrations with popular AI frameworks,
properly wired into the gateway_v2 architecture.

Modules:
    - memory: Agent memory system (SQLite/Redis)
    - safety: Action confirmation and safety levels
    - discovery: Tool discovery from ROS
    - langchain_adapter: LangChain Tool and Agent adapters
    - autogpt_adapter: AutoGPT plugin adapter
    - mcp_transport: Model Context Protocol server transport
    - dashboard_server: Real-time monitoring dashboard

Example:
    from agent_ros_bridge.gateway_v2.core import Bridge
    from agent_ros_bridge.integrations import (
        AgentMemory, SafetyManager, ToolDiscovery,
        ROSBridgeTool, AutoGPTAdapter, MCPServerTransport
    )

    # Create bridge with AI features
    bridge = Bridge()
    bridge.memory = AgentMemory()
    bridge.safety = SafetyManager()
    bridge.tools = ToolDiscovery(bridge)
"""

__version__ = "0.5.0"

from .autogpt_adapter import AutoGPTAdapter
from .dashboard_server import DashboardServer
from .discovery import ROSAction, ToolDiscovery
from .langchain_adapter import ROSAgent, ROSBridgeTool
from .mcp_transport import MCPServerTransport
from .memory import AgentMemory
from .safety import SafetyLevel, SafetyManager

__all__ = [
    "AgentMemory",
    "SafetyManager",
    "SafetyLevel",
    "ToolDiscovery",
    "ROSAction",
    "ROSBridgeTool",
    "ROSAgent",
    "AutoGPTAdapter",
    "MCPServerTransport",
    "DashboardServer",
]
