"""Tool Discovery - Auto-discover ROS tools and export to AI formats."""

import logging
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, asdict

logger = logging.getLogger(__name__)


@dataclass
class ROSAction:
    """Discovered ROS action."""
    name: str
    action_type: str  # "topic", "service", "action"
    ros_type: str
    description: str
    parameters: Dict[str, Any]
    dangerous: bool = False


class ToolDiscovery:
    """Discover ROS tools and export to AI-compatible formats.
    
    Example:
        discovery = ToolDiscovery(bridge)
        tools = discovery.discover_all()
        
        # Export to MCP format
        mcp_tools = discovery.to_mcp_tools(tools)
        
        # Export to OpenAI format
        openai_tools = discovery.to_openai_functions(tools)
    """
    
    def __init__(self, bridge=None):
        self.bridge = bridge
        self._cache: Dict[str, ROSAction] = {}
        logger.info("ToolDiscovery initialized")
    
    def discover_all(self) -> List[ROSAction]:
        """Discover all available ROS tools."""
        tools = []
        
        if self.bridge:
            # Discover from actual ROS
            tools.extend(self._discover_topics())
            tools.extend(self._discover_services())
            tools.extend(self._discover_actions())
        else:
            # Return cached or default
            tools = list(self._cache.values())
        
        # Update cache
        for tool in tools:
            self._cache[tool.name] = tool
        
        return tools
    
    def _discover_topics(self) -> List[ROSAction]:
        """Discover ROS topics."""
        topics = []
        # Implementation depends on ROS version
        # This is a placeholder
        return topics
    
    def _discover_services(self) -> List[ROSAction]:
        """Discover ROS services."""
        services = []
        # Implementation depends on ROS version
        return services
    
    def _discover_actions(self) -> List[ROSAction]:
        """Discover ROS actions."""
        actions = []
        # Implementation depends on ROS version
        return actions
    
    def to_mcp_tools(self, tools: Optional[List[ROSAction]] = None) -> List[Dict]:
        """Convert to MCP (Model Context Protocol) tool format."""
        if tools is None:
            tools = self.discover_all()
        
        mcp_tools = []
        for tool in tools:
            mcp_tool = {
                "name": tool.name,
                "description": tool.description,
                "inputSchema": {
                    "type": "object",
                    "properties": tool.parameters,
                    "required": list(tool.parameters.keys())
                }
            }
            mcp_tools.append(mcp_tool)
        
        return mcp_tools
    
    def to_openai_functions(self, tools: Optional[List[ROSAction]] = None) -> List[Dict]:
        """Convert to OpenAI function calling format."""
        if tools is None:
            tools = self.discover_all()
        
        functions = []
        for tool in tools:
            func = {
                "type": "function",
                "function": {
                    "name": tool.name,
                    "description": tool.description,
                    "parameters": {
                        "type": "object",
                        "properties": tool.parameters,
                        "required": list(tool.parameters.keys())
                    }
                }
            }
            functions.append(func)
        
        return functions
    
    def get_dangerous_tools(self) -> List[ROSAction]:
        """Get list of dangerous tools requiring confirmation."""
        all_tools = self.discover_all()
        return [t for t in all_tools if t.dangerous]
    
    def get_tool(self, name: str) -> Optional[ROSAction]:
        """Get a specific tool by name."""
        return self._cache.get(name)
    
    def invalidate_cache(self):
        """Invalidate tool cache."""
        self._cache.clear()
        logger.debug("Tool cache invalidated")
