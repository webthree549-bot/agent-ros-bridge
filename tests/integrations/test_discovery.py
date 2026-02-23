"""Tests for ToolDiscovery."""

import pytest
from agent_ros_bridge.integrations.discovery import ToolDiscovery, ROSAction


@pytest.fixture
def discovery():
    """Create tool discovery."""
    return ToolDiscovery()  # No bridge for unit tests


class TestToolDiscovery:
    """Test ToolDiscovery functionality."""
    
    def test_discover_all_empty(self, discovery):
        """Test discovery without bridge returns empty list."""
        tools = discovery.discover_all()
        assert isinstance(tools, list)
    
    def test_to_mcp_tools(self, discovery):
        """Test conversion to MCP format."""
        # Add a mock tool
        discovery._cache["test_action"] = ROSAction(
            name="test_action",
            action_type="topic",
            ros_type="std_msgs/String",
            description="Test action",
            parameters={"param1": "string"}
        )
        
        mcp_tools = discovery.to_mcp_tools()
        
        assert len(mcp_tools) == 1
        assert mcp_tools[0]["name"] == "test_action"
        assert "inputSchema" in mcp_tools[0]
    
    def test_to_openai_functions(self, discovery):
        """Test conversion to OpenAI format."""
        discovery._cache["test_action"] = ROSAction(
            name="test_action",
            action_type="topic",
            ros_type="std_msgs/String",
            description="Test action",
            parameters={}
        )
        
        functions = discovery.to_openai_functions()
        
        assert len(functions) == 1
        assert functions[0]["type"] == "function"
        assert "function" in functions[0]
    
    def test_cache_invalidation(self, discovery):
        """Test cache invalidation."""
        discovery._cache["test"] = "value"
        
        discovery.invalidate_cache()
        
        assert len(discovery._cache) == 0
