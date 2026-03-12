"""Unit tests for LangChain Adapter.

TDD tests for ROSBridgeTool, ROSAgent, LangChainAction.
"""
import pytest
from unittest.mock import Mock, AsyncMock, patch, MagicMock

from agent_ros_bridge.integrations.langchain_adapter import (
    ROSBridgeTool,
    ROSAgent,
    LangChainAction,
    LANGCHAIN_AVAILABLE,
)


class TestLangChainAction:
    """Test LangChainAction data class."""
    
    def test_action_creation(self):
        """LangChainAction can be created."""
        action = LangChainAction(
            name="navigate",
            description="Navigate to a position",
            parameters={"x": {"type": "float"}, "y": {"type": "float"}}
        )
        assert action.name == "navigate"
        assert action.description == "Navigate to a position"
        assert "x" in action.parameters


@pytest.mark.skipif(not LANGCHAIN_AVAILABLE, reason="langchain not available")
class TestROSBridgeTool:
    """Test ROSBridgeTool."""
    
    @pytest.fixture
    def mock_bridge(self):
        """Create a mock bridge."""
        bridge = Mock()
        bridge.execute_action = AsyncMock(return_value={"status": "success"})
        return bridge
    
    def test_tool_creation(self, mock_bridge):
        """ROSBridgeTool can be created."""
        tool = ROSBridgeTool(bridge=mock_bridge, actions=["navigate", "move_arm"])
        
        assert tool.bridge == mock_bridge
        assert tool.actions == ["navigate", "move_arm"]
        assert tool.name == "ros_bridge"
    
    def test_tool_creation_without_actions(self, mock_bridge):
        """ROSBridgeTool works without actions."""
        tool = ROSBridgeTool(bridge=mock_bridge)
        
        assert tool.actions == []
        assert "Control ROS robot" in tool.description
    
    def test_tool_builds_description(self, mock_bridge):
        """Tool builds description from actions."""
        tool = ROSBridgeTool(bridge=mock_bridge, actions=["navigate", "move_arm"])
        
        assert "navigate" in tool.description
        assert "move_arm" in tool.description
    
    def test_get_available_actions(self, mock_bridge):
        """Tool returns available actions."""
        tool = ROSBridgeTool(bridge=mock_bridge, actions=["navigate", "move_arm"])
        
        actions = tool.get_available_actions()
        assert actions == ["navigate", "move_arm"]
    
    def test_get_available_actions_returns_copy(self, mock_bridge):
        """Tool returns copy of actions list."""
        tool = ROSBridgeTool(bridge=mock_bridge, actions=["navigate"])
        
        actions = tool.get_available_actions()
        actions.append("new_action")
        
        # Original should not be modified
        assert tool.actions == ["navigate"]
    
    def test_parse_action_navigate(self, mock_bridge):
        """Tool parses navigate action from query."""
        tool = ROSBridgeTool(bridge=mock_bridge, actions=["navigate", "move_arm"])
        
        action = tool._parse_action("Navigate to position (5, 3)")
        assert action == "navigate"
    
    def test_parse_action_move(self, mock_bridge):
        """Tool parses move action from query."""
        tool = ROSBridgeTool(bridge=mock_bridge, actions=["navigate", "move_arm"])
        
        action = tool._parse_action("Move forward 1 meter")
        assert action == "navigate"
    
    def test_parse_action_arm(self, mock_bridge):
        """Tool parses arm action from query."""
        tool = ROSBridgeTool(bridge=mock_bridge, actions=["navigate", "move_arm"])
        
        action = tool._parse_action("Move arm to position X")
        assert action == "move_arm"
    
    def test_parse_action_status(self, mock_bridge):
        """Tool parses status action from query."""
        tool = ROSBridgeTool(bridge=mock_bridge, actions=["get_status", "navigate"])
        
        action = tool._parse_action("Get robot status")
        assert action == "get_status"
    
    def test_parse_action_default_to_first(self, mock_bridge):
        """Tool defaults to first action when no match."""
        tool = ROSBridgeTool(bridge=mock_bridge, actions=["navigate", "move_arm"])
        
        action = tool._parse_action("Do something random")
        assert action == "navigate"
    
    def test_parse_action_unknown_when_empty(self, mock_bridge):
        """Tool returns unknown when no actions defined."""
        tool = ROSBridgeTool(bridge=mock_bridge, actions=[])
        
        action = tool._parse_action("Do something")
        assert action == "unknown"
    
    @pytest.mark.asyncio
    async def test_arun_executes_action(self, mock_bridge):
        """Tool executes action via bridge."""
        tool = ROSBridgeTool(bridge=mock_bridge, actions=["navigate"])
        
        result = await tool._arun("Navigate to (5, 3)")
        
        mock_bridge.execute_action.assert_called_once()
        assert "success" in result
    
    @pytest.mark.asyncio
    async def test_arun_without_bridge(self):
        """Tool returns error when bridge not available."""
        tool = ROSBridgeTool(bridge=None, actions=["navigate"])
        
        result = await tool._arun("Navigate to (5, 3)")
        
        assert "error" in result
        assert "Bridge not available" in result
    
    @pytest.mark.asyncio
    async def test_arun_handles_exception(self, mock_bridge):
        """Tool handles execution exceptions."""
        mock_bridge.execute_action = AsyncMock(side_effect=Exception("Connection failed"))
        tool = ROSBridgeTool(bridge=mock_bridge, actions=["navigate"])
        
        result = await tool._arun("Navigate to (5, 3)")
        
        assert "error" in result
        assert "Connection failed" in result
    
    def test_run_sync_uses_async(self, mock_bridge):
        """Synchronous run uses async implementation."""
        tool = ROSBridgeTool(bridge=mock_bridge, actions=["navigate"])
        
        with patch.object(tool, '_arun') as mock_arun:
            mock_arun.return_value = '{"status": "success"}'
            
            result = tool._run("Navigate to (5, 3)")
            
            mock_arun.assert_called_once()
            assert result == '{"status": "success"}'


@pytest.mark.skipif(not LANGCHAIN_AVAILABLE, reason="langchain not available")
class TestROSAgent:
    """Test ROSAgent."""
    
    @pytest.fixture
    def mock_bridge(self):
        """Create a mock bridge."""
        return Mock()
    
    def test_agent_creation(self, mock_bridge):
        """ROSAgent can be created."""
        agent = ROSAgent(bridge=mock_bridge)
        
        assert agent.bridge == mock_bridge
        assert agent.llm is None
        assert agent.tool is not None
    
    def test_agent_creation_with_llm(self, mock_bridge):
        """ROSAgent can be created with LLM."""
        mock_llm = Mock()
        agent = ROSAgent(bridge=mock_bridge, llm=mock_llm)
        
        assert agent.llm == mock_llm
    
    def test_plan_task(self, mock_bridge):
        """Agent plans task into steps."""
        agent = ROSAgent(bridge=mock_bridge)
        
        steps = agent._plan_task("Navigate to kitchen")
        
        assert len(steps) == 1
        assert "Navigate to kitchen" in steps[0]
    
    @pytest.mark.asyncio
    async def test_run_executes_task(self, mock_bridge):
        """Agent executes task and returns results."""
        agent = ROSAgent(bridge=mock_bridge)
        
        # Mock the tool's _arun method
        agent.tool._arun = AsyncMock(return_value='{"status": "success"}')
        
        result = await agent.run("Navigate to kitchen")
        
        assert result["task"] == "Navigate to kitchen"
        assert len(result["steps"]) == 1
        assert len(result["results"]) == 1
        assert result["success"] is True
    
    @pytest.mark.asyncio
    async def test_run_detects_failure(self, mock_bridge):
        """Agent detects failed steps."""
        agent = ROSAgent(bridge=mock_bridge)
        
        # Mock the tool's _arun method to return error
        agent.tool._arun = AsyncMock(return_value='{"error": "Failed"}')
        
        result = await agent.run("Navigate to kitchen")
        
        assert result["success"] is False


class TestLangChainNotAvailable:
    """Test behavior when langchain is not available."""
    
    @patch("agent_ros_bridge.integrations.langchain_adapter.LANGCHAIN_AVAILABLE", False)
    def test_tool_creation_raises_import_error(self):
        """Tool creation raises ImportError when langchain not available."""
        mock_bridge = Mock()
        
        with pytest.raises(ImportError, match="langchain package required"):
            ROSBridgeTool(bridge=mock_bridge)
