"""Tests for LangChain adapter."""

from unittest.mock import AsyncMock, MagicMock, patch, Mock
import json

import pytest

from agent_ros_bridge.integrations.langchain_adapter import (
    LangChainAction,
    ROSBridgeTool,
    ROSAgent,
)


class TestLangChainAction:
    """Test LangChainAction dataclass."""

    def test_action_creation(self):
        """Test creating action."""
        action = LangChainAction(
            name="navigate",
            description="Navigate to position",
            parameters={"x": {"type": "number"}},
        )
        assert action.name == "navigate"
        assert action.description == "Navigate to position"


class TestROSBridgeTool:
    """Test ROSBridgeTool."""

    def test_init_without_langchain(self):
        """Test initialization without langchain."""
        with patch("agent_ros_bridge.integrations.langchain_adapter.LANGCHAIN_AVAILABLE", False):
            mock_bridge = Mock()
            with pytest.raises(ImportError, match="langchain"):
                ROSBridgeTool(mock_bridge)

    def test_init_with_langchain(self):
        """Test initialization with langchain."""
        with patch("agent_ros_bridge.integrations.langchain_adapter.LANGCHAIN_AVAILABLE", True):
            mock_bridge = Mock()
            tool = ROSBridgeTool(mock_bridge, actions=["navigate", "move"])
            assert tool.bridge is mock_bridge
            assert tool.actions == ["navigate", "move"]

    def test_build_description(self):
        """Test building tool description."""
        with patch("agent_ros_bridge.integrations.langchain_adapter.LANGCHAIN_AVAILABLE", True):
            mock_bridge = Mock()
            tool = ROSBridgeTool(mock_bridge, actions=["navigate", "move"])
            desc = tool._build_description()
            assert "navigate" in desc
            assert "move" in desc

    def test_get_available_actions(self):
        """Test getting available actions."""
        with patch("agent_ros_bridge.integrations.langchain_adapter.LANGCHAIN_AVAILABLE", True):
            mock_bridge = Mock()
            tool = ROSBridgeTool(mock_bridge, actions=["navigate", "move"])
            actions = tool.get_available_actions()
            assert actions == ["navigate", "move"]

    def test_parse_action_navigate(self):
        """Test parsing navigate action."""
        with patch("agent_ros_bridge.integrations.langchain_adapter.LANGCHAIN_AVAILABLE", True):
            mock_bridge = Mock()
            tool = ROSBridgeTool(mock_bridge, actions=["navigate", "move"])
            action = tool._parse_action("Navigate to position 5,3")
            assert action == "navigate"

    def test_parse_action_arm(self):
        """Test parsing arm action."""
        with patch("agent_ros_bridge.integrations.langchain_adapter.LANGCHAIN_AVAILABLE", True):
            mock_bridge = Mock()
            tool = ROSBridgeTool(mock_bridge, actions=["move_arm", "get_status"])
            # Use query without "move" to avoid matching navigate first
            action = tool._parse_action("Lift the arm up")
            # "arm" in query matches the keyword check for move_arm
            assert action == "move_arm"

    def test_parse_action_status(self):
        """Test parsing status action."""
        with patch("agent_ros_bridge.integrations.langchain_adapter.LANGCHAIN_AVAILABLE", True):
            mock_bridge = Mock()
            tool = ROSBridgeTool(mock_bridge, actions=["get_status", "navigate"])
            action = tool._parse_action("What is the status?")
            assert action == "get_status"

    def test_parse_action_default(self):
        """Test parsing with default action."""
        with patch("agent_ros_bridge.integrations.langchain_adapter.LANGCHAIN_AVAILABLE", True):
            mock_bridge = Mock()
            tool = ROSBridgeTool(mock_bridge, actions=["navigate"])
            action = tool._parse_action("Something random")
            assert action == "navigate"

    @pytest.mark.asyncio
    async def test_arun_success(self):
        """Test async execution success."""
        with patch("agent_ros_bridge.integrations.langchain_adapter.LANGCHAIN_AVAILABLE", True):
            mock_bridge = Mock()
            mock_bridge.execute_action = AsyncMock(return_value={"success": True})
            tool = ROSBridgeTool(mock_bridge, actions=["navigate"])
            
            result = await tool._arun("Navigate to 5,3")
            
            data = json.loads(result)
            assert data["success"] is True

    @pytest.mark.asyncio
    async def test_arun_no_bridge(self):
        """Test async execution without bridge."""
        with patch("agent_ros_bridge.integrations.langchain_adapter.LANGCHAIN_AVAILABLE", True):
            tool = ROSBridgeTool(None, actions=["navigate"])
            
            result = await tool._arun("Navigate to 5,3")
            
            data = json.loads(result)
            assert "error" in data

    @pytest.mark.asyncio
    async def test_arun_exception(self):
        """Test async execution with exception."""
        with patch("agent_ros_bridge.integrations.langchain_adapter.LANGCHAIN_AVAILABLE", True):
            mock_bridge = Mock()
            mock_bridge.execute_action = AsyncMock(side_effect=Exception("Failed"))
            tool = ROSBridgeTool(mock_bridge, actions=["navigate"])
            
            result = await tool._arun("Navigate to 5,3")
            
            data = json.loads(result)
            assert "error" in data


class TestROSAgent:
    """Test ROSAgent."""

    def test_init(self):
        """Test initialization."""
        with patch("agent_ros_bridge.integrations.langchain_adapter.LANGCHAIN_AVAILABLE", True):
            mock_bridge = Mock()
            agent = ROSAgent(mock_bridge)
            assert agent.bridge is mock_bridge
            assert agent.llm is None
            assert agent.tool is not None

    def test_init_with_llm(self):
        """Test initialization with LLM."""
        with patch("agent_ros_bridge.integrations.langchain_adapter.LANGCHAIN_AVAILABLE", True):
            mock_bridge = Mock()
            mock_llm = Mock()
            agent = ROSAgent(mock_bridge, llm=mock_llm)
            assert agent.llm is mock_llm

    @pytest.mark.asyncio
    async def test_run(self):
        """Test running agent."""
        with patch("agent_ros_bridge.integrations.langchain_adapter.LANGCHAIN_AVAILABLE", True):
            mock_bridge = Mock()
            agent = ROSAgent(mock_bridge)
            agent.tool._arun = AsyncMock(return_value=json.dumps({"success": True}))
            
            result = await agent.run("Navigate to 5,3")
            
            assert result["task"] == "Navigate to 5,3"
            assert result["success"] is True

    def test_plan_task(self):
        """Test task planning."""
        with patch("agent_ros_bridge.integrations.langchain_adapter.LANGCHAIN_AVAILABLE", True):
            mock_bridge = Mock()
            agent = ROSAgent(mock_bridge)
            steps = agent._plan_task("Do something complex")
            assert len(steps) == 1
            assert "Do something complex" in steps[0]
