"""LangChain integration for Agent ROS Bridge.

Provides ROSBridge as a LangChain Tool for easy integration with LangChain agents.

Example:
    from langchain.agents import initialize_agent, Tool
    from agent_ros_bridge.langchain import ROSBridgeTool
    
    tools = [ROSBridgeTool(bridge)]
    agent = initialize_agent(tools, llm, agent="zero-shot-react-description")
    agent.run("Navigate the robot to position (5, 3)")
"""

import json
import logging
from typing import Any, Dict, List, Optional, Type
from dataclasses import dataclass

try:
    from langchain.tools import BaseTool
    from langchain.callbacks.manager import CallbackManagerForToolRun, AsyncCallbackManagerForToolRun
    LANGCHAIN_AVAILABLE = True
except ImportError:
    LANGCHAIN_AVAILABLE = False
    BaseTool = object

from agent_ros_bridge import ROSBridge
from agent_ros_bridge.discovery import ToolDiscovery, SafetyLevel

logger = logging.getLogger(__name__)


@dataclass
class LangChainAction:
    """Description of an action for LangChain."""
    name: str
    description: str
    parameters: Dict[str, Any]
    safety_level: SafetyLevel


class ROSBridgeTool(BaseTool):
    """LangChain Tool for controlling ROS robots.
    
    This tool wraps the ROSBridge and exposes robot actions as LangChain tools.
    
    Example:
        tool = ROSBridgeTool(
            bridge=bridge,
            actions=["navigate", "move_arm", "get_status"]
        )
        
        # Or auto-discover all actions
        tool = ROSBridgeTool.from_bridge(bridge)
    """
    
    name: str = "ros"
    description: str = """Control a ROS robot. Available actions include:
    - navigate: Move to coordinates (x, y, theta)
    - move_arm: Move robotic arm to position
    - grasp: Control gripper
    - get_status: Get robot status
    - get_battery: Check battery level
    Use specific action names as the 'action' parameter.
    """
    
    bridge: Optional[ROSBridge] = None
    discovery: Optional[ToolDiscovery] = None
    _actions: Dict[str, LangChainAction] = {}
    
    def __init__(self, bridge: ROSBridge, actions: Optional[List[str]] = None, **kwargs):
        """Initialize ROSBridgeTool.
        
        Args:
            bridge: ROSBridge instance
            actions: List of action names to expose (None for all)
            **kwargs: Additional BaseTool arguments
        """
        if not LANGCHAIN_AVAILABLE:
            raise ImportError(
                "LangChain integration requires langchain: "
                "pip install langchain"
            )
        
        super().__init__(**kwargs)
        self.bridge = bridge
        self.discovery = ToolDiscovery(bridge)
        
        # Discover and filter actions
        if actions:
            self._register_specific_actions(actions)
        else:
            self._register_all_actions()
    
    @classmethod
    async def from_bridge(cls, bridge: ROSBridge, **kwargs) -> "ROSBridgeTool":
        """Create tool with auto-discovered actions.
        
        Args:
            bridge: ROSBridge instance
            **kwargs: Additional arguments
            
        Returns:
            Configured ROSBridgeTool
        """
        tool = cls(bridge, actions=None, **kwargs)
        await tool.discovery.discover_all()
        return tool
    
    def _register_specific_actions(self, actions: List[str]):
        """Register specific actions."""
        for action_name in actions:
            action = LangChainAction(
                name=action_name,
                description=f"Execute {action_name} action",
                parameters={},
                safety_level=SafetyLevel.MEDIUM
            )
            self._actions[action_name] = action
        
        # Update description
        self.description = self._generate_description()
    
    def _register_all_actions(self):
        """Register all available actions."""
        registered = self.bridge.get_registered_actions()
        for action_name in registered:
            action = LangChainAction(
                name=action_name,
                description=f"Execute {action_name} action",
                parameters={},
                safety_level=SafetyLevel.MEDIUM
            )
            self._actions[action_name] = action
        
        # Update description
        self.description = self._generate_description()
    
    def _generate_description(self) -> str:
        """Generate tool description from available actions."""
        action_list = ", ".join(self._actions.keys())
        return f"""Control a ROS robot. Available actions: {action_list}.
        
        Use the 'action' parameter to specify which action to execute.
        Each action may require different parameters.
        Dangerous actions like navigation and arm movement require careful consideration.
        """
    
    def _run(
        self,
        query: str,
        run_manager: Optional[CallbackManagerForToolRun] = None
    ) -> str:
        """Synchronous execution (not recommended for ROS).
        
        Args:
            query: JSON string with action and parameters
            
        Returns:
            Result as string
        """
        # ROS is async, so this runs async in sync context
        import asyncio
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        
        return loop.run_until_complete(self._arun(query, run_manager))
    
    async def _arun(
        self,
        query: str,
        run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Async execution of ROS action.
        
        Args:
            query: JSON string with action and parameters
            
        Returns:
            Result as string
        """
        try:
            # Parse query
            params = json.loads(query)
            action = params.get("action")
            
            if not action:
                return "Error: No action specified. Use 'action' field."
            
            if action not in self._actions:
                available = ", ".join(self._actions.keys())
                return f"Error: Unknown action '{action}'. Available: {available}"
            
            # Remove action from params
            action_params = {k: v for k, v in params.items() if k != "action"}
            
            # Check safety
            action_info = self._actions[action]
            if action_info.safety_level == SafetyLevel.DANGEROUS:
                if run_manager:
                    await run_manager.on_tool_error(
                        f"Dangerous action '{action}' requires confirmation"
                    )
            
            # Execute action
            result = await self.bridge.call_action(action, **action_params)
            
            if result.get("success"):
                return json.dumps(result.get("result", "Success"))
            else:
                return f"Error: {result.get('error', 'Unknown error')}"
        
        except json.JSONDecodeError:
            return "Error: Invalid JSON. Use format: '{\"action\": \"navigate\", \"x\": 5, \"y\": 3}'"
        except Exception as e:
            logger.error(f"LangChain tool error: {e}")
            return f"Error: {str(e)}"
    
    def get_actions_schema(self) -> Dict[str, Any]:
        """Get JSON schema for all available actions.
        
        Returns:
            Dictionary mapping action names to their schemas
        """
        return {
            name: {
                "description": action.description,
                "parameters": action.parameters
            }
            for name, action in self._actions.items()
        }


class ROSAgent:
    """High-level LangChain agent for ROS robots.
    
    Combines multiple ROS tools into a complete agent.
    
    Example:
        agent = ROSAgent(bridge, llm)
        result = await agent.run("Pick up the object at (3, 4) and place it at (8, 2)")
    """
    
    def __init__(self, bridge: ROSBridge, llm, **kwargs):
        """Initialize ROS agent.
        
        Args:
            bridge: ROSBridge instance
            llm: LangChain LLM
            **kwargs: Additional agent configuration
        """
        if not LANGCHAIN_AVAILABLE:
            raise ImportError("LangChain required: pip install langchain")
        
        self.bridge = bridge
        self.llm = llm
        self.tools = self._create_tools()
        self.agent = self._create_agent(**kwargs)
    
    def _create_tools(self) -> List[BaseTool]:
        """Create list of ROS tools."""
        return [
            ROSBridgeTool(self.bridge),
        ]
    
    def _create_agent(self, **kwargs):
        """Create LangChain agent."""
        from langchain.agents import initialize_agent, AgentType
        
        return initialize_agent(
            self.tools,
            self.llm,
            agent=kwargs.get("agent_type", AgentType.ZERO_SHOT_REACT_DESCRIPTION),
            verbose=kwargs.get("verbose", True),
            **{k: v for k, v in kwargs.items() if k not in ["agent_type", "verbose"]}
        )
    
    async def run(self, query: str) -> str:
        """Run agent with query.
        
        Args:
            query: Natural language instruction
            
        Returns:
            Agent response
        """
        return await self.agent.arun(query)
    
    async def plan(self, goal: str) -> List[Dict]:
        """Generate plan to achieve goal.
        
        Args:
            goal: High-level goal
            
        Returns:
            List of planned actions
        """
        # Use LLM to generate plan
        prompt = f"""Given the goal: "{goal}"
        
        Break this down into specific robot actions from this list:
        {", ".join(self.bridge.get_registered_actions())}
        
        Return as JSON list: [{{"action": "...", "parameters": {{...}}}}]"""
        
        response = await self.llm.apredict(prompt)
        
        try:
            return json.loads(response)
        except json.JSONDecodeError:
            logger.error(f"Failed to parse plan: {response}")
            return []


# Utility functions
def create_ros_tool(bridge: ROSBridge, **kwargs) -> ROSBridgeTool:
    """Create a ROS tool for LangChain.
    
    Args:
        bridge: ROSBridge instance
        **kwargs: Tool configuration
        
    Returns:
        Configured ROSBridgeTool
    """
    return ROSBridgeTool(bridge, **kwargs)


__all__ = [
    "ROSBridgeTool",
    "ROSAgent",
    "LangChainAction",
    "create_ros_tool"
]
