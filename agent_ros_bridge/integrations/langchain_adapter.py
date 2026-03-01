"""LangChain Integration - Properly integrated into gateway_v2."""

import json
import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

try:
    from langchain.callbacks.manager import CallbackManagerForToolRun
    from langchain.tools import BaseTool

    LANGCHAIN_AVAILABLE = True
except ImportError:
    LANGCHAIN_AVAILABLE = False
    BaseTool = object
    CallbackManagerForToolRun = Any  # type: ignore

logger = logging.getLogger(__name__)


@dataclass
class LangChainAction:
    """Description of an action for LangChain."""

    name: str
    description: str
    parameters: Dict[str, Any]


class ROSBridgeTool(BaseTool):
    """LangChain Tool for controlling ROS robots.

    This integrates with gateway_v2 Bridge for actual execution.

    Example:
        from agent_ros_bridge.gateway_v2.core import Bridge
        from agent_ros_bridge.integrations.langchain_adapter import ROSBridgeTool

        bridge = Bridge()
        tool = ROSBridgeTool(
            bridge=bridge,
            actions=["navigate", "move_arm", "get_status"]
        )

        # Use with LangChain
        from langchain.agents import initialize_agent
        agent = initialize_agent([tool], llm, agent="zero-shot-react-description")
        agent.run("Navigate to position (5, 3)")
    """

    name: str = "ros_bridge"
    description: str = "Control ROS robots via the bridge"
    bridge: Any = None
    actions: List[str] = None

    def __init__(self, bridge, actions: Optional[List[str]] = None, **kwargs):
        if not LANGCHAIN_AVAILABLE:
            raise ImportError("langchain package required")

        super().__init__(**kwargs)
        self.bridge = bridge
        self.actions = actions or []

        # Build description from actions
        if self.actions:
            self.description = self._build_description()

    def _build_description(self) -> str:
        """Build tool description from available actions."""
        desc = "Control ROS robot with these actions:\n"
        for action in self.actions:
            desc += f"- {action}\n"
        desc += "\nUse specific action names as the 'action' parameter."
        return desc

    def _run(self, query: str, run_manager: Optional[CallbackManagerForToolRun] = None) -> str:
        """Execute tool (synchronous)."""
        import asyncio

        try:
            return asyncio.get_event_loop().run_until_complete(self._arun(query, run_manager))
        except RuntimeError:
            # No event loop
            loop = asyncio.new_event_loop()
            return loop.run_until_complete(self._arun(query, run_manager))

    async def _arun(
        self, query: str, run_manager: Optional[CallbackManagerForToolRun] = None
    ) -> str:
        """Execute tool (asynchronous)."""
        try:
            # Parse query to determine action
            # This is simplified - real implementation would use proper parsing
            action = self._parse_action(query)

            # Execute via bridge
            if self.bridge and hasattr(self.bridge, "execute_action"):
                result = await self.bridge.execute_action(action, {"query": query})
                return json.dumps(result)
            else:
                return json.dumps({"error": "Bridge not available"})

        except Exception as e:
            logger.error(f"Tool execution error: {e}")
            return json.dumps({"error": str(e)})

    def _parse_action(self, query: str) -> str:
        """Parse action from query."""
        # Simple keyword matching - real impl would be smarter
        query_lower = query.lower()

        if "navigate" in query_lower or "move" in query_lower:
            return "navigate"
        elif "arm" in query_lower:
            return "move_arm"
        elif "status" in query_lower:
            return "get_status"

        # Default to first available action
        return self.actions[0] if self.actions else "unknown"

    def get_available_actions(self) -> List[str]:
        """Get list of available actions."""
        return self.actions.copy()


class ROSAgent:
    """High-level agent with planning capabilities.

    Combines ROSBridgeTool with planning for complex tasks.
    """

    def __init__(self, bridge, llm=None):
        self.bridge = bridge
        self.llm = llm
        self.tool = ROSBridgeTool(bridge)
        logger.info("ROSAgent initialized")

    async def run(self, task: str) -> Dict[str, Any]:
        """Execute a task with planning.

        Example:
            agent = ROSAgent(bridge, llm)
            result = await agent.run("Pick up object at (3,4) and move to (5,2)")
        """
        # Simple implementation - would use LLM for planning in real version
        steps = self._plan_task(task)
        results = []

        for step in steps:
            result = await self.tool._arun(step)
            results.append({"step": step, "result": result})

        return {
            "task": task,
            "steps": steps,
            "results": results,
            "success": all(not r.get("error") for r in results),
        }

    def _plan_task(self, task: str) -> List[str]:
        """Break task into steps."""
        # Placeholder - real impl would use LLM
        return [f"Execute: {task}"]
