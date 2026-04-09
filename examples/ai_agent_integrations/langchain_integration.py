"""LangChain Integration Example for Agent ROS Bridge.

Demonstrates how LangChain agents can use Agent ROS Bridge
to interact with ROS robots via natural language.
"""

import asyncio
from typing import Any

from langchain.agents import AgentExecutor, create_openai_functions_agent
from langchain.tools import BaseTool, StructuredTool
from langchain_core.messages import SystemMessage
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_openai import ChatOpenAI

from agent_ros_bridge.agentic import RobotAgent
from agent_ros_bridge.tools import ROSTopicEchoTool, ROSServiceCallTool


class ROSBridgeTool(BaseTool):
    """LangChain tool wrapper for Agent ROS Bridge."""
    
    name: str = "ros_bridge"
    description: str = """
    Use this tool to control ROS robots and interact with ROS topics/services.
    
    Available commands:
    - "navigate to [location]": Navigate robot to a location
    - "get topic [topic_name]": Get data from a ROS topic
    - "call service [service_name]": Call a ROS service
    - "get robot status": Get current robot status
    
    Safety: All commands go through human-in-the-loop validation.
    """
    
    def __init__(self, robot_agent: RobotAgent):
        super().__init__()
        self.robot_agent = robot_agent
        self.topic_tool = ROSTopicEchoTool()
        self.service_tool = ROSServiceCallTool()
    
    def _run(self, command: str) -> str:
        """Execute ROS command."""
        command = command.lower().strip()
        
        # Navigation commands
        if "navigate" in command or "go to" in command or "move to" in command:
            return self._handle_navigation(command)
        
        # Topic commands
        if "topic" in command or "get" in command:
            return self._handle_topic(command)
        
        # Service commands
        if "service" in command or "call" in command:
            return self._handle_service(command)
        
        # Status commands
        if "status" in command:
            return self._handle_status()
        
        return f"Unknown command: {command}. Try: navigate, topic, service, or status"
    
    def _handle_navigation(self, command: str) -> str:
        """Handle navigation commands."""
        # Extract location from command
        # "navigate to kitchen" -> location: kitchen
        parts = command.split(" to ") if " to " in command else command.split()
        location = parts[-1] if len(parts) > 1 else "unknown"
        
        # Execute through RobotAgent (with safety)
        result = self.robot_agent.execute(f"Navigate to {location}")
        
        if result.success:
            return f"✅ Successfully navigated to {location}. " \
                   f"Execution time: {result.duration_seconds:.1f}s"
        else:
            return f"❌ Navigation failed: {result.message}"
    
    def _handle_topic(self, command: str) -> str:
        """Handle topic commands."""
        # Extract topic name
        # "get topic /cmd_vel" -> topic: /cmd_vel
        words = command.split()
        topic = None
        for i, word in enumerate(words):
            if word.startswith("/"):
                topic = word
                break
        
        if not topic:
            return "❌ No topic specified. Usage: get topic /topic_name"
        
        result = self.topic_tool.execute(topic=topic, count=1)
        
        if result.success:
            return f"✅ Topic {topic} data:\n{result.output}"
        else:
            return f"❌ Failed to get topic: {result.error}"
    
    def _handle_service(self, command: str) -> str:
        """Handle service commands."""
        words = command.split()
        service = None
        for word in words:
            if word.startswith("/"):
                service = word
                break
        
        if not service:
            return "❌ No service specified. Usage: call service /service_name"
        
        result = self.service_tool.execute(service=service)
        
        if result.success:
            return f"✅ Service {service} called:\n{result.output}"
        else:
            return f"❌ Service call failed: {result.error}"
    
    def _handle_status(self) -> str:
        """Handle status commands."""
        status = self.robot_agent.get_state()
        return f"""
🤖 Robot Status:
- Device: {status.get('device_id', 'unknown')}
- Type: {status.get('device_type', 'unknown')}
- Safety: {self.robot_agent.safety.safety_validation_status}
- Shadow Hours: {self.robot_agent.safety.shadow_mode_hours_collected:.1f}/200
        """.strip()
    
    async def _arun(self, command: str) -> str:
        """Async execution."""
        return self._run(command)


def create_ros_agent(openai_api_key: str = None) -> AgentExecutor:
    """Create a LangChain agent with ROS Bridge capabilities.
    
    Example:
        agent = create_ros_agent()
        result = agent.invoke({"input": "Navigate to the kitchen"})
    """
    # Initialize RobotAgent with safety
    robot_agent = RobotAgent(
        device_id="langchain_bot",
        device_type="mobile_robot",
        require_confirmation=True,  # Safety enforced
    )
    
    # Create ROS Bridge tool
    ros_tool = ROSBridgeTool(robot_agent)
    
    # Setup LangChain
    llm = ChatOpenAI(
        model="gpt-4",
        temperature=0,
        api_key=openai_api_key,
    )
    
    # Create system prompt with safety context
    system_prompt = """You are a ROS robot control assistant.
    
Your job is to help users control robots using natural language.
You have access to a ROS Bridge that can:
- Navigate robots to locations
- Read ROS topic data
- Call ROS services
- Check robot status

SAFETY FIRST:
- All commands require human confirmation
- The system uses shadow mode to learn from corrections
- Emergency stop is always available
- Gradual rollout from 0% to 100% autonomy

When the user asks you to do something:
1. Interpret their natural language request
2. Use the ros_bridge tool to execute
3. Report back the results clearly

If something fails, explain why and suggest alternatives.
    """
    
    prompt = ChatPromptTemplate.from_messages([
        ("system", system_prompt),
        MessagesPlaceholder(variable_name="chat_history"),
        ("human", "{input}"),
        MessagesPlaceholder(variable_name="agent_scratchpad"),
    ])
    
    # Create agent
    agent = create_openai_functions_agent(llm, [ros_tool], prompt)
    
    return AgentExecutor(
        agent=agent,
        tools=[ros_tool],
        verbose=True,
        handle_parsing_errors=True,
    )


# Example usage
if __name__ == "__main__":
    # Create agent
    agent = create_ros_agent()
    
    # Example interactions
    examples = [
        "Navigate to the kitchen",
        "What's the robot's current status?",
        "Get data from /cmd_vel topic",
        "Call the /clear_costmap service",
        "Move to position x=1.0, y=2.0",
    ]
    
    for example in examples:
        print(f"\n📝 User: {example}")
        result = agent.invoke({"input": example, "chat_history": []})
        print(f"🤖 Agent: {result['output']}")
