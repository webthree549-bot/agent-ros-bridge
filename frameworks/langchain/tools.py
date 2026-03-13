"""LangChain integration for Agent ROS Bridge.

Provides LangChain tools for robot control.
"""

from typing import Any

from langchain.callbacks.manager import CallbackManagerForToolRun
from langchain.tools import BaseTool
from pydantic import BaseModel, Field


class ROS2PublishInput(BaseModel):
    """Input for ROS2 publish tool."""

    topic: str = Field(description="ROS topic name")
    message_type: str = Field(description="ROS message type (e.g., std_msgs/String)")
    data: dict[str, Any] = Field(description="Message data as dictionary")


class ROS2PublishTool(BaseTool):
    """Tool for publishing messages to ROS2 topics."""

    name: str = "ros2_publish"
    description: str = """Publish a message to a ROS2 topic.
    
    Use this to send commands to robots, such as:
    - Move commands: topic='/cmd_vel', message_type='geometry_msgs/Twist', data={'linear': {'x': 0.5}}
    - Gripper commands: topic='/gripper', message_type='std_msgs/String', data={'data': 'close'}
    """
    args_schema: type[BaseModel] = ROS2PublishInput

    def __init__(self, bridge_client):
        super().__init__()
        self.bridge_client = bridge_client

    def _run(
        self,
        topic: str,
        message_type: str,
        data: dict[str, Any],
        run_manager: CallbackManagerForToolRun | None = None,
    ) -> str:
        """Execute the tool."""
        try:
            result = self.bridge_client.publish(topic, message_type, data)
            return f"Published to {topic}: {result}"
        except Exception as e:
            return f"Error publishing to {topic}: {str(e)}"

    async def _arun(self, **kwargs) -> str:
        """Async execution."""
        return self._run(**kwargs)


class ROS2SubscribeInput(BaseModel):
    """Input for ROS2 subscribe tool."""

    topic: str = Field(description="ROS topic name")
    message_type: str = Field(description="ROS message type")
    timeout: float = Field(default=5.0, description="Timeout in seconds")


class ROS2SubscribeTool(BaseTool):
    """Tool for subscribing to ROS2 topics."""

    name: str = "ros2_subscribe"
    description: str = """Subscribe to a ROS2 topic and get the latest message.
    
    Use this to read sensor data, such as:
    - Laser scans: topic='/scan', message_type='sensor_msgs/LaserScan'
    - Camera images: topic='/camera/image_raw', message_type='sensor_msgs/Image'
    - Odometry: topic='/odom', message_type='nav_msgs/Odometry'
    """
    args_schema: type[BaseModel] = ROS2SubscribeInput

    def __init__(self, bridge_client):
        super().__init__()
        self.bridge_client = bridge_client

    def _run(
        self,
        topic: str,
        message_type: str,
        timeout: float = 5.0,
        run_manager: CallbackManagerForToolRun | None = None,
    ) -> str:
        """Execute the tool."""
        try:
            result = self.bridge_client.subscribe(topic, message_type, timeout)
            return f"Received from {topic}: {result}"
        except Exception as e:
            return f"Error subscribing to {topic}: {str(e)}"

    async def _arun(self, **kwargs) -> str:
        """Async execution."""
        return self._run(**kwargs)


class ROS2ActionInput(BaseModel):
    """Input for ROS2 action tool."""

    action_name: str = Field(description="Action server name")
    action_type: str = Field(description="Action type")
    goal: dict[str, Any] = Field(description="Goal parameters")
    timeout: float = Field(default=30.0, description="Timeout in seconds")


class ROS2ActionTool(BaseTool):
    """Tool for executing ROS2 actions."""

    name: str = "ros2_action"
    description: str = """Execute a ROS2 action.
    
    Use this for long-running tasks, such as:
    - Navigation: action_name='/navigate_to_pose', action_type='nav2_msgs/NavigateToPose'
    - Arm movement: action_name='/move_arm', action_type='moveit_msgs/MoveGroup'
    """
    args_schema: type[BaseModel] = ROS2ActionInput

    def __init__(self, bridge_client):
        super().__init__()
        self.bridge_client = bridge_client

    def _run(
        self,
        action_name: str,
        action_type: str,
        goal: dict[str, Any],
        timeout: float = 30.0,
        run_manager: CallbackManagerForToolRun | None = None,
    ) -> str:
        """Execute the tool."""
        try:
            result = self.bridge_client.execute_action(action_name, action_type, goal, timeout)
            return f"Action {action_name} completed: {result}"
        except Exception as e:
            return f"Error executing action {action_name}: {str(e)}"

    async def _arun(self, **kwargs) -> str:
        """Async execution."""
        return self._run(**kwargs)


class BridgeStatusInput(BaseModel):
    """Input for bridge status tool."""

    pass


class BridgeStatusTool(BaseTool):
    """Tool for checking bridge status."""

    name: str = "bridge_status"
    description: str = """Check the status of the Agent ROS Bridge.
    
    Returns information about connected robots, active sessions, and system health.
    """
    args_schema: type[BaseModel] = BridgeStatusInput

    def __init__(self, bridge_client):
        super().__init__()
        self.bridge_client = bridge_client

    def _run(self, run_manager: CallbackManagerForToolRun | None = None) -> str:
        """Execute the tool."""
        try:
            status = self.bridge_client.get_status()
            return f"Bridge status: {status}"
        except Exception as e:
            return f"Error getting status: {str(e)}"

    async def _arun(self, **kwargs) -> str:
        """Async execution."""
        return self._run(**kwargs)


class NaturalLanguageCommandInput(BaseModel):
    """Input for natural language command tool."""

    command: str = Field(description="Natural language command for the robot")
    robot_id: str | None = Field(default=None, description="Optional robot ID")


class NaturalLanguageCommandTool(BaseTool):
    """Tool for executing natural language robot commands."""

    name: str = "robot_command"
    description: str = """Execute a natural language command for a robot.
    
    Examples:
    - "move forward 1 meter"
    - "turn left 90 degrees"
    - "go to the kitchen"
    - "pick up the red object"
    - "take a photo"
    """
    args_schema: type[BaseModel] = NaturalLanguageCommandInput

    def __init__(self, bridge_client):
        super().__init__()
        self.bridge_client = bridge_client

    def _run(
        self,
        command: str,
        robot_id: str | None = None,
        run_manager: CallbackManagerForToolRun | None = None,
    ) -> str:
        """Execute the tool."""
        try:
            result = self.bridge_client.execute_natural_language(command, robot_id)
            return f"Command executed: {result}"
        except Exception as e:
            return f"Error executing command: {str(e)}"

    async def _arun(self, **kwargs) -> str:
        """Async execution."""
        return self._run(**kwargs)


# Convenience function to get all tools
def get_ros_tools(bridge_client):
    """Get all ROS-related LangChain tools.

    Args:
        bridge_client: Bridge client instance

    Returns:
        List of LangChain tools
    """
    return [
        ROS2PublishTool(bridge_client),
        ROS2SubscribeTool(bridge_client),
        ROS2ActionTool(bridge_client),
        BridgeStatusTool(bridge_client),
        NaturalLanguageCommandTool(bridge_client),
    ]
