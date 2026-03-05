"""LangChain client for Agent ROS Bridge.

Provides easy integration with LangChain agents.
"""

from typing import Dict, Any, Optional, List
import requests


class AgentROSBridgeClient:
    """Client for connecting to Agent ROS Bridge.
    
    Example:
        >>> from agent_ros_bridge.frameworks.langchain import AgentROSBridgeClient
        >>> client = AgentROSBridgeClient("http://localhost:8765", token="your-jwt")
        >>> client.publish("/cmd_vel", "geometry_msgs/Twist", {"linear": {"x": 0.5}})
    """
    
    def __init__(
        self,
        base_url: str = "http://localhost:8765",
        token: Optional[str] = None,
        timeout: float = 30.0
    ):
        """Initialize client.
        
        Args:
            base_url: Bridge URL
            token: JWT authentication token
            timeout: Request timeout
        """
        self.base_url = base_url.rstrip('/')
        self.token = token
        self.timeout = timeout
        self.session = requests.Session()
        
        if token:
            self.session.headers["Authorization"] = f"Bearer {token}"
    
    def publish(
        self,
        topic: str,
        message_type: str,
        data: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Publish message to ROS topic.
        
        Args:
            topic: ROS topic name
            message_type: ROS message type
            data: Message data
            
        Returns:
            Response from bridge
        """
        response = self.session.post(
            f"{self.base_url}/api/v1/publish",
            json={
                "topic": topic,
                "message_type": message_type,
                "data": data
            },
            timeout=self.timeout
        )
        response.raise_for_status()
        return response.json()
    
    def subscribe(
        self,
        topic: str,
        message_type: str,
        timeout: float = 5.0
    ) -> Dict[str, Any]:
        """Subscribe to ROS topic.
        
        Args:
            topic: ROS topic name
            message_type: ROS message type
            timeout: Subscription timeout
            
        Returns:
            Latest message data
        """
        response = self.session.post(
            f"{self.base_url}/api/v1/subscribe",
            json={
                "topic": topic,
                "message_type": message_type,
                "timeout": timeout
            },
            timeout=timeout + 5
        )
        response.raise_for_status()
        return response.json()
    
    def execute_action(
        self,
        action_name: str,
        action_type: str,
        goal: Dict[str, Any],
        timeout: float = 30.0
    ) -> Dict[str, Any]:
        """Execute ROS action.
        
        Args:
            action_name: Action server name
            action_type: Action type
            goal: Goal parameters
            timeout: Action timeout
            
        Returns:
            Action result
        """
        response = self.session.post(
            f"{self.base_url}/api/v1/action",
            json={
                "action_name": action_name,
                "action_type": action_type,
                "goal": goal,
                "timeout": timeout
            },
            timeout=timeout + 5
        )
        response.raise_for_status()
        return response.json()
    
    def execute_natural_language(
        self,
        command: str,
        robot_id: Optional[str] = None
    ) -> Dict[str, Any]:
        """Execute natural language command.
        
        Args:
            command: Natural language command
            robot_id: Optional robot ID
            
        Returns:
            Command result
        """
        payload = {"command": command}
        if robot_id:
            payload["robot_id"] = robot_id
        
        response = self.session.post(
            f"{self.base_url}/api/v1/nl",
            json=payload,
            timeout=self.timeout
        )
        response.raise_for_status()
        return response.json()
    
    def get_status(self) -> Dict[str, Any]:
        """Get bridge status.
        
        Returns:
            Status information
        """
        response = self.session.get(
            f"{self.base_url}/api/v1/status",
            timeout=self.timeout
        )
        response.raise_for_status()
        return response.json()
    
    def list_robots(self) -> List[Dict[str, Any]]:
        """List connected robots.
        
        Returns:
            List of robot information
        """
        response = self.session.get(
            f"{self.base_url}/api/v1/robots",
            timeout=self.timeout
        )
        response.raise_for_status()
        return response.json().get("robots", [])
    
    def get_robot_info(self, robot_id: str) -> Dict[str, Any]:
        """Get robot information.
        
        Args:
            robot_id: Robot identifier
            
        Returns:
            Robot information
        """
        response = self.session.get(
            f"{self.base_url}/api/v1/robots/{robot_id}",
            timeout=self.timeout
        )
        response.raise_for_status()
        return response.json()


# Convenience imports
from .tools import (
    ROS2PublishTool,
    ROS2SubscribeTool,
    ROS2ActionTool,
    BridgeStatusTool,
    NaturalLanguageCommandTool,
    get_ros_tools,
)

__all__ = [
    "AgentROSBridgeClient",
    "ROS2PublishTool",
    "ROS2SubscribeTool",
    "ROS2ActionTool",
    "BridgeStatusTool",
    "NaturalLanguageCommandTool",
    "get_ros_tools",
]
