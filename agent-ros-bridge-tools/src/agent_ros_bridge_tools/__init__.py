"""Agent ROS Bridge Tools - ROS CLI tool ecosystem.

Provides familiar ROS tools (rostopic, rosservice, etc.) as Agent ROS Bridge
integrations. Compatible with NASA ROSA tool format (MIT licensed).

Example:
    >>> from agent_ros_bridge_tools import ROSTopicEchoTool
    >>> tool = ROSTopicEchoTool()
    >>> result = await tool.execute("/cmd_vel")

Available Tools:
    - ROSTopicEchoTool: Echo messages from ROS topics
    - ROSServiceCallTool: Call ROS services
    - ROSNodeListTool: List running ROS nodes
    - ROSParamGetTool: Get ROS parameters
    - ROSBagPlayTool: Play recorded bag files
"""

__version__ = "0.7.0.dev1"

from .base import Tool, ToolResult, register_tool
from .rostopic_echo import ROSTopicEchoTool
from .rosservice_call import ROSServiceCallTool
from .rosnode_list import ROSNodeListTool
from .rosparam_get import ROSParamGetTool
from .rosbag_play import ROSBagPlayTool

__all__ = [
    # Base
    "Tool",
    "ToolResult", 
    "register_tool",
    # Tools
    "ROSTopicEchoTool",
    "ROSServiceCallTool",
    "ROSNodeListTool",
    "ROSParamGetTool",
    "ROSBagPlayTool",
    "__version__",
]
