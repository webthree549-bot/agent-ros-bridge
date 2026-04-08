"""ROS Action Client for Agent ROS Bridge.

Supports ROS Actions (ROS1 actionlib, ROS2 rclpy actions) for long-running
tasks like navigation, motion planning, and manipulation.

Features:
- Send action goals with timeouts
- Receive feedback during execution
- Get final results
- Cancel/preempt ongoing actions
- Support for both ROS1 and ROS2

Usage:
    from agent_ros_bridge.actions import create_action_client

    client = create_action_client("navigate_to_pose", "nav2_msgs/action/NavigateToPose")
    result = await client.send_goal({"pose": {...}}, feedback_cb=on_feedback)
"""

from agent_ros_bridge.actions.base_client import BaseActionClient
from agent_ros_bridge.actions.factory import create_action_client
from agent_ros_bridge.actions.ros2_client import ROS2ActionClient
from agent_ros_bridge.actions.simulated_client import SimulatedActionClient
from agent_ros_bridge.actions.types import (
    ActionFeedback,
    ActionGoal,
    ActionResult,
    ActionStatus,
)

__all__ = [
    # Clients
    "BaseActionClient",
    "ROS2ActionClient",
    "SimulatedActionClient",
    # Factory
    "create_action_client",
    # Types
    "ActionStatus",
    "ActionGoal",
    "ActionFeedback",
    "ActionResult",
]
