"""Factory for creating action clients."""

import logging

from agent_ros_bridge.actions.base_client import BaseActionClient
from agent_ros_bridge.actions.ros2_client import ROS2ActionClient
from agent_ros_bridge.actions.simulated_client import SimulatedActionClient

logger = logging.getLogger("ros_actions")


def create_action_client(
    action_name: str, action_type: str, ros_version: str = "ros2"
) -> BaseActionClient:
    """Factory function to create appropriate action client.

    Args:
        action_name: Name of the ROS action.
        action_type: Type of the action.
        ros_version: ROS version ("ros1" or "ros2").

    Returns:
        BaseActionClient: The created action client.
    """
    # Try to use real ROS client
    if ros_version == "ros2":
        try:
            import rclpy  # noqa: F401

            return ROS2ActionClient(action_name, action_type)
        except ImportError:
            logger.warning("rclpy not available, using simulated action client")
            return SimulatedActionClient(action_name, action_type)
    else:
        # ROS1 not yet implemented
        logger.warning("ROS1 action client not implemented, using simulated")
        return SimulatedActionClient(action_name, action_type)
