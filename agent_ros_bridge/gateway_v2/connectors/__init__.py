"""ROS1 and ROS2 connectors for robot communication.

This package provides connectors for ROS1 (Noetic) and ROS2 (Humble/Jazzy/Iron)
robot middleware systems, enabling unified access to robot capabilities.

Available connectors:
    - ROS1Connector: For ROS1 Noetic robots
    - ROS2Connector: For ROS2 Humble/Jazzy/Iron robots
"""

from agent_ros_bridge.gateway_v2.connectors.ros1_connector import ROS1Connector
from agent_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector

__all__ = ["ROS1Connector", "ROS2Connector"]
