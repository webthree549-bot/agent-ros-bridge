"""ROS connectors for gateway v2."""

# ROS1
try:
    from .ros1 import ROS1Connector, TopicInfo as ROS1TopicInfo
except ImportError:
    ROS1Connector = None
    ROS1TopicInfo = None

# ROS2
try:
    from .ros2 import ROS2Connector, TopicInfo as ROS2TopicInfo
except ImportError:
    ROS2Connector = None
    ROS2TopicInfo = None

# Export common TopicInfo (they're the same dataclass)
TopicInfo = ROS2TopicInfo or ROS1TopicInfo

__all__ = ["ROS1Connector", "ROS2Connector", "TopicInfo"]
