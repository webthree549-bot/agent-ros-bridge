# Communication Layers - Isolated ROS1/ROS2/OpenClaw with unified API
from openclaw_ros_bridge.communication.ros1_communicator import ROS1Communicator, ros1_comm
from openclaw_ros_bridge.communication.ros2_communicator import ROS2Communicator, ros2_comm
from openclaw_ros_bridge.communication.openclaw_communicator import OpenClawCommunicator, openclaw_comm
from openclaw_ros_bridge.communication.msg_converter import ROSMsgConverter, msg_converter

def get_ros_communicator():
    """
    Get the ROS1/ROS2 communicator based on VersionManager detection
    Returns unified API communicator (no ROS version checks needed)
    """
    from openclaw_ros_bridge.version.version_manager import version_manager
    return ros1_comm if version_manager.ROS_TYPE == "ros1" else ros2_comm

__all__ = [
    "get_ros_communicator",
    "ROS1Communicator",
    "ROS2Communicator",
    "OpenClawCommunicator",
    "ROSMsgConverter",
    "ros1_comm",
    "ros2_comm",
    "openclaw_comm",
    "msg_converter"
]
