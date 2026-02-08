# Communication Layers - Isolated ROS1/ROS2/OpenClaw with unified API
import os

# Conditional imports based on ROS version
ROS_TYPE = os.getenv("ROS_TYPE", "").lower()
if not ROS_TYPE:
    # Auto-detect from ROS_DISTRO
    ROS_DISTRO = os.getenv("ROS_DISTRO", "").lower()
    ROS_TYPE = "ros1" if ROS_DISTRO == "noetic" else "ros2"

# Import available communicators
try:
    from openclaw_ros_bridge.communication.ros1_communicator import ROS1Communicator, ros1_comm
except ImportError:
    ROS1Communicator = None
    ros1_comm = None

try:
    from openclaw_ros_bridge.communication.ros2_communicator import ROS2Communicator, ros2_comm
except ImportError:
    ROS2Communicator = None
    ros2_comm = None

from openclaw_ros_bridge.communication.openclaw_communicator import OpenClawCommunicator, openclaw_comm
from openclaw_ros_bridge.communication.msg_converter import ROSMsgConverter, msg_converter

def get_ros_communicator():
    """
    Get the ROS1/ROS2 communicator based on VersionManager detection
    Returns unified API communicator (no ROS version checks needed)
    """
    from openclaw_ros_bridge.version.version_manager import version_manager
    if version_manager.ROS_TYPE == "ros1" and ros1_comm is not None:
        return ros1_comm
    elif ros2_comm is not None:
        return ros2_comm
    raise RuntimeError("No ROS communicator available")

__all__ = [
    "get_ros_communicator",
    "OpenClawCommunicator",
    "ROSMsgConverter",
    "openclaw_comm",
    "msg_converter"
]

if ROS1Communicator is not None:
    __all__.extend(["ROS1Communicator", "ros1_comm"])
if ROS2Communicator is not None:
    __all__.extend(["ROS2Communicator", "ros2_comm"])
