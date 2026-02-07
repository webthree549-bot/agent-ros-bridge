#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS Msg Converter - Version-agnostic ROS1/ROS2 message conversion"""
import json
from typing import Any, Dict, Optional
from openclaw_ros_bridge.base.logger import get_logger
from openclaw_ros_bridge.version.version_manager import version_manager

logger = get_logger(__name__)

class ROSMsgConverter:
    """ROS1/ROS2 Message Converter - Auto-detects ROS version"""
    def __init__(self):
        self.ros_type = version_manager.ROS_TYPE
        self.ros_distro = version_manager.ROS_DISTRO

    def ros2json(self, ros_msg: Any) -> Dict[str, Any]:
        """Convert ROS msg to JSON (works with both ROS1/ROS2)"""
        if ros_msg is None:
            return {}
        msg_dict = {"_ros_msg_type": type(ros_msg).__name__, "_ros_distro": self.ros_distro}
        for attr in dir(ros_msg):
            if not attr.startswith("_") and not callable(getattr(ros_msg, attr)):
                try:
                    val = getattr(ros_msg, attr)
                    # Handle nested ROS messages
                    if hasattr(val, '__class__') and hasattr(val, '__slots__'):
                        msg_dict[attr] = self.ros2json(val)
                    else:
                        msg_dict[attr] = val
                except Exception as e:
                    logger.debug(f"Skipping attr {attr}: {e}")
        return msg_dict

    def json2ros(self, data: Dict[str, Any], ros_msg_type: str, target_ros_type: Optional[str] = None) -> Any:
        """Convert JSON to ROS msg (version-agnostic)"""
        target = target_ros_type or self.ros_type
        try:
            if target == "ros1":
                return self._json2ros1(data, ros_msg_type)
            else:
                return self._json2ros2(data, ros_msg_type)
        except Exception as e:
            logger.error(f"JSON to ROS conversion failed: {e}")
            return data

    def _json2ros1(self, data: Dict[str, Any], msg_type: str) -> Any:
        """Convert JSON to ROS1 message"""
        try:
            # Dynamically import ROS1 message type
            module_name, class_name = msg_type.rsplit('/', 1) if '/' in msg_type else ('std_msgs.msg', msg_type)
            module = __import__(module_name, fromlist=[class_name])
            msg_class = getattr(module, class_name)
            msg = msg_class()
            for key, value in data.items():
                if key.startswith('_'):
                    continue
                if hasattr(msg, key):
                    setattr(msg, key, value)
            return msg
        except Exception as e:
            logger.error(f"ROS1 message conversion failed: {e}")
            return data

    def _json2ros2(self, data: Dict[str, Any], msg_type: str) -> Any:
        """Convert JSON to ROS2 message"""
        try:
            # Dynamically import ROS2 message type
            module_name, class_name = msg_type.rsplit('/', 1) if '/' in msg_type else ('std_msgs.msg', msg_type)
            module = __import__(module_name, fromlist=[class_name])
            msg_class = getattr(module, class_name)
            msg = msg_class()
            for key, value in data.items():
                if key.startswith('_'):
                    continue
                if hasattr(msg, key):
                    setattr(msg, key, value)
            return msg
        except Exception as e:
            logger.error(f"ROS2 message conversion failed: {e}")
            return data

msg_converter = ROSMsgConverter()