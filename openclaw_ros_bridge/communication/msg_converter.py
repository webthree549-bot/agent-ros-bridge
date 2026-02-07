#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS Msg Converter - Auto-convert ROS1 ↔ ROS2 messages"""
import json
from typing import Any, Dict
from openclaw_ros_bridge.base.logger import get_logger

logger = get_logger(__name__)

class ROSMsgConverter:
    """ROS1/ROS2 Message Converter"""
    def __init__(self):
        pass

    def ros2json(self, ros_msg: Any) -> Dict[str, Any]:
        """Convert ROS msg to JSON"""
        msg_dict = {}
        for attr in dir(ros_msg):
            if not attr.startswith("_") and not callable(getattr(ros_msg, attr)):
                msg_dict[attr] = getattr(ros_msg, attr)
        return msg_dict

    def json2ros(self, data: Dict[str, Any], ros_msg_type: str) -> Any:
        """Convert JSON to ROS msg"""
        return data

msg_converter = ROSMsgConverter()