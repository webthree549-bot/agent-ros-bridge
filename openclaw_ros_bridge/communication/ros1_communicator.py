#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS1 Communicator - Unified API wrapper for rospy (ROS1 Noetic)"""
import rospy
from typing import Callable, Type, Dict, Any, Optional
from openclaw_ros_bridge.base.logger import get_logger
from openclaw_ros_bridge.version.version_manager import version_manager

logger = get_logger(__name__)

class ROS1Communicator:
    """ROS1 Noetic Communicator"""
    def __init__(self, node_name: Optional[str] = None):
        self.ros_distro = version_manager.ROS_DISTRO
        self.node_name = node_name or f"ros1_openclaw_bridge_{self.ros_distro}"
        self._publishers: Dict[str, rospy.Publisher] = {}
        self._subscribers: Dict[str, rospy.Subscriber] = {}
        self._qos = version_manager.get_ros_param("default_qos", 10)
        self._node_initialized = False

    @property
    def is_initialized(self) -> bool:
        return self._node_initialized and rospy.core.is_initialized()

    def _init_node(self) -> None:
        if not self.is_initialized:
            rospy.init_node(self.node_name, anonymous=False)
            self._node_initialized = True
            logger.info(f"ROS1 node initialized: {self.node_name}")

    def subscribe(self, topic_name: str, msg_type: Type, callback: Callable, qos_profile: Optional[int] = None) -> None:
        self._init_node()
        qos = qos_profile or self._qos
        if topic_name not in self._subscribers:
            self._subscribers[topic_name] = rospy.Subscriber(topic_name, msg_type, callback, queue_size=qos)
            logger.info(f"ROS1 subscribed: {topic_name}")

    def publish(self, topic_name: str, msg_type: Type, data: Any, qos_profile: Optional[int] = None) -> None:
        self._init_node()
        qos = qos_profile or self._qos
        if topic_name not in self._publishers:
            self._publishers[topic_name] = rospy.Publisher(topic_name, msg_type, queue_size=qos)
        msg = msg_type()
        if isinstance(data, dict):
            for key, value in data.items():
                if hasattr(msg, key):
                    setattr(msg, key, value)
        else:
            msg.data = data
        self._publishers[topic_name].publish(msg)

    def get_param(self, param_name: str, default: Any = None) -> Any:
        self._init_node()
        return rospy.get_param(param_name, default)

    def set_param(self, param_name: str, value: Any) -> bool:
        self._init_node()
        try:
            rospy.set_param(param_name, value)
            return True
        except:
            return False

    def spin(self, spin_once: bool = False) -> None:
        if not self.is_initialized:
            return
        if spin_once:
            rospy.sleep(0.1)
        else:
            rospy.spin()

    def destroy_node(self) -> None:
        if self.is_initialized:
            rospy.core.shutdown()
            self._node_initialized = False

ros1_comm = ROS1Communicator()