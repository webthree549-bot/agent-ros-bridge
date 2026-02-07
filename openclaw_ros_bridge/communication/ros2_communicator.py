#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 Communicator - Unified API wrapper for rclpy (Humble/Jazzy)"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from typing import Callable, Type, Dict, Any, Optional
from openclaw_ros_bridge.base.logger import get_logger
from openclaw_ros_bridge.version.version_manager import version_manager

logger = get_logger(__name__)

class ROS2Communicator:
    """ROS2 Communicator"""
    def __init__(self, node_name: Optional[str] = None):
        self.ros_distro = version_manager.ROS_DISTRO
        self.node_name = node_name or f"ros2_openclaw_bridge_{self.ros_distro}"
        self._node: Optional[Node] = None
        self._publishers: Dict[str, rclpy.publisher.Publisher] = {}
        self._subscribers: Dict[str, rclpy.subscription.Subscription] = {}
        self._qos_default = self._create_qos(version_manager.get_ros_param("default_qos", 10))
        self._node_initialized = False

    @property
    def is_initialized(self) -> bool:
        return self._node_initialized and self._node is not None

    def _create_qos(self, depth: int = 10) -> QoSProfile:
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=depth
        )

    def _init_node(self) -> None:
        if not self.is_initialized:
            rclpy.init(args=None)
            self._node = Node(self.node_name)
            self._node_initialized = True
            logger.info(f"ROS2 node initialized: {self.node_name}")

    def subscribe(self, topic_name: str, msg_type: Type, callback: Callable, qos_profile: Optional[int] = None) -> None:
        self._init_node()
        qos = self._create_qos(qos_profile) if qos_profile else self._qos_default
        if topic_name not in self._subscribers:
            self._subscribers[topic_name] = self._node.create_subscription(msg_type, topic_name, callback, qos)
            logger.info(f"ROS2 subscribed: {topic_name}")

    def publish(self, topic_name: str, msg_type: Type, data: Any, qos_profile: Optional[int] = None) -> None:
        self._init_node()
        qos = self._create_qos(qos_profile) if qos_profile else self._qos_default
        if topic_name not in self._publishers:
            self._publishers[topic_name] = self._node.create_publisher(msg_type, topic_name, qos)
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
        try:
            return self._node.get_parameter(param_name).value
        except:
            self._node.declare_parameter(param_name, default)
            return default

    def set_param(self, param_name: str, value: Any) -> bool:
        self._init_node()
        try:
            from rclpy.parameter import Parameter
            self._node.set_parameter(Parameter(param_name, Parameter.Type.STRING, str(value)))
            return True
        except:
            return False

    def spin(self, spin_once: bool = False) -> None:
        if not self.is_initialized:
            return
        if spin_once:
            rclpy.spin_once(self._node)
        else:
            rclpy.spin(self._node)

    def destroy_node(self) -> None:
        if self.is_initialized:
            self._node.destroy_node()
            rclpy.shutdown()
            self._node = None
            self._node_initialized = False

ros2_comm = ROS2Communicator()