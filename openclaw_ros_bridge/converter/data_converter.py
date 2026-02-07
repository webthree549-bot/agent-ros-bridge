#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Data Converter - Standardized ROS msg ↔ OpenClaw JSON conversion"""
import json
import time
from typing import Dict, Any, Optional
from openclaw_ros_bridge.version.version_manager import version_manager

class DataConverter:
    """ROS ↔ OpenClaw Data Converter - version-agnostic protocol translation"""
    def __init__(self):
        self.oc_version = version_manager.OC_VER
        self.ros_type = version_manager.ROS_TYPE
        self.ros_distro = version_manager.ROS_DISTRO
        self.business_id = version_manager.get_oc_param("business_id", "default_001")
        self.oc_data_prefix = version_manager.get_oc_param("data_prefix", "")
        self.mock_mode = version_manager.MOCK_MODE

    def _add_oc_metadata(self, data: Dict[str, Any], data_type: str) -> Dict[str, Any]:
        """Add OpenClaw v1.x/v2.x metadata to data (prefix, business ID, type)"""
        metadata = {
            "business_id": self.business_id,
            "data_type": data_type,
            "timestamp": data.get("timestamp", time.time()),
            "openclaw_version": self.oc_version,
            "ros_type": self.ros_type
        }
        # Add v1.x data prefix (v2.x uses raw keys)
        if self.oc_version == "v1" and self.oc_data_prefix:
            prefixed_data = {f"{self.oc_data_prefix}{k}": v for k, v in data.items() if k != "timestamp"}
            return {**metadata, **prefixed_data}
        return {**metadata, **data}

    def _remove_oc_metadata(self, oc_data: Dict[str, Any]) -> Dict[str, Any]:
        """Strip OpenClaw metadata and reverse v1.x prefix for ROS conversion"""
        core_keys = ["business_id", "data_type", "timestamp", "openclaw_version", "ros_type"]
        core_data = {k: v for k, v in oc_data.items() if k not in core_keys}
        # Remove v1.x prefix
        if self.oc_version == "v1" and self.oc_data_prefix:
            core_data = {k.replace(self.oc_data_prefix, ""): v for k, v in core_data.items()}
        # Add timestamp if missing
        if "timestamp" not in core_data and "timestamp" in oc_data:
            core_data["timestamp"] = oc_data["timestamp"]
        return core_data

    def ros2oc(self, ros_msg: Any, data_type: str, ros_msg_type: Optional[str] = None) -> str:
        """Convert ROS1/ROS2 message to OpenClaw v1.x/v2.x compliant JSON string"""
        if self.mock_mode:
            mock_data = self._add_oc_metadata({"data": "mock_ros_data", "value": 0.0}, data_type)
            return json.dumps(mock_data)
        # Convert ROS msg to dict
        from openclaw_ros_bridge.communication.msg_converter import msg_converter
        ros_dict = msg_converter.ros2json(ros_msg) if ros_msg else {"data": []}
        # Add OpenClaw metadata and format
        oc_dict = self._add_oc_metadata(ros_dict, data_type)
        return json.dumps(oc_dict, ensure_ascii=False)

    def oc2ros(self, oc_json: str, ros_msg_type: str, target_ros_type: Optional[str] = None) -> Any:
        """Convert OpenClaw v1.x/v2.x JSON string to ROS1/ROS2 message object"""
        target_ros = target_ros_type or self.ros_type
        if self.mock_mode:
            from openclaw_ros_bridge.communication.msg_converter import msg_converter
            return msg_converter.json2ros({"data": "mock_oc_data"}, ros_msg_type, target_ros)
        # Parse JSON and strip metadata
        try:
            oc_dict = json.loads(oc_json)
            core_data = self._remove_oc_metadata(oc_dict)
            # Convert core data to ROS msg
            from openclaw_ros_bridge.communication.msg_converter import msg_converter
            return msg_converter.json2ros(core_data, ros_msg_type, target_ros)
        except json.JSONDecodeError:
            from openclaw_ros_bridge.communication.msg_converter import msg_converter
            return msg_converter.json2ros({}, ros_msg_type, target_ros)

    def batch_ros2oc(self, ros_msg_list: list, data_type: str) -> str:
        """Batch convert ROS messages to OpenClaw JSON (for high-frequency sensors)"""
        from openclaw_ros_bridge.communication.msg_converter import msg_converter
        batch_data = [msg_converter.ros2json(msg) for msg in ros_msg_list]
        oc_batch = self._add_oc_metadata({"batch": batch_data, "count": len(batch_data)}, data_type)
        return json.dumps(oc_batch, ensure_ascii=False)

# Global Data Converter Instance
data_converter = DataConverter()
