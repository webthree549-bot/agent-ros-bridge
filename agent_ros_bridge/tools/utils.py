"""Utility functions for tool management."""

import re
from typing import Any


def sanitize_tool_name(name: str) -> str:
    """Sanitize a name to be a valid tool identifier.

    Converts to lowercase, replaces special chars with underscores.
    """
    # Replace spaces and special chars with underscores
    sanitized = re.sub(r'[^a-zA-Z0-9_]', '_', name)
    # Remove multiple consecutive underscores
    sanitized = re.sub(r'_+', '_', sanitized)
    # Remove leading/trailing underscores
    sanitized = sanitized.strip('_')
    # Convert to lowercase
    return sanitized.lower()


def parse_ros_type(ros_type: str) -> tuple[str, str]:
    """Parse ROS message type string.

    Args:
        ros_type: Type string like "std_msgs/String" or "geometry_msgs/msg/Twist"

    Returns:
        Tuple of (package_name, message_name)
    """
    parts = ros_type.split('/')
    if len(parts) == 2:
        return parts[0], parts[1]
    elif len(parts) == 3 and parts[1] == 'msg':
        return parts[0], parts[2]
    else:
        raise ValueError(f"Invalid ROS type format: {ros_type}")


def create_message_dict(msg_type: str, data: dict) -> dict[str, Any]:
    """Create a message dictionary with type information."""
    return {
        "type": msg_type,
        **data,
    }
