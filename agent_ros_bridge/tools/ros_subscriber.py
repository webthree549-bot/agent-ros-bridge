"""ROS Subscriber tool."""

from typing import Any
from .base import ROSTool, ToolResult


class ROSSubscriberTool(ROSTool):
    """Tool for subscribing to ROS topics."""

    def __init__(
        self,
        topic_name: str,
        message_type: str,
        node=None,
    ):
        self.topic_name = topic_name
        self.message_type = message_type
        self._node = node
        self._subscriber = None
        self._latest_message = None

    @property
    def name(self) -> str:
        """Generate tool name from topic name."""
        clean_name = self.topic_name.replace("/", "_").strip("_")
        return f"ros_subscribe_{clean_name}"

    @property
    def description(self) -> str:
        return f"ROS subscriber tool for {self.topic_name} ({self.message_type})"

    def execute(self, **kwargs) -> dict[str, Any]:
        """Get latest message from topic."""
        if self._latest_message is None:
            return {
                "success": False,
                "error": "No message received yet",
            }

        return {
            "success": True,
            "data": self._latest_message,
        }

    def _callback(self, msg: Any) -> None:
        """Callback for new messages."""
        self._latest_message = msg
