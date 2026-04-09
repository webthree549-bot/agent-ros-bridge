"""ROS Publisher tool."""

from typing import Any

from .base import ROSTool


class ROSPublisherTool(ROSTool):
    """Tool for publishing ROS messages."""

    def __init__(
        self,
        topic_name: str,
        message_type: str,
        node=None,
        qos_profile: str = "default",
    ):
        self.topic_name = topic_name
        self.message_type = message_type
        self._node = node
        self.qos_profile = qos_profile
        self._publisher = None

    @property
    def name(self) -> str:
        """Generate tool name from topic name."""
        clean_name = self.topic_name.replace("/", "_").strip("_")
        return f"ros_publish_{clean_name}"

    @property
    def description(self) -> str:
        return f"ROS publisher tool for {self.topic_name} ({self.message_type})"

    def execute(self, **kwargs) -> dict[str, Any]:
        """Publish a message."""
        data = kwargs.get("data", {})

        if self._node is None:
            return {
                "success": False,
                "error": "ROS node not available",
            }

        try:
            # Create publisher if not exists
            if self._publisher is None:
                self._publisher = self._node.create_publisher(
                    self.message_type,
                    self.topic_name,
                    10,  # QoS depth
                )

            # Publish message
            msg = self._create_message(data)
            self._publisher.publish(msg)

            return {
                "success": True,
                "message": f"Published to {self.topic_name}",
            }

        except Exception as e:
            return {
                "success": False,
                "error": str(e),
            }

    def _create_message(self, data: dict) -> Any:
        """Create ROS message from data."""
        # Simplified - real implementation would use ROS message types
        return data
