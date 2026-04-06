"""ROSTopicEchoTool - Echo messages from ROS topics.

Ported from NASA ROSA (MIT License).
"""

import time
from typing import Any

from .base import Tool, ToolResult, register_tool


@register_tool
class ROSTopicEchoTool(Tool):
    """Echo messages from a ROS topic.

    Compatible with NASA ROSA rostopic_echo tool format.

    Example:
        >>> tool = ROSTopicEchoTool()
        >>> result = await tool.execute("/cmd_vel", count=5)
        >>> print(result.data)
    """

    name = "rostopic_echo"
    description = "Echo messages from a ROS topic"
    version = "1.0.0"

    async def execute(
        self,
        topic_name: str,
        count: int = 1,
        rate: float | None = None,
        timeout: float = 5.0,
    ) -> ToolResult:
        """Echo messages from a ROS topic.

        Args:
            topic_name: Full topic name (e.g., /cmd_vel)
            count: Number of messages to echo (0 = infinite)
            rate: Rate limit in Hz (None = no limit)
            timeout: Maximum time to wait for messages

        Returns:
            ToolResult with echoed messages
        """
        start_time = time.time()

        try:
            # Try to import ROS2 libraries
            try:
                import rclpy
                from rclpy.node import Node
                from rosidl_runtime_py import message_to_yaml

                return await self._execute_ros2(
                    topic_name, count, rate, timeout, start_time
                )
            except ImportError:
                # Fallback to ROS1
                try:
                    import rospy
                    from rospy import msg_to_yaml

                    return await self._execute_ros1(
                        topic_name, count, rate, timeout, start_time
                    )
                except ImportError:
                    return ToolResult.error_result(
                        "Neither ROS1 nor ROS2 available"
                    )

        except Exception as e:
            return ToolResult.error_result(
                f"Error echoing topic: {e}",
                execution_time_ms=(time.time() - start_time) * 1000,
            )

    async def _execute_ros2(
        self, topic_name: str, count: int, rate: float | None, timeout: float, start_time: float
    ) -> ToolResult:
        """Execute using ROS2."""
        import rclpy
        from rclpy.node import Node
        from rosidl_runtime_py import message_to_yaml
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        messages = []
        received_count = 0

        # Create temporary node
        rclpy.init()
        node = Node("rostopic_echo_tool")

        try:
            # Get topic type
            topic_type = self._get_topic_type_ros2(node, topic_name)
            if not topic_type:
                return ToolResult.error_result(f"Topic {topic_name} not found")

            # Import message type dynamically
            msg_module, msg_type = topic_type.split("/")
            msg_class = self._import_message_class(msg_module, msg_type)

            if not msg_class:
                return ToolResult.error_result(f"Cannot import message type: {topic_type}")

            # Subscribe and collect messages
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

            def callback(msg):
                nonlocal received_count
                if count > 0 and received_count >= count:
                    return
                messages.append(message_to_yaml(msg))
                received_count += 1

            subscription = node.create_subscription(msg_class, topic_name, callback, qos)

            # Spin until we have enough messages or timeout
            while (count == 0 or received_count < count) and (time.time() - start_time) < timeout:
                rclpy.spin_once(node, timeout_sec=0.1)

            return ToolResult.success_result(
                data={
                    "topic": topic_name,
                    "type": topic_type,
                    "messages": messages,
                    "count": len(messages),
                },
                execution_time_ms=(time.time() - start_time) * 1000,
            )

        finally:
            node.destroy_node()
            rclpy.shutdown()

    async def _execute_ros1(
        self, topic_name: str, count: int, rate: float | None, timeout: float, start_time: float
    ) -> ToolResult:
        """Execute using ROS1."""
        import rospy
        from rospy import msg_to_yaml

        messages = []
        received_count = 0

        # Get topic type
        topic_type = self._get_topic_type_ros1(topic_name)
        if not topic_type:
            return ToolResult.error_result(f"Topic {topic_name} not found")

        # Import message type
        msg_class = self._import_message_class_ros1(topic_type)
        if not msg_class:
            return ToolResult.error_result(f"Cannot import message type: {topic_type}")

        def callback(msg):
            nonlocal received_count
            if count > 0 and received_count >= count:
                return
            messages.append(msg_to_yaml(msg))
            received_count += 1

        subscriber = rospy.Subscriber(topic_name, msg_class, callback)

        # Wait for messages
        while (count == 0 or received_count < count) and (time.time() - start_time) < timeout:
            rospy.sleep(0.1)

        subscriber.unregister()

        return ToolResult.success_result(
            data={
                "topic": topic_name,
                "type": topic_type,
                "messages": messages,
                "count": len(messages),
            },
            execution_time_ms=(time.time() - start_time) * 1000,
        )

    def _get_topic_type_ros2(self, node: "Node", topic_name: str) -> str | None:
        """Get topic type using ROS2."""
        topics = node.get_topic_names_and_types()
        for name, types in topics:
            if name == topic_name and types:
                return types[0]
        return None

    def _get_topic_type_ros1(self, topic_name: str) -> str | None:
        """Get topic type using ROS1."""
        import rospy

        topics = rospy.get_published_topics()
        for name, msg_type in topics:
            if name == topic_name:
                return msg_type
        return None

    def _import_message_class(self, module: str, msg_type: str) -> type | None:
        """Import ROS2 message class dynamically."""
        try:
            import importlib

            msg_module = importlib.import_module(f"{module}.msg")
            return getattr(msg_module, msg_type)
        except (ImportError, AttributeError):
            return None

    def _import_message_class_ros1(self, topic_type: str) -> type | None:
        """Import ROS1 message class dynamically."""
        try:
            import importlib

            pkg, msg = topic_type.split("/")
            msg_module = importlib.import_module(f"{pkg}.msg")
            return getattr(msg_module, msg)
        except (ImportError, AttributeError, ValueError):
            return None

    def get_schema(self) -> dict[str, Any]:
        """Get tool parameter schema."""
        return {
            **super().get_schema(),
            "parameters": {
                "topic_name": {
                    "type": "string",
                    "description": "Full topic name (e.g., /cmd_vel)",
                    "required": True,
                },
                "count": {
                    "type": "integer",
                    "description": "Number of messages to echo (0 = infinite)",
                    "default": 1,
                    "minimum": 0,
                },
                "rate": {
                    "type": "number",
                    "description": "Rate limit in Hz",
                    "default": None,
                },
                "timeout": {
                    "type": "number",
                    "description": "Maximum time to wait for messages",
                    "default": 5.0,
                },
            },
        }
