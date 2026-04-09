"""rostopic_echo tool for Agent ROS Bridge.

Echo messages from a ROS topic.
Compatible with NASA ROSA rostopic_echo tool.
"""

import time

from agent_ros_bridge.tools.base import ROSTool, ToolResult


class ROSTopicEchoTool(ROSTool):
    """Echo messages from a ROS topic."""

    name = "rostopic_echo"
    description = "Echo messages from a ROS topic"
    version = "1.0.0"

    def execute(
        self,
        topic: str,
        count: int = 1,
        timeout_sec: float = 5.0,
        **kwargs,
    ) -> ToolResult:
        """Execute rostopic_echo.

        Args:
            topic: ROS topic name (e.g., '/cmd_vel', '/odom')
            count: Number of messages to echo (default: 1)
            timeout_sec: Timeout in seconds (default: 5.0)

        Returns:
            ToolResult with echoed messages
        """
        start_time = time.time()

        try:
            # Try ROS2 first
            return self._execute_ros2(topic, count, timeout_sec)
        except ImportError:
            # ROS2 not available, return mock result
            return ToolResult(
                success=False,
                output="",
                error="ROS2 not available. Install ros-humble-desktop.",
                data={"topic": topic, "count": count},
                execution_time_ms=(time.time() - start_time) * 1000,
            )
        except Exception as e:
            return ToolResult(
                success=False,
                output="",
                error=str(e),
                data={"topic": topic},
                execution_time_ms=(time.time() - start_time) * 1000,
            )

    def _execute_ros2(
        self, topic: str, count: int, timeout_sec: float
    ) -> ToolResult:
        """Execute with ROS2."""
        import time

        import rclpy
        from rclpy.node import Node

        # Initialize ROS2 if needed
        if not rclpy.ok():
            rclpy.init()

        # Create temporary node
        node = Node("rostopic_echo_tool")

        messages = []
        start_time = time.time()

        # Get topic type
        topic_names_and_types = node.get_topic_names_and_types()
        topic_type = None
        for name, types in topic_names_and_types:
            if name == topic:
                topic_type = types[0] if types else None
                break

        if not topic_type:
            node.destroy_node()
            return ToolResult(
                success=False,
                output="",
                error=f"Topic '{topic}' not found or has no type",
            )

        # Import message type dynamically
        try:
            msg_module, msg_class = topic_type.split("/")
            msg_module = msg_module.replace("_", "_")  # Ensure valid module name
            exec(f"from {msg_module}.msg import {msg_class} as MsgType")
            MsgType = locals()["MsgType"]
        except Exception as e:
            node.destroy_node()
            return ToolResult(
                success=False,
                output="",
                error=f"Failed to import message type: {e}",
            )

        # Subscribe to topic
        def callback(msg):
            messages.append(msg)

        subscription = node.create_subscription(MsgType, topic, callback, 10)

        # Spin until we get enough messages or timeout
        while len(messages) < count and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Cleanup
        node.destroy_node()

        execution_time = (time.time() - start_time) * 1000

        if not messages:
            return ToolResult(
                success=False,
                output="",
                error=f"No messages received on '{topic}' within {timeout_sec}s",
                execution_time_ms=execution_time,
            )

        # Convert messages to readable format
        output_lines = []
        for i, msg in enumerate(messages[:count]):
            output_lines.append(f"--- Message {i + 1} ---")
            output_lines.append(str(msg))

        return ToolResult(
            success=True,
            output="\n".join(output_lines),
            data={
                "topic": topic,
                "topic_type": topic_type,
                "message_count": len(messages),
                "requested_count": count,
            },
            execution_time_ms=execution_time,
        )
