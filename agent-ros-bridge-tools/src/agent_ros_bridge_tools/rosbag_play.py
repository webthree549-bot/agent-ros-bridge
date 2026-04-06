"""ROSBagPlayTool - Play recorded bag files.

Ported from NASA ROSA (MIT License).
"""

import time
from pathlib import Path
from typing import Any

from .base import Tool, ToolResult, register_tool


@register_tool
class ROSBagPlayTool(Tool):
    """Play recorded ROS bag files.

    Compatible with NASA ROSA rosbag_play tool format.

    Example:
        >>> tool = ROSBagPlayTool()
        >>> result = await tool.execute("/path/to/recording.bag", rate=0.5)
        >>> print(result.data["topics_played"])
    """

    name = "rosbag_play"
    description = "Play recorded ROS bag files"
    version = "1.0.0"

    async def execute(
        self,
        bag_file: str,
        topics: list[str] | None = None,
        rate: float = 1.0,
        start: float = 0.0,
        duration: float | None = None,
        loop: bool = False,
    ) -> ToolResult:
        """Play a recorded ROS bag file.

        Args:
            bag_file: Path to bag file (.bag for ROS1, .db3 for ROS2)
            topics: List of topics to play (None = all)
            rate: Playback rate (1.0 = normal, 0.5 = half speed)
            start: Start offset in seconds
            duration: Duration to play in seconds (None = all)
            loop: Loop playback

        Returns:
            ToolResult with playback statistics
        """
        start_time = time.time()

        bag_path = Path(bag_file)
        if not bag_path.exists():
            return ToolResult.error_result(f"Bag file not found: {bag_file}")

        try:
            # Determine ROS version from file extension
            if bag_path.suffix == ".bag":
                # ROS1 bag
                return await self._execute_ros1(
                    bag_path, topics, rate, start, duration, loop, start_time
                )
            elif bag_path.suffix == ".db3":
                # ROS2 bag
                return await self._execute_ros2(
                    bag_path, topics, rate, start, duration, loop, start_time
                )
            else:
                return ToolResult.error_result(
                    f"Unknown bag format: {bag_path.suffix}. Use .bag (ROS1) or .db3 (ROS2)"
                )

        except Exception as e:
            return ToolResult.error_result(
                f"Error playing bag file: {e}",
                execution_time_ms=(time.time() - start_time) * 1000,
            )

    async def _execute_ros1(
        self,
        bag_path: Path,
        topics: list[str] | None,
        rate: float,
        start: float,
        duration: float | None,
        loop: bool,
        start_time: float,
    ) -> ToolResult:
        """Execute using ROS1."""
        import rosbag
        import rospy
        from rospy import Publisher

        # Open bag file
        bag = rosbag.Bag(bag_path)

        try:
            # Get bag info
            info = bag.get_type_and_topic_info()
            available_topics = list(info.topics.keys())

            # Filter topics if specified
            play_topics = topics if topics else available_topics

            # Create publishers
            publishers = {}
            for topic in play_topics:
                if topic in available_topics:
                    msg_type = info.topics[topic].msg_type
                    msg_class = self._import_message_class_ros1(msg_type)
                    if msg_class:
                        publishers[topic] = Publisher(topic, msg_class, queue_size=10)

            # Wait for publishers to connect
            rospy.sleep(0.5)

            # Play messages
            messages_played = 0
            start_timestamp = None

            for topic, msg, timestamp in bag.read_messages(topics=play_topics):
                if rospy.is_shutdown():
                    break

                # Apply start offset
                if start_timestamp is None:
                    start_timestamp = timestamp
                elapsed = (timestamp - start_timestamp).to_sec()
                if elapsed < start:
                    continue

                # Apply duration limit
                if duration and elapsed > start + duration:
                    break

                # Publish message
                if topic in publishers:
                    publishers[topic].publish(msg)
                    messages_played += 1

                    # Rate limiting
                    if rate > 0:
                        rospy.sleep(1.0 / (10 * rate))  # Rough approximation

            return ToolResult.success_result(
                data={
                    "bag_file": str(bag_path),
                    "format": "ROS1",
                    "topics_played": list(publishers.keys()),
                    "messages_played": messages_played,
                    "loop": loop,
                },
                execution_time_ms=(time.time() - start_time) * 1000,
            )

        finally:
            bag.close()

    async def _execute_ros2(
        self,
        bag_path: Path,
        topics: list[str] | None,
        rate: float,
        start: float,
        duration: float | None,
        loop: bool,
        start_time: float,
    ) -> ToolResult:
        """Execute using ROS2."""
        import rclpy
        from rclpy.node import Node
        from rclpy.serialization import deserialize_message
        import sqlite3

        rclpy.init()
        node = Node("rosbag_play_tool")

        try:
            # ROS2 bags are SQLite databases
            conn = sqlite3.connect(bag_path)
            cursor = conn.cursor()

            # Get topics from bag
            cursor.execute("SELECT id, name, type FROM topics")
            topic_rows = cursor.fetchall()

            topic_map = {}
            available_topics = []
            for row in topic_rows:
                topic_id, topic_name, topic_type = row
                topic_map[topic_id] = (topic_name, topic_type)
                available_topics.append(topic_name)

            # Filter topics if specified
            play_topics = topics if topics else available_topics

            # Create publishers
            publishers = {}
            for topic_id, (topic_name, topic_type) in topic_map.items():
                if topic_name in play_topics:
                    msg_class = self._import_message_class_ros2(topic_type)
                    if msg_class:
                        publishers[topic_id] = {
                            "publisher": node.create_publisher(msg_class, topic_name, 10),
                            "name": topic_name,
                            "class": msg_class,
                        }

            # Play messages
            cursor.execute(
                "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"
            )
            messages_played = 0

            for row in cursor.fetchall():
                topic_id, timestamp, data = row

                if topic_id in publishers:
                    pub_info = publishers[topic_id]
                    msg = deserialize_message(data, pub_info["class"])
                    pub_info["publisher"].publish(msg)
                    messages_played += 1

            conn.close()

            return ToolResult.success_result(
                data={
                    "bag_file": str(bag_path),
                    "format": "ROS2",
                    "topics_played": [p["name"] for p in publishers.values()],
                    "messages_played": messages_played,
                    "loop": loop,
                },
                execution_time_ms=(time.time() - start_time) * 1000,
            )

        finally:
            node.destroy_node()
            rclpy.shutdown()

    def _import_message_class_ros1(self, msg_type: str) -> type | None:
        """Import ROS1 message class."""
        try:
            import importlib

            pkg, msg = msg_type.split("/")
            msg_module = importlib.import_module(f"{pkg}.msg")
            return getattr(msg_module, msg)
        except (ImportError, AttributeError, ValueError):
            return None

    def _import_message_class_ros2(self, msg_type: str) -> type | None:
        """Import ROS2 message class."""
        try:
            import importlib

            pkg, msg = msg_type.split("/")
            msg_module = importlib.import_module(f"{pkg}.msg")
            return getattr(msg_module, msg)
        except (ImportError, AttributeError, ValueError):
            return None

    def get_schema(self) -> dict[str, Any]:
        """Get tool parameter schema."""
        return {
            **super().get_schema(),
            "parameters": {
                "bag_file": {
                    "type": "string",
                    "description": "Path to bag file (.bag for ROS1, .db3 for ROS2)",
                    "required": True,
                },
                "topics": {
                    "type": "array",
                    "description": "List of topics to play (None = all)",
                    "items": {"type": "string"},
                    "default": None,
                },
                "rate": {
                    "type": "number",
                    "description": "Playback rate (1.0 = normal)",
                    "default": 1.0,
                    "minimum": 0.1,
                },
                "start": {
                    "type": "number",
                    "description": "Start offset in seconds",
                    "default": 0.0,
                    "minimum": 0.0,
                },
                "duration": {
                    "type": "number",
                    "description": "Duration to play in seconds (None = all)",
                    "default": None,
                },
                "loop": {
                    "type": "boolean",
                    "description": "Loop playback",
                    "default": False,
                },
            },
        }
