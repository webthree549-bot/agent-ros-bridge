"""ROSNodeListTool - List running ROS nodes.

Ported from NASA ROSA (MIT License).
"""

import time
from typing import Any

from .base import Tool, ToolResult, register_tool


@register_tool
class ROSNodeListTool(Tool):
    """List running ROS nodes.

    Compatible with NASA ROSA rosnode_list tool format.

    Example:
        >>> tool = ROSNodeListTool()
        >>> result = await tool.execute()
        >>> print(result.data["nodes"])
    """

    name = "rosnode_list"
    description = "List running ROS nodes"
    version = "1.0.0"

    async def execute(
        self,
        include_services: bool = True,
        include_topics: bool = True,
    ) -> ToolResult:
        """List running ROS nodes.

        Args:
            include_services: Include services for each node
            include_topics: Include topics for each node

        Returns:
            ToolResult with list of nodes and details
        """
        start_time = time.time()

        try:
            # Try ROS2 first
            try:
                import rclpy
                return await self._execute_ros2(
                    include_services, include_topics, start_time
                )
            except ImportError:
                # Fallback to ROS1
                try:
                    import rospy
                    import rosnode
                    return await self._execute_ros1(
                        include_services, include_topics, start_time
                    )
                except ImportError:
                    return ToolResult.error_result("Neither ROS1 nor ROS2 available")

        except Exception as e:
            return ToolResult.error_result(
                f"Error listing nodes: {e}",
                execution_time_ms=(time.time() - start_time) * 1000,
            )

    async def _execute_ros2(
        self, include_services: bool, include_topics: bool, start_time: float
    ) -> ToolResult:
        """Execute using ROS2."""
        import rclpy
        from rclpy.node import Node

        rclpy.init()
        node = Node("rosnode_list_tool")

        try:
            # Get all node names
            node_names = node.get_node_names()

            nodes_info = []
            for node_name in node_names:
                info = {"name": node_name}

                if include_topics:
                    # Get publisher topics
                    pubs = node.get_publisher_names_and_types_by_node(node_name, "")
                    info["publishers"] = [{"topic": t, "types": ts} for t, ts in pubs]

                    # Get subscriber topics
                    subs = node.get_subscription_names_and_types_by_node(node_name, "")
                    info["subscribers"] = [{"topic": t, "types": ts} for t, ts in subs]

                if include_services:
                    # Get services
                    srvs = node.get_service_names_and_types_by_node(node_name, "")
                    info["services"] = [{"service": s, "types": ts} for s, ts in srvs]

                nodes_info.append(info)

            return ToolResult.success_result(
                data={
                    "nodes": nodes_info,
                    "count": len(nodes_info),
                },
                execution_time_ms=(time.time() - start_time) * 1000,
            )

        finally:
            node.destroy_node()
            rclpy.shutdown()

    async def _execute_ros1(
        self, include_services: bool, include_topics: bool, start_time: float
    ) -> ToolResult:
        """Execute using ROS1."""
        import rosnode

        node_names = rosnode.get_node_names()

        nodes_info = []
        for node_name in node_names:
            info = {"name": node_name}

            if include_topics:
                # Get topics for this node
                pubs, subs = rosnode.get_node_topic_names(node_name)
                info["published_topics"] = pubs
                info["subscribed_topics"] = subs

            if include_services:
                # Get services for this node
                services = rosnode.get_node_service_names(node_name)
                info["services"] = services

            # Get node URI
            try:
                uri = rosnode.get_node_uri(node_name)
                info["uri"] = uri
            except:
                pass

            nodes_info.append(info)

        return ToolResult.success_result(
            data={
                "nodes": nodes_info,
                "count": len(nodes_info),
            },
            execution_time_ms=(time.time() - start_time) * 1000,
        )

    def get_schema(self) -> dict[str, Any]:
        """Get tool parameter schema."""
        return {
            **super().get_schema(),
            "parameters": {
                "include_services": {
                    "type": "boolean",
                    "description": "Include services for each node",
                    "default": True,
                },
                "include_topics": {
                    "type": "boolean",
                    "description": "Include topics for each node",
                    "default": True,
                },
            },
        }
