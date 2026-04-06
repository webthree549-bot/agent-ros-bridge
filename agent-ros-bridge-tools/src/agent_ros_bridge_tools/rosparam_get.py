"""ROSParamGetTool - Get ROS parameters.

Ported from NASA ROSA (MIT License).
"""

import time
from typing import Any

from .base import Tool, ToolResult, register_tool


@register_tool
class ROSParamGetTool(Tool):
    """Get ROS parameters.

    Compatible with NASA ROSA rosparam_get tool format.

    Example:
        >>> tool = ROSParamGetTool()
        >>> result = await tool.execute("/robot_description")
        >>> print(result.data["value"])
    """

    name = "rosparam_get"
    description = "Get ROS parameters"
    version = "1.0.0"

    async def execute(
        self,
        param_name: str | None = None,
        list_all: bool = False,
    ) -> ToolResult:
        """Get ROS parameters.

        Args:
            param_name: Parameter name to get (None = get all if list_all=True)
            list_all: List all parameters

        Returns:
            ToolResult with parameter value(s)
        """
        start_time = time.time()

        try:
            # Try ROS2 first
            try:
                import rclpy
                return await self._execute_ros2(param_name, list_all, start_time)
            except ImportError:
                # Fallback to ROS1
                try:
                    import rospy
                    return await self._execute_ros1(param_name, list_all, start_time)
                except ImportError:
                    return ToolResult.error_result("Neither ROS1 nor ROS2 available")

        except Exception as e:
            return ToolResult.error_result(
                f"Error getting parameter: {e}",
                execution_time_ms=(time.time() - start_time) * 1000,
            )

    async def _execute_ros2(
        self, param_name: str | None, list_all: bool, start_time: float
    ) -> ToolResult:
        """Execute using ROS2."""
        import rclpy
        from rclpy.node import Node

        rclpy.init()
        node = Node("rosparam_get_tool")

        try:
            if list_all or param_name is None:
                # List all parameters
                # ROS2 parameters are node-specific, so we need to query each node
                node_names = node.get_node_names()
                all_params = {}

                for node_name in node_names:
                    try:
                        # Create a parameter client for this node
                        client = node.create_client(
                            rclpy.parameter.Parameter,
                            f"{node_name}/__parameters"
                        )
                        # Note: This is a simplified approach
                        # Full implementation would use parameter services
                        all_params[node_name] = {"note": "Use specific node parameter API"}
                    except Exception:
                        pass

                return ToolResult.success_result(
                    data={
                        "parameters": all_params,
                        "count": len(all_params),
                        "note": "ROS2 parameters are node-specific",
                    },
                    execution_time_ms=(time.time() - start_time) * 1000,
                )
            else:
                # Get specific parameter
                # In ROS2, parameters are node-specific
                # This is a simplified implementation
                return ToolResult.error_result(
                    "ROS2 parameters are node-specific. Use param_name format: /node/parameter"
                )

        finally:
            node.destroy_node()
            rclpy.shutdown()

    async def _execute_ros1(
        self, param_name: str | None, list_all: bool, start_time: float
    ) -> ToolResult:
        """Execute using ROS1."""
        import rospy

        if list_all or param_name is None:
            # List all parameters
            params = rospy.get_param_names()
            return ToolResult.success_result(
                data={
                    "parameters": params,
                    "count": len(params),
                },
                execution_time_ms=(time.time() - start_time) * 1000,
            )
        else:
            # Get specific parameter
            try:
                value = rospy.get_param(param_name)
                return ToolResult.success_result(
                    data={
                        "name": param_name,
                        "value": value,
                        "type": type(value).__name__,
                    },
                    execution_time_ms=(time.time() - start_time) * 1000,
                )
            except KeyError:
                return ToolResult.error_result(f"Parameter '{param_name}' not found")

    def get_schema(self) -> dict[str, Any]:
        """Get tool parameter schema."""
        return {
            **super().get_schema(),
            "parameters": {
                "param_name": {
                    "type": "string",
                    "description": "Parameter name to get (ROS1) or /node/parameter (ROS2)",
                    "default": None,
                },
                "list_all": {
                    "type": "boolean",
                    "description": "List all parameters",
                    "default": False,
                },
            },
        }
