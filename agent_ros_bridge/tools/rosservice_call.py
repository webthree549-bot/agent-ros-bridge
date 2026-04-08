"""rosservice_call tool for Agent ROS Bridge.

Call a ROS service.
Compatible with NASA ROSA rosservice_call tool.
"""

import time
from typing import Any

from agent_ros_bridge.tools.base import ROSTool, ToolResult


class ROSServiceCallTool(ROSTool):
    """Call a ROS service."""

    name = "rosservice_call"
    description = "Call a ROS service"
    version = "1.0.0"

    def execute(
        self,
        service: str,
        request: dict[str, Any] = None,
        timeout_sec: float = 5.0,
        **kwargs,
    ) -> ToolResult:
        """Execute rosservice_call.

        Args:
            service: ROS service name (e.g., '/get_plan', '/clear_costmap')
            request: Service request parameters (default: {})
            timeout_sec: Timeout in seconds (default: 5.0)

        Returns:
            ToolResult with service response
        """
        start_time = time.time()
        request = request or {}

        try:
            return self._execute_ros2(service, request, timeout_sec)
        except ImportError:
            return ToolResult(
                success=False,
                output="",
                error="ROS2 not available. Install ros-humble-desktop.",
                data={"service": service, "request": request},
                execution_time_ms=(time.time() - start_time) * 1000,
            )
        except Exception as e:
            return ToolResult(
                success=False,
                output="",
                error=str(e),
                data={"service": service},
                execution_time_ms=(time.time() - start_time) * 1000,
            )

    def _execute_ros2(
        self, service: str, request: dict[str, Any], timeout_sec: float
    ) -> ToolResult:
        """Execute with ROS2."""
        import rclpy
        from rclpy.node import Node

        # Initialize ROS2 if needed
        if not rclpy.ok():
            rclpy.init()

        # Create temporary node
        node = Node("rosservice_call_tool")

        # Get service type
        service_names_and_types = node.get_service_names_and_types()
        service_type = None
        for name, types in service_names_and_types:
            if name == service:
                service_type = types[0] if types else None
                break

        if not service_type:
            node.destroy_node()
            return ToolResult(
                success=False,
                output="",
                error=f"Service '{service}' not found",
            )

        # Import service type dynamically
        try:
            srv_module, srv_class = service_type.split("/")
            exec(f"from {srv_module}.srv import {srv_class} as SrvType")
            SrvType = locals()["SrvType"]
        except Exception as e:
            node.destroy_node()
            return ToolResult(
                success=False,
                output="",
                error=f"Failed to import service type: {e}",
            )

        # Create client
        client = node.create_client(SrvType, service)

        # Wait for service
        if not client.wait_for_service(timeout_sec=timeout_sec):
            node.destroy_node()
            return ToolResult(
                success=False,
                output="",
                error=f"Service '{service}' not available within {timeout_sec}s",
            )

        # Create request
        req = SrvType.Request()
        for key, value in request.items():
            if hasattr(req, key):
                setattr(req, key, value)

        # Call service
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

        # Cleanup
        node.destroy_node()

        execution_time = (time.time() - start_time) * 1000

        if future.result() is None:
            return ToolResult(
                success=False,
                output="",
                error="Service call failed or timed out",
                execution_time_ms=execution_time,
            )

        # Format response
        response = future.result()
        output_lines = [f"Service: {service}", f"Type: {service_type}", "Response:"]

        for attr in dir(response):
            if not attr.startswith("_") and hasattr(response, attr):
                try:
                    val = getattr(response, attr)
                    if not callable(val):
                        output_lines.append(f"  {attr}: {val}")
                except Exception:
                    pass

        return ToolResult(
            success=True,
            output="\n".join(output_lines),
            data={
                "service": service,
                "service_type": service_type,
                "response": str(response),
            },
            execution_time_ms=execution_time,
        )
