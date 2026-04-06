"""ROSServiceCallTool - Call ROS services.

Ported from NASA ROSA (MIT License).
"""

import time
from typing import Any

from .base import Tool, ToolResult, register_tool


@register_tool
class ROSServiceCallTool(Tool):
    """Call a ROS service.

    Compatible with NASA ROSA rosservice_call tool format.

    Example:
        >>> tool = ROSServiceCallTool()
        >>> result = await tool.execute("/spawn", x=1.0, y=2.0, name="robot1")
        >>> print(result.data)
    """

    name = "rosservice_call"
    description = "Call a ROS service"
    version = "1.0.0"

    async def execute(
        self,
        service_name: str,
        **kwargs
    ) -> ToolResult:
        """Call a ROS service.

        Args:
            service_name: Full service name (e.g., /spawn)
            **kwargs: Service request parameters

        Returns:
            ToolResult with service response
        """
        start_time = time.time()

        try:
            # Try ROS2 first
            try:
                import rclpy
                return await self._execute_ros2(service_name, kwargs, start_time)
            except ImportError:
                # Fallback to ROS1
                try:
                    import rospy
                    return await self._execute_ros1(service_name, kwargs, start_time)
                except ImportError:
                    return ToolResult.error_result("Neither ROS1 nor ROS2 available")

        except Exception as e:
            return ToolResult.error_result(
                f"Error calling service: {e}",
                execution_time_ms=(time.time() - start_time) * 1000,
            )

    async def _execute_ros2(
        self, service_name: str, params: dict, start_time: float
    ) -> ToolResult:
        """Execute using ROS2."""
        import rclpy
        from rclpy.node import Node

        rclpy.init()
        node = Node("rosservice_call_tool")

        try:
            # Get service type
            service_type = self._get_service_type_ros2(node, service_name)
            if not service_type:
                return ToolResult.error_result(f"Service {service_name} not found")

            # Import service classes
            srv_module, srv_type = service_type.split("/")
            srv_class, req_class, resp_class = self._import_service_classes_ros2(
                srv_module, srv_type
            )

            if not all([srv_class, req_class, resp_class]):
                return ToolResult.error_result(f"Cannot import service type: {service_type}")

            # Create client and call service
            client = node.create_client(srv_class, service_name)

            if not client.wait_for_service(timeout_sec=5.0):
                return ToolResult.error_result(f"Service {service_name} not available")

            # Build request
            request = req_class()
            for key, value in params.items():
                if hasattr(request, key):
                    setattr(request, key, value)

            # Call service
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

            if future.result() is None:
                return ToolResult.error_result("Service call timed out")

            response = future.result()

            # Convert response to dict
            response_dict = {}
            for field_name in dir(response):
                if not field_name.startswith("_"):
                    try:
                        response_dict[field_name] = getattr(response, field_name)
                    except AttributeError:
                        pass

            return ToolResult.success_result(
                data={
                    "service": service_name,
                    "type": service_type,
                    "response": response_dict,
                },
                execution_time_ms=(time.time() - start_time) * 1000,
            )

        finally:
            node.destroy_node()
            rclpy.shutdown()

    async def _execute_ros1(
        self, service_name: str, params: dict, start_time: float
    ) -> ToolResult:
        """Execute using ROS1."""
        import rospy

        # Get service type
        import rosservice

        service_type = rosservice.get_service_type(service_name)
        if not service_type:
            return ToolResult.error_result(f"Service {service_name} not found")

        # Import service class
        srv_class, req_class, resp_class = self._import_service_classes_ros1(service_type)
        if not all([srv_class, req_class, resp_class]):
            return ToolResult.error_result(f"Cannot import service type: {service_type}")

        # Create proxy and call service
        rospy.wait_for_service(service_name, timeout=5.0)
        service_proxy = rospy.ServiceProxy(service_name, srv_class)

        # Build request
        request = req_class()
        for key, value in params.items():
            if hasattr(request, key):
                setattr(request, key, value)

        # Call service
        response = service_proxy(request)

        # Convert response to dict
        response_dict = {}
        for slot in response.__slots__:
            response_dict[slot] = getattr(response, slot)

        return ToolResult.success_result(
            data={
                "service": service_name,
                "type": service_type,
                "response": response_dict,
            },
            execution_time_ms=(time.time() - start_time) * 1000,
        )

    def _get_service_type_ros2(self, node: "Node", service_name: str) -> str | None:
        """Get service type using ROS2."""
        services = node.get_service_names_and_types()
        for name, types in services:
            if name == service_name and types:
                return types[0]
        return None

    def _import_service_classes_ros2(self, module: str, srv_type: str):
        """Import ROS2 service classes dynamically."""
        try:
            import importlib

            srv_module = importlib.import_module(f"{module}.srv")
            srv_class = getattr(srv_module, srv_type)
            req_class = getattr(srv_module, f"{srv_type}_Request")
            resp_class = getattr(srv_module, f"{srv_type}_Response")
            return srv_class, req_class, resp_class
        except (ImportError, AttributeError):
            return None, None, None

    def _import_service_classes_ros1(self, service_type: str):
        """Import ROS1 service classes dynamically."""
        try:
            import importlib

            pkg, srv = service_type.split("/")
            srv_module = importlib.import_module(f"{pkg}.srv")
            srv_class = getattr(srv_module, srv)
            req_class = getattr(srv_module, f"{srv}Request")
            resp_class = getattr(srv_module, f"{srv}Response")
            return srv_class, req_class, resp_class
        except (ImportError, AttributeError, ValueError):
            return None, None, None

    def get_schema(self) -> dict[str, Any]:
        """Get tool parameter schema."""
        return {
            **super().get_schema(),
            "parameters": {
                "service_name": {
                    "type": "string",
                    "description": "Full service name (e.g., /spawn)",
                    "required": True,
                },
            },
            "additional_parameters": True,
        }
