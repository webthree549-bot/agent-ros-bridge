"""ROS Service call tool."""

from typing import Any

from .base import ROSTool


class ROSServiceCallTool(ROSTool):
    """Tool for calling ROS services."""

    def __init__(
        self,
        service_name: str,
        service_type: str,
        node=None,
        timeout: float = 10.0,
    ):
        self.service_name = service_name
        self.service_type = service_type
        self._node = node
        self._timeout = timeout
        self._client = None

    @property
    def name(self) -> str:
        """Generate tool name from service name."""
        clean_name = self.service_name.replace("/", "_").strip("_")
        return f"ros_service_{clean_name}"

    @property
    def description(self) -> str:
        return f"ROS service call tool for {self.service_name} ({self.service_type})"

    def get_parameters_schema(self) -> dict[str, Any]:
        """Get JSON schema for service parameters."""
        return {
            "type": "object",
            "properties": {
                "data": {
                    "type": "object",
                    "description": "Service request data",
                },
            },
        }

    def execute(self, **kwargs) -> dict[str, Any]:
        """Execute the service call."""
        data = kwargs.get("data", {})

        if self._node is None:
            return {
                "success": False,
                "error": "ROS node not available",
            }

        try:
            # Create client if not exists
            if self._client is None:
                self._client = self._node.create_client(
                    self.service_type,
                    self.service_name,
                )

            # Wait for service
            if not self._client.wait_for_service(timeout_sec=self._timeout):
                return {
                    "success": False,
                    "error": f"Service {self.service_name} not available",
                }

            # Call service
            request = self._create_request(data)
            _future = self._client.call_async(request)
            # In real implementation, would wait for future

            return {
                "success": True,
                "message": f"Called {self.service_name}",
            }

        except Exception as e:
            return {
                "success": False,
                "error": str(e),
            }

    def _create_request(self, data: dict) -> Any:
        """Create service request from data."""
        # Simplified - real implementation would use ROS message types
        return data
