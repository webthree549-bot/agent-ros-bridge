"""Custom exceptions for Agent ROS Bridge.

Provides structured error handling with user-friendly messages.
"""


class AgentROSBridgeError(Exception):
    """Base exception for all Agent ROS Bridge errors."""

    def __init__(self, message: str, details: dict | None = None):
        self.details = details or {}
        super().__init__(message)


class RobotConnectionError(AgentROSBridgeError):
    """Failed to connect to robot."""

    def __init__(self, message: str, robot_id: str | None = None):
        self.robot_id = robot_id
        prefix = f"Robot connection failed{' for ' + robot_id if robot_id else ''}"
        super().__init__(f"{prefix}: {message}", {"robot_id": robot_id})


class SafetyValidationError(AgentROSBridgeError):
    """Safety validation failed."""

    def __init__(self, message: str, violation_type: str | None = None):
        self.violation_type = violation_type
        super().__init__(
            f"Safety validation failed: {message}",
            {"violation_type": violation_type},
        )


class TransportError(AgentROSBridgeError):
    """Transport layer error."""

    pass


class ToolExecutionError(AgentROSBridgeError):
    """Tool execution failed."""

    def __init__(self, message: str, tool_name: str | None = None):
        self.tool_name = tool_name
        prefix = f"[{tool_name}] " if tool_name else ""
        super().__init__(f"{prefix}Tool execution failed: {message}", {"tool_name": tool_name})


class ConfigurationError(AgentROSBridgeError):
    """Configuration error."""

    pass


class ValidationError(AgentROSBridgeError):
    """Data validation error."""

    pass
