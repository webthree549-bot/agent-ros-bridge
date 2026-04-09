"""Robot API for external control."""

from dataclasses import dataclass
from typing import Any

# Forward declarations for compatibility
NavigationGoal = None
ManipulationGoal = None
RobotCommandResult = None


@dataclass
class RobotController:
    """Robot controller class (for backwards compatibility)."""

    robot_id: str = "default"
    connected: bool = False

    def connect(self) -> bool:
        """Connect to robot."""
        self.connected = True
        return True

    def disconnect(self) -> None:
        """Disconnect from robot."""
        self.connected = False

    def navigate_to(self, goal) -> Any:
        """Navigate to goal."""
        return {"success": True}


@dataclass
class RobotAPI:
    """High-level API for robot control."""

    robot_id: str
    connected: bool = False
    _last_error: str = ""

    def connect(self) -> bool:
        """Connect to the robot.

        Returns:
            True if connection successful
        """
        try:
            # In real implementation, establish ROS connection
            self._connect_ros()
            self.connected = True
            return True
        except Exception as e:
            self._last_error = str(e)
            self.connected = False
            return False

    def disconnect(self) -> None:
        """Disconnect from the robot."""
        if self.connected:
            self._disconnect_ros()
            self.connected = False

    def _connect_ros(self) -> None:
        """Internal ROS connection (mock)."""
        pass

    def _disconnect_ros(self) -> None:
        """Internal ROS disconnection (mock)."""
        pass

    def send_command(self, command: dict[str, Any]) -> dict[str, Any]:
        """Send command to robot.

        Args:
            command: Command dictionary

        Returns:
            Response from robot
        """
        if not self.connected:
            return {"success": False, "error": "Not connected"}

        # In real implementation, send ROS command
        return {"success": True, "result": "command_executed"}

    def get_status(self) -> dict[str, Any]:
        """Get robot status.

        Returns:
            Status dictionary
        """
        return {
            "robot_id": self.robot_id,
            "connected": self.connected,
            "status": "idle" if self.connected else "disconnected",
        }
