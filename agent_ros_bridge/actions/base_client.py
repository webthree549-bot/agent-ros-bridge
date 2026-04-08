"""Base action client for Agent ROS Bridge."""

import asyncio
import logging
from abc import abstractmethod
from collections.abc import Callable
from datetime import UTC, datetime
from typing import Any

from agent_ros_bridge.actions.types import ActionFeedback, ActionGoal, ActionResult, ActionStatus

logger = logging.getLogger("ros_actions")


class BaseActionClient:
    """Base class for ROS action clients."""

    def __init__(self, action_name: str, action_type: str, ros_version: str = "ros2"):
        """Initialize action client with name and type.

        Args:
            action_name: Name of the ROS action.
            action_type: Type of the action (e.g., "nav2_msgs/action/NavigateToPose").
            ros_version: ROS version ("ros1" or "ros2").
        """
        self.action_name = action_name
        self.action_type = action_type
        self.ros_version = ros_version
        self.status = ActionStatus.IDLE
        self._feedback_callbacks: list[Callable[[ActionFeedback], None]] = []
        self._result_callbacks: list[Callable[[ActionResult], None]] = []
        self._current_goal: ActionGoal | None = None
        self._start_time: datetime | None = None
        self._client: Any | None = None
        self._connected = False

    @property
    def connected(self) -> bool:
        """Public accessor for connection status."""
        return self._connected

    @abstractmethod
    async def connect(self) -> bool:
        """Connect to action server."""
        raise NotImplementedError

    @abstractmethod
    async def disconnect(self):
        """Disconnect from action server."""
        raise NotImplementedError

    @abstractmethod
    async def send_goal(
        self, goal_data: dict[str, Any], timeout_sec: float = 30.0
    ) -> ActionResult:
        """Send goal to action server."""
        raise NotImplementedError

    @abstractmethod
    async def cancel_goal(self) -> bool:
        """Cancel current goal."""
        raise NotImplementedError

    def register_feedback_callback(self, callback: Callable[[ActionFeedback], None]):
        """Register callback for feedback updates."""
        self._feedback_callbacks.append(callback)

    def register_result_callback(self, callback: Callable[[ActionResult], None]):
        """Register callback for results."""
        self._result_callbacks.append(callback)

    def _notify_feedback(self, feedback: ActionFeedback):
        """Notify all feedback callbacks."""
        for cb in self._feedback_callbacks:
            try:
                cb(feedback)
            except Exception as e:
                logger.error(f"Feedback callback error: {e}")

    def _notify_result(self, result: ActionResult):
        """Notify all result callbacks."""
        for cb in self._result_callbacks:
            try:
                cb(result)
            except Exception as e:
                logger.error(f"Result callback error: {e}")
