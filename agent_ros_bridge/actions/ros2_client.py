"""ROS2 action client for Agent ROS Bridge."""

import asyncio
import contextlib
import logging
import uuid
from datetime import UTC, datetime
from typing import Any

from agent_ros_bridge.actions.base_client import BaseActionClient
from agent_ros_bridge.actions.types import ActionFeedback, ActionGoal, ActionResult, ActionStatus

logger = logging.getLogger("ros_actions")


class ROS2ActionClient(BaseActionClient):
    """ROS2 action client using rclpy."""

    def __init__(self, action_name: str, action_type: str):
        """Initialize ROS2 action client.

        Args:
            action_name: Name of the ROS2 action.
            action_type: Type of the action.
        """
        super().__init__(action_name, action_type, "ros2")
        self._goal_handle: Any | None = None
        self._node: Any | None = None

    async def connect(self) -> bool:
        """Connect to ROS2 action server."""
        try:
            import rclpy
            from rclpy.action import ActionClient
            from rclpy.node import Node  # noqa: F401

            # Get or create node
            with contextlib.suppress(Exception):
                self._node = rclpy.create_node(
                    f"action_client_{self.action_name.replace('/', '_')}"
                )

            # Create action client
            self._client = ActionClient(self._node, self.action_type, self.action_name)

            # Wait for server
            if not self._client.wait_for_server(timeout_sec=5.0):
                logger.error(f"Action server '{self.action_name}' not available")
                return False

            self._connected = True
            logger.info(f"✅ Connected to ROS2 action: {self.action_name}")
            return True

        except ImportError:
            logger.error("rclpy not available")
            return False
        except Exception as e:
            logger.error(f"Failed to connect to action server: {e}")
            return False

    async def disconnect(self):
        """Disconnect from action server."""
        if self._client:
            self._client.destroy()
        self._connected = False

    async def send_goal(self, goal_data: dict[str, Any], timeout_sec: float = 30.0) -> ActionResult:
        """Send goal to ROS2 action server."""
        if not self._connected:
            return ActionResult(
                goal_id="",
                success=False,
                status=ActionStatus.LOST,
                error_message="Not connected to action server",
            )

        goal_id = str(uuid.uuid4())[:8]

        goal = ActionGoal(
            goal_id=goal_id,
            goal_data=goal_data,
            action_type=self.action_type,
            timeout_sec=timeout_sec,
        )

        self._current_goal = goal
        self._start_time = datetime.now(UTC)
        self.status = ActionStatus.PENDING

        try:
            # Create goal message
            goal_msg = self._create_goal_message(goal_data)

            # Send goal
            if self._client is None:
                return ActionResult(
                    goal_id=goal_id,
                    success=False,
                    status=ActionStatus.LOST,
                    error_message="Client not connected",
                )
            self._goal_handle = await self._client.send_goal_async(
                goal_msg, feedback_callback=self._on_feedback
            )

            if self._goal_handle is None or not self._goal_handle.accepted:
                self.status = ActionStatus.REJECTED
                return ActionResult(
                    goal_id=goal_id,
                    success=False,
                    status=ActionStatus.REJECTED,
                    error_message="Goal rejected by server",
                )

            self.status = ActionStatus.ACTIVE
            logger.info(f"▶️  Action goal accepted: {goal_id}")

            # Wait for result with timeout
            if self._goal_handle is None:
                return ActionResult(
                    goal_id=goal_id,
                    success=False,
                    status=ActionStatus.LOST,
                    error_message="Goal handle is None",
                )
            result_future = self._goal_handle.get_result_async()

            try:
                result_response = await asyncio.wait_for(
                    asyncio.wrap_future(result_future), timeout=timeout_sec
                )

                execution_time = (datetime.now(UTC) - self._start_time).total_seconds()

                # Check result status
                if result_response.status == 4:  # SUCCEEDED
                    self.status = ActionStatus.SUCCEEDED
                    result = ActionResult(
                        goal_id=goal_id,
                        success=True,
                        status=ActionStatus.SUCCEEDED,
                        result_data=self._parse_result(result_response.result),
                        execution_time_sec=execution_time,
                    )
                elif result_response.status == 5:  # ABORTED
                    self.status = ActionStatus.ABORTED
                    result = ActionResult(
                        goal_id=goal_id,
                        success=False,
                        status=ActionStatus.ABORTED,
                        error_message="Goal aborted by server",
                        execution_time_sec=execution_time,
                    )
                elif result_response.status == 2:  # CANCELED
                    self.status = ActionStatus.CANCELED
                    result = ActionResult(
                        goal_id=goal_id,
                        success=False,
                        status=ActionStatus.CANCELED,
                        execution_time_sec=execution_time,
                    )
                else:
                    self.status = ActionStatus.ABORTED
                    result = ActionResult(
                        goal_id=goal_id,
                        success=False,
                        status=ActionStatus.ABORTED,
                        error_message=f"Unknown status: {result_response.status}",
                        execution_time_sec=execution_time,
                    )

                self._notify_result(result)
                return result

            except TimeoutError:
                self.status = ActionStatus.ABORTED
                await self.cancel_goal()
                return ActionResult(
                    goal_id=goal_id,
                    success=False,
                    status=ActionStatus.ABORTED,
                    error_message=f"Timeout after {timeout_sec}s",
                    execution_time_sec=timeout_sec,
                )

        except Exception as e:
            logger.error(f"Action execution failed: {e}")
            self.status = ActionStatus.ABORTED
            return ActionResult(
                goal_id=goal_id,
                success=False,
                status=ActionStatus.ABORTED,
                error_message=str(e),
            )

    async def cancel_goal(self) -> bool:
        """Cancel current goal."""
        if self._goal_handle and self.status == ActionStatus.ACTIVE:
            cancel_future = self._goal_handle.cancel_goal_async()
            await asyncio.wrap_future(cancel_future)
            self.status = ActionStatus.CANCELED
            return True
        return False

    def _on_feedback(self, feedback_msg):
        """Handle feedback from action server."""
        feedback_data = self._parse_feedback(feedback_msg)

        feedback = ActionFeedback(
            goal_id=self._current_goal.goal_id if self._current_goal else "",
            feedback_data=feedback_data,
        )

        self._notify_feedback(feedback)

    def _create_goal_message(self, goal_data: dict[str, Any]):
        """Create goal message from dict (simplified)."""

        class GoalMsg:
            def __init__(self, data):
                for k, v in data.items():
                    setattr(self, k, v)

        return GoalMsg(goal_data)

    def _parse_feedback(self, feedback_msg) -> dict[str, Any]:
        """Parse feedback message to dict."""
        result = {}
        for attr in dir(feedback_msg):
            if not attr.startswith("_"):
                try:
                    val = getattr(feedback_msg, attr)
                    if not callable(val):
                        result[attr] = val
                except Exception:
                    continue
        return result

    def _parse_result(self, result_msg) -> dict[str, Any]:
        """Parse result message to dict."""
        return self._parse_feedback(result_msg)
