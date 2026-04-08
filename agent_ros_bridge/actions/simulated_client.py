"""Simulated action client for testing without ROS."""

import asyncio
import logging
import uuid
from datetime import UTC, datetime
from typing import Any

from agent_ros_bridge.actions.base_client import BaseActionClient
from agent_ros_bridge.actions.types import ActionFeedback, ActionGoal, ActionResult, ActionStatus

logger = logging.getLogger("ros_actions")


class SimulatedActionClient(BaseActionClient):
    """Simulated action client for testing without ROS."""

    async def connect(self) -> bool:
        """Simulated connect."""
        self._connected = True
        logger.info(f"✅ Simulated action client: {self.action_name}")
        return True

    async def disconnect(self):
        """Simulated disconnect."""
        self._connected = False

    async def send_goal(
        self, goal_data: dict[str, Any], _timeout_sec: float = 30.0
    ) -> ActionResult:
        """Simulated goal execution with feedback."""
        goal_id = str(uuid.uuid4())[:8]

        self._current_goal = ActionGoal(
            goal_id=goal_id, goal_data=goal_data, action_type=self.action_type
        )

        self.status = ActionStatus.ACTIVE
        self._start_time = datetime.now(UTC)

        logger.info(f"▶️  Simulated action: {self.action_name} goal {goal_id}")

        # Simulate execution with feedback
        steps = 10
        for i in range(steps):
            await asyncio.sleep(0.5)

            # Generate simulated feedback based on action type
            feedback_data = self._generate_simulated_feedback(i, steps, goal_data)

            feedback = ActionFeedback(goal_id=goal_id, feedback_data=feedback_data)
            self._notify_feedback(feedback)

            logger.debug(f"Simulated feedback: {feedback_data}")

        # Complete successfully
        execution_time = (datetime.now(UTC) - self._start_time).total_seconds()
        self.status = ActionStatus.SUCCEEDED

        result = ActionResult(
            goal_id=goal_id,
            success=True,
            status=ActionStatus.SUCCEEDED,
            result_data={"message": "Simulated action completed"},
            execution_time_sec=execution_time,
        )

        self._notify_result(result)
        logger.info(f"✅ Simulated action completed: {goal_id}")

        return result

    async def cancel_goal(self) -> bool:
        """Simulated cancel."""
        self.status = ActionStatus.CANCELED
        return True

    def _generate_simulated_feedback(
        self, step: int, total: int, _goal_data: dict
    ) -> dict[str, Any]:
        """Generate simulated feedback based on action type."""
        progress = (step + 1) / total

        if "navigate" in self.action_name.lower():
            return {
                "distance_remaining": 10.0 * (1 - progress),
                "current_speed": 0.5,
                "estimated_time_remaining": 10.0 * (1 - progress),
                "progress": progress,
            }
        elif "manipulate" in self.action_name.lower() or "move" in self.action_name.lower():
            return {"current_joint": step, "progress": progress, "status": "moving"}
        else:
            return {"progress": progress, "step": step, "status": "active"}
