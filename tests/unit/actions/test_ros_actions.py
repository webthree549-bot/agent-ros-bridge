"""Unit tests for ROS Action Client.

Tests for ActionClient, ActionStatus, ActionGoal, ActionFeedback, ActionResult.
"""

from datetime import UTC, datetime
from unittest.mock import Mock

import pytest

from agent_ros_bridge.actions import (
    ActionFeedback,
    ActionGoal,
    ActionResult,
    ActionStatus,
    BaseActionClient,
)


class TestActionStatus:
    """Test ActionStatus enum."""

    def test_action_status_values(self):
        """ActionStatus has all expected values."""
        assert ActionStatus.IDLE is not None
        assert ActionStatus.PENDING is not None
        assert ActionStatus.ACTIVE is not None
        assert ActionStatus.PREEMPTING is not None
        assert ActionStatus.SUCCEEDED is not None
        assert ActionStatus.ABORTED is not None
        assert ActionStatus.REJECTED is not None
        assert ActionStatus.CANCELED is not None
        assert ActionStatus.LOST is not None

    def test_action_status_is_enum(self):
        """ActionStatus is an Enum."""
        assert isinstance(ActionStatus.IDLE, ActionStatus)


class TestActionGoal:
    """Test ActionGoal dataclass."""

    def test_action_goal_creation(self):
        """ActionGoal can be created."""
        goal = ActionGoal(
            goal_id="goal_001",
            goal_data={"pose": {"x": 1.0, "y": 2.0}},
            action_type="nav2_msgs/action/NavigateToPose",
            timeout_sec=60.0,
        )

        assert goal.goal_id == "goal_001"
        assert goal.goal_data["pose"]["x"] == 1.0
        assert goal.action_type == "nav2_msgs/action/NavigateToPose"
        assert goal.timeout_sec == 60.0
        assert goal.created_at is not None

    def test_action_goal_default_timeout(self):
        """ActionGoal has default timeout."""
        goal = ActionGoal(
            goal_id="goal_002",
            goal_data={},
            action_type="test_action",
        )

        assert goal.timeout_sec == 30.0  # Default value

    def test_action_goal_auto_timestamp(self):
        """ActionGoal auto-generates timestamp."""
        before = datetime.now(UTC)
        goal = ActionGoal(
            goal_id="goal_003",
            goal_data={},
            action_type="test_action",
        )
        after = datetime.now(UTC)

        assert before <= goal.created_at <= after


class TestActionFeedback:
    """Test ActionFeedback dataclass."""

    def test_action_feedback_creation(self):
        """ActionFeedback can be created."""
        feedback = ActionFeedback(
            goal_id="goal_001",
            feedback_data={"distance_remaining": 5.0},
        )

        assert feedback.goal_id == "goal_001"
        assert feedback.feedback_data["distance_remaining"] == 5.0
        assert feedback.timestamp is not None

    def test_action_feedback_auto_timestamp(self):
        """ActionFeedback auto-generates timestamp."""
        before = datetime.now(UTC)
        feedback = ActionFeedback(
            goal_id="goal_001",
            feedback_data={},
        )
        after = datetime.now(UTC)

        assert before <= feedback.timestamp <= after


class TestActionResult:
    """Test ActionResult dataclass."""

    def test_action_result_success(self):
        """ActionResult for successful execution."""
        result = ActionResult(
            goal_id="goal_001",
            success=True,
            status=ActionStatus.SUCCEEDED,
            result_data={"final_pose": {"x": 1.0, "y": 2.0}},
            execution_time_sec=10.5,
        )

        assert result.goal_id == "goal_001"
        assert result.success is True
        assert result.status == ActionStatus.SUCCEEDED
        assert result.result_data["final_pose"]["x"] == 1.0
        assert result.execution_time_sec == 10.5
        assert result.error_message is None

    def test_action_result_failure(self):
        """ActionResult for failed execution."""
        result = ActionResult(
            goal_id="goal_002",
            success=False,
            status=ActionStatus.ABORTED,
            error_message="Obstacle detected",
            execution_time_sec=5.0,
        )

        assert result.success is False
        assert result.status == ActionStatus.ABORTED
        assert result.error_message == "Obstacle detected"

    def test_action_result_default_values(self):
        """ActionResult has default values."""
        result = ActionResult(
            goal_id="goal_003",
            success=True,
            status=ActionStatus.SUCCEEDED,
        )

        assert result.result_data == {}
        assert result.error_message is None
        assert result.execution_time_sec == 0.0


class TestBaseActionClient:
    """Test BaseActionClient."""

    def test_base_client_creation(self):
        """BaseActionClient can be created."""
        client = BaseActionClient(
            action_name="navigate_to_pose",
            action_type="nav2_msgs/action/NavigateToPose",
        )

        assert client.action_name == "navigate_to_pose"
        assert client.action_type == "nav2_msgs/action/NavigateToPose"
        assert client._connected is False

    def test_base_client_initial_state(self):
        """BaseActionClient starts in correct state."""
        client = BaseActionClient(
            action_name="test_action",
            action_type="test_action/Action",
        )

        assert client._connected is False
        assert client._feedback_callbacks == []
        assert client._result_callbacks == []

    @pytest.mark.asyncio
    async def test_connect_not_implemented(self):
        """BaseActionClient.connect raises NotImplementedError."""
        client = BaseActionClient(
            action_name="test_action",
            action_type="test_action/Action",
        )

        with pytest.raises(NotImplementedError):
            await client.connect()

    @pytest.mark.asyncio
    async def test_disconnect_not_implemented(self):
        """BaseActionClient.disconnect raises NotImplementedError."""
        client = BaseActionClient(
            action_name="test_action",
            action_type="test_action/Action",
        )

        with pytest.raises(NotImplementedError):
            await client.disconnect()

    @pytest.mark.asyncio
    async def test_send_goal_not_implemented(self):
        """BaseActionClient.send_goal raises NotImplementedError."""
        client = BaseActionClient(
            action_name="test_action",
            action_type="test_action/Action",
        )

        with pytest.raises(NotImplementedError):
            await client.send_goal({})

    @pytest.mark.asyncio
    async def test_cancel_goal_not_implemented(self):
        """BaseActionClient.cancel_goal raises NotImplementedError."""
        client = BaseActionClient(
            action_name="test_action",
            action_type="test_action/Action",
        )

        with pytest.raises(NotImplementedError):
            await client.cancel_goal()

    def test_register_feedback_callback(self):
        """BaseActionClient can register feedback callback."""
        client = BaseActionClient(
            action_name="test_action",
            action_type="test_action/Action",
        )

        callback = Mock()
        client.register_feedback_callback(callback)

        assert callback in client._feedback_callbacks

    def test_register_result_callback(self):
        """BaseActionClient can register result callback."""
        client = BaseActionClient(
            action_name="test_action",
            action_type="test_action/Action",
        )

        callback = Mock()
        client.register_result_callback(callback)

        assert callback in client._result_callbacks

    def test_notify_feedback(self):
        """_notify_feedback calls all registered callbacks."""
        client = BaseActionClient(
            action_name="test_action",
            action_type="test_action/Action",
        )

        callback1 = Mock()
        callback2 = Mock()
        client.register_feedback_callback(callback1)
        client.register_feedback_callback(callback2)

        feedback = ActionFeedback(goal_id="test", feedback_data={})
        client._notify_feedback(feedback)

        callback1.assert_called_once_with(feedback)
        callback2.assert_called_once_with(feedback)

    def test_notify_result(self):
        """_notify_result calls all registered callbacks."""
        client = BaseActionClient(
            action_name="test_action",
            action_type="test_action/Action",
        )

        callback1 = Mock()
        callback2 = Mock()
        client.register_result_callback(callback1)
        client.register_result_callback(callback2)

        result = ActionResult(
            goal_id="test",
            success=True,
            status=ActionStatus.SUCCEEDED,
        )
        client._notify_result(result)

        callback1.assert_called_once_with(result)
        callback2.assert_called_once_with(result)
