"""Tests for actions module."""

import asyncio
from datetime import UTC, datetime
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from agent_ros_bridge.actions import (
    ActionFeedback,
    ActionGoal,
    ActionResult,
    ActionStatus,
    BaseActionClient,
    SimulatedActionClient,
    create_action_client,
)


class TestActionStatus:
    """Test ActionStatus enum."""

    def test_enum_values(self):
        """Test that all enum values exist."""
        assert ActionStatus.IDLE.name == "IDLE"
        assert ActionStatus.PENDING.name == "PENDING"
        assert ActionStatus.ACTIVE.name == "ACTIVE"
        assert ActionStatus.PREEMPTING.name == "PREEMPTING"
        assert ActionStatus.SUCCEEDED.name == "SUCCEEDED"
        assert ActionStatus.ABORTED.name == "ABORTED"
        assert ActionStatus.REJECTED.name == "REJECTED"
        assert ActionStatus.CANCELED.name == "CANCELED"
        assert ActionStatus.LOST.name == "LOST"


class TestActionGoal:
    """Test ActionGoal dataclass."""

    def test_default_creation(self):
        """Test creating ActionGoal with defaults."""
        goal = ActionGoal(
            goal_id="goal123",
            goal_data={"x": 1.0, "y": 2.0},
            action_type="nav2_msgs/action/NavigateToPose",
        )
        assert goal.goal_id == "goal123"
        assert goal.goal_data == {"x": 1.0, "y": 2.0}
        assert goal.action_type == "nav2_msgs/action/NavigateToPose"
        assert goal.timeout_sec == 30.0
        assert isinstance(goal.created_at, datetime)

    def test_custom_timeout(self):
        """Test creating ActionGoal with custom timeout."""
        goal = ActionGoal(
            goal_id="goal456",
            goal_data={},
            action_type="test",
            timeout_sec=60.0,
        )
        assert goal.timeout_sec == 60.0


class TestActionFeedback:
    """Test ActionFeedback dataclass."""

    def test_default_creation(self):
        """Test creating ActionFeedback."""
        feedback = ActionFeedback(
            goal_id="goal123",
            feedback_data={"progress": 0.5},
        )
        assert feedback.goal_id == "goal123"
        assert feedback.feedback_data == {"progress": 0.5}
        assert isinstance(feedback.timestamp, datetime)


class TestActionResult:
    """Test ActionResult dataclass."""

    def test_default_creation(self):
        """Test creating ActionResult with defaults."""
        result = ActionResult(
            goal_id="goal123",
            success=True,
            status=ActionStatus.SUCCEEDED,
        )
        assert result.goal_id == "goal123"
        assert result.success is True
        assert result.status == ActionStatus.SUCCEEDED
        assert result.result_data == {}
        assert result.error_message is None
        assert result.execution_time_sec == 0.0

    def test_with_error(self):
        """Test creating ActionResult with error."""
        result = ActionResult(
            goal_id="goal456",
            success=False,
            status=ActionStatus.ABORTED,
            error_message="Timeout",
            execution_time_sec=30.0,
        )
        assert result.success is False
        assert result.error_message == "Timeout"
        assert result.execution_time_sec == 30.0


class TestBaseActionClient:
    """Test BaseActionClient class."""

    @pytest.fixture
    def client(self):
        """Create base action client fixture."""
        return BaseActionClient("test_action", "test_type", "ros2")

    def test_init(self, client):
        """Test initialization."""
        assert client.action_name == "test_action"
        assert client.action_type == "test_type"
        assert client.ros_version == "ros2"
        assert client.status == ActionStatus.IDLE
        assert client._feedback_callbacks == []
        assert client._result_callbacks == []
        assert client._current_goal is None
        assert client._connected is False

    def test_register_feedback_callback(self, client):
        """Test registering feedback callback."""
        callback = MagicMock()
        client.register_feedback_callback(callback)
        assert callback in client._feedback_callbacks

    def test_register_result_callback(self, client):
        """Test registering result callback."""
        callback = MagicMock()
        client.register_result_callback(callback)
        assert callback in client._result_callbacks

    def test_notify_feedback(self, client):
        """Test notifying feedback callbacks."""
        callback1 = MagicMock()
        callback2 = MagicMock()
        client.register_feedback_callback(callback1)
        client.register_feedback_callback(callback2)

        feedback = ActionFeedback(goal_id="test", feedback_data={"progress": 0.5})
        client._notify_feedback(feedback)

        callback1.assert_called_once_with(feedback)
        callback2.assert_called_once_with(feedback)

    def test_notify_feedback_error_handling(self, client, caplog):
        """Test feedback callback error handling."""
        callback = MagicMock(side_effect=Exception("Callback error"))
        client.register_feedback_callback(callback)

        feedback = ActionFeedback(goal_id="test", feedback_data={})
        with caplog.at_level("ERROR"):
            client._notify_feedback(feedback)
            assert "Feedback callback error" in caplog.text

    def test_notify_result(self, client):
        """Test notifying result callbacks."""
        callback = MagicMock()
        client.register_result_callback(callback)

        result = ActionResult(goal_id="test", success=True, status=ActionStatus.SUCCEEDED)
        client._notify_result(result)

        callback.assert_called_once_with(result)

    def test_connect_not_implemented(self, client):
        """Test connect raises NotImplementedError."""
        with pytest.raises(NotImplementedError):
            asyncio.run(client.connect())

    def test_disconnect_not_implemented(self, client):
        """Test disconnect raises NotImplementedError."""
        with pytest.raises(NotImplementedError):
            asyncio.run(client.disconnect())

    def test_send_goal_not_implemented(self, client):
        """Test send_goal raises NotImplementedError."""
        with pytest.raises(NotImplementedError):
            asyncio.run(client.send_goal({}))

    def test_cancel_goal_not_implemented(self, client):
        """Test cancel_goal raises NotImplementedError."""
        with pytest.raises(NotImplementedError):
            asyncio.run(client.cancel_goal())


class TestSimulatedActionClient:
    """Test SimulatedActionClient class."""

    @pytest.fixture
    def client(self):
        """Create simulated action client fixture."""
        return SimulatedActionClient("navigate_to_pose", "nav2_msgs/action/NavigateToPose")

    @pytest.mark.asyncio
    async def test_connect(self, client, caplog):
        """Test connect method."""
        with caplog.at_level("INFO"):
            result = await client.connect()
            assert result is True
            assert client._connected is True
            assert "Simulated action client" in caplog.text

    @pytest.mark.asyncio
    async def test_disconnect(self, client):
        """Test disconnect method."""
        await client.connect()
        await client.disconnect()
        assert client._connected is False

    @pytest.mark.asyncio
    async def test_send_goal_navigation(self, client, caplog):
        """Test send_goal for navigation action."""
        await client.connect()

        feedback_received = []
        client.register_feedback_callback(lambda f: feedback_received.append(f))

        with caplog.at_level("INFO"):
            result = await client.send_goal({"target_pose": {"x": 1.0, "y": 2.0}})

            assert result.success is True
            assert result.status == ActionStatus.SUCCEEDED
            assert "message" in result.result_data
            assert result.execution_time_sec > 0

            # Should have received feedback
            assert len(feedback_received) > 0
            assert "progress" in feedback_received[0].feedback_data
            assert "distance_remaining" in feedback_received[0].feedback_data

    @pytest.mark.asyncio
    async def test_send_goal_manipulation(self, caplog):
        """Test send_goal for manipulation action."""
        client = SimulatedActionClient("move_arm", "manipulation_msgs/action/MoveArm")
        await client.connect()

        with caplog.at_level("INFO"):
            result = await client.send_goal({"joint_positions": [0.0, 0.0]})
            assert result.success is True

    @pytest.mark.asyncio
    async def test_send_goal_generic(self, caplog):
        """Test send_goal for generic action."""
        client = SimulatedActionClient("do_something", "std_msgs/Empty")
        await client.connect()

        result = await client.send_goal({})
        assert result.success is True

    @pytest.mark.asyncio
    async def test_cancel_goal(self, client):
        """Test cancel_goal method."""
        await client.connect()
        result = await client.cancel_goal()
        assert result is True
        assert client.status == ActionStatus.CANCELED

    @pytest.mark.asyncio
    async def test_generate_simulated_feedback_navigation(self, client):
        """Test _generate_simulated_feedback for navigation."""
        feedback = client._generate_simulated_feedback(5, 10, {})
        assert "distance_remaining" in feedback
        assert "current_speed" in feedback
        assert "progress" in feedback
        assert feedback["progress"] == 0.6

    @pytest.mark.asyncio
    async def test_generate_simulated_feedback_manipulation(self, caplog):
        """Test _generate_simulated_feedback for manipulation."""
        client = SimulatedActionClient("move_arm", "manipulation_msgs/action/MoveArm")
        feedback = client._generate_simulated_feedback(3, 10, {})
        assert "current_joint" in feedback
        assert "progress" in feedback

    @pytest.mark.asyncio
    async def test_generate_simulated_feedback_generic(self, caplog):
        """Test _generate_simulated_feedback for generic action."""
        client = SimulatedActionClient("do_something", "std_msgs/Empty")
        feedback = client._generate_simulated_feedback(7, 10, {})
        assert "progress" in feedback
        assert "step" in feedback


class TestCreateActionClient:
    """Test create_action_client factory function."""

    def test_create_simulated_when_rclpy_not_available(self, caplog):
        """Test that simulated client is created when rclpy not available."""
        with patch.dict("sys.modules", {"rclpy": None}):
            with caplog.at_level("WARNING"):
                client = create_action_client("test", "test_type", "ros2")
                assert isinstance(client, SimulatedActionClient)
                assert "rclpy not available" in caplog.text

    def test_create_simulated_for_ros1(self, caplog):
        """Test that simulated client is created for ROS1."""
        with caplog.at_level("WARNING"):
            client = create_action_client("test", "test_type", "ros1")
            assert isinstance(client, SimulatedActionClient)
            # Note: SimulatedActionClient inherits ros_version from BaseActionClient
            # which defaults to "ros2" in the constructor

    def test_create_ros2_when_rclpy_available(self):
        """Test that ROS2 client is created when rclpy available."""
        mock_rclpy = MagicMock()
        with patch.dict("sys.modules", {"rclpy": mock_rclpy}):
            client = create_action_client("test", "test_type", "ros2")
            # Should try to create ROS2ActionClient
            from agent_ros_bridge.actions import ROS2ActionClient

            assert isinstance(client, ROS2ActionClient)
