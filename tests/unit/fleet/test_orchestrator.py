"""Unit tests for fleet orchestrator.

TDD tests for FleetOrchestrator, Task, FleetRobot, RobotCapability.
"""

from datetime import UTC, datetime, timedelta
from unittest.mock import Mock

import pytest

from agent_ros_bridge.fleet.orchestrator import (FleetMetrics,
                                                 FleetOrchestrator, FleetRobot,
                                                 RobotCapability, RobotStatus,
                                                 Task, TaskStatus)


class TestRobotCapability:
    """Test RobotCapability data class."""

    def test_default_capabilities(self):
        """RobotCapability has sensible defaults."""
        caps = RobotCapability()
        assert caps.can_navigate is True
        assert caps.can_manipulate is False
        assert caps.can_lift is False
        assert caps.max_payload_kg == 0.0
        assert caps.max_speed_ms == 1.0
        assert caps.battery_hours == 4.0
        assert caps.ros_version == "ros2"
        assert caps.special_skills == set()

    def test_custom_capabilities(self):
        """RobotCapability can be customized."""
        caps = RobotCapability(
            can_navigate=True,
            can_manipulate=True,
            can_lift=True,
            max_payload_kg=10.0,
            max_speed_ms=2.5,
            battery_hours=8.0,
            ros_version="ros1",
            special_skills={"pick", "place"},
        )
        assert caps.can_manipulate is True
        assert caps.can_lift is True
        assert caps.max_payload_kg == 10.0
        assert caps.max_speed_ms == 2.5
        assert caps.battery_hours == 8.0
        assert caps.ros_version == "ros1"
        assert "pick" in caps.special_skills


class TestTask:
    """Test Task data class."""

    def test_task_default_values(self):
        """Task has sensible defaults."""
        task = Task()
        assert task.id is not None  # Auto-generated
        assert task.type == "navigate"
        assert task.priority == 5
        assert task.target_location is None
        assert task.payload_kg == 0.0
        assert task.status == TaskStatus.PENDING
        assert task.assigned_robot is None
        assert task.dependencies == []
        assert task.metadata == {}

    def test_task_custom_values(self):
        """Task can have custom values."""
        deadline = datetime.now(UTC) + timedelta(hours=1)
        task = Task(
            type="manipulate",
            priority=1,
            target_location="kitchen",
            payload_kg=5.0,
            deadline=deadline,
            dependencies=["task1", "task2"],
            metadata={"urgent": True},
        )
        assert task.type == "manipulate"
        assert task.priority == 1
        assert task.target_location == "kitchen"
        assert task.payload_kg == 5.0
        assert task.deadline == deadline
        assert task.dependencies == ["task1", "task2"]
        assert task.metadata["urgent"] is True

    def test_task_auto_id_generation(self):
        """Task auto-generates unique IDs."""
        task1 = Task()
        task2 = Task()
        assert task1.id != task2.id
        assert len(task1.id) == 8  # First 8 chars of UUID


class TestFleetRobot:
    """Test FleetRobot data class."""

    def test_fleet_robot_creation(self):
        """FleetRobot can be created."""
        caps = RobotCapability()
        robot = FleetRobot(robot_id="r1", name="Test Robot", capabilities=caps)
        assert robot.robot_id == "r1"
        assert robot.name == "Test Robot"
        assert robot.capabilities == caps
        assert robot.status == RobotStatus.IDLE
        assert robot.current_task is None
        assert robot.current_location is None
        assert robot.battery_percent == 100.0
        assert robot.total_tasks_completed == 0
        assert robot.ros_endpoint == "localhost"

    def test_fleet_robot_defaults(self):
        """FleetRobot has sensible defaults."""
        caps = RobotCapability()
        robot = FleetRobot(robot_id="r1", name="Test", capabilities=caps)
        assert robot.status == RobotStatus.IDLE
        assert robot.battery_percent == 100.0
        assert robot.total_tasks_completed == 0
        assert robot.ros_endpoint == "localhost"
        assert robot.last_seen is not None


class TestFleetOrchestrator:
    """Test FleetOrchestrator."""

    @pytest.fixture
    def orchestrator(self):
        """Create a fresh orchestrator."""
        return FleetOrchestrator()

    @pytest.fixture
    def sample_robot(self):
        """Create a sample robot."""
        return FleetRobot(
            robot_id="tb4_01",
            name="TurtleBot4 #1",
            capabilities=RobotCapability(can_navigate=True, battery_hours=4.0),
        )

    @pytest.fixture
    def sample_task(self):
        """Create a sample task."""
        return Task(type="navigate", target_location="zone_a", priority=3)

    def test_orchestrator_creation(self, orchestrator):
        """Orchestrator can be created."""
        assert orchestrator.robots == {}
        assert orchestrator.tasks == {}
        assert orchestrator.task_queue == []
        assert orchestrator.running is False

    @pytest.mark.asyncio
    async def test_orchestrator_start_stop(self, orchestrator):
        """Orchestrator can start and stop."""
        await orchestrator.start()
        assert orchestrator.running is True
        assert orchestrator._allocation_loop_task is not None

        await orchestrator.stop()
        assert orchestrator.running is False

    @pytest.mark.asyncio
    async def test_add_robot(self, orchestrator, sample_robot):
        """Robot can be added to fleet."""
        result = await orchestrator.add_robot(sample_robot)
        assert result is True
        assert "tb4_01" in orchestrator.robots
        assert orchestrator.robots["tb4_01"].name == "TurtleBot4 #1"

    @pytest.mark.asyncio
    async def test_add_duplicate_robot_fails(self, orchestrator, sample_robot):
        """Adding duplicate robot returns False."""
        await orchestrator.add_robot(sample_robot)
        result = await orchestrator.add_robot(sample_robot)
        assert result is False

    @pytest.mark.asyncio
    async def test_remove_robot(self, orchestrator, sample_robot):
        """Robot can be removed from fleet."""
        await orchestrator.add_robot(sample_robot)
        result = await orchestrator.remove_robot("tb4_01")
        assert result is True
        assert "tb4_01" not in orchestrator.robots

    @pytest.mark.asyncio
    async def test_remove_unknown_robot_fails(self, orchestrator):
        """Removing unknown robot returns False."""
        result = await orchestrator.remove_robot("unknown")
        assert result is False

    @pytest.mark.asyncio
    async def test_submit_task(self, orchestrator, sample_task):
        """Task can be submitted."""
        task_id = await orchestrator.submit_task(sample_task)
        assert task_id is not None
        assert task_id in orchestrator.tasks
        assert sample_task in orchestrator.task_queue

    @pytest.mark.asyncio
    async def test_submit_multiple_tasks_priority_order(self, orchestrator):
        """Tasks are queued in priority order."""
        task_low = Task(priority=5)
        task_high = Task(priority=1)
        task_medium = Task(priority=3)

        await orchestrator.submit_task(task_low)
        await orchestrator.submit_task(task_high)
        await orchestrator.submit_task(task_medium)

        # Should be ordered: high (1), medium (3), low (5)
        assert orchestrator.task_queue[0].priority == 1
        assert orchestrator.task_queue[1].priority == 3
        assert orchestrator.task_queue[2].priority == 5

    @pytest.mark.asyncio
    async def test_cancel_pending_task(self, orchestrator, sample_task):
        """Pending task can be cancelled."""
        task_id = await orchestrator.submit_task(sample_task)
        result = await orchestrator.cancel_task(task_id)
        assert result is True
        assert orchestrator.tasks[task_id].status == TaskStatus.CANCELLED

    @pytest.mark.asyncio
    async def test_cancel_unknown_task_fails(self, orchestrator):
        """Cancelling unknown task returns False."""
        result = await orchestrator.cancel_task("unknown")
        assert result is False

    @pytest.mark.asyncio
    async def test_cancel_completed_task_fails(self, orchestrator, sample_task):
        """Cancelling completed task returns False."""
        task_id = await orchestrator.submit_task(sample_task)
        orchestrator.tasks[task_id].status = TaskStatus.COMPLETED
        result = await orchestrator.cancel_task(task_id)
        assert result is False

    @pytest.mark.asyncio
    async def test_update_robot_status(self, orchestrator, sample_robot):
        """Robot status can be updated."""
        await orchestrator.add_robot(sample_robot)

        await orchestrator.update_robot_status(
            "tb4_01", status=RobotStatus.BUSY, location="kitchen", battery=85.0
        )

        robot = orchestrator.robots["tb4_01"]
        assert robot.status == RobotStatus.BUSY
        assert robot.current_location == "kitchen"
        assert robot.battery_percent == 85.0

    @pytest.mark.asyncio
    async def test_update_unknown_robot_silently_fails(self, orchestrator):
        """Updating unknown robot fails silently."""
        # Should not raise exception
        await orchestrator.update_robot_status("unknown", status=RobotStatus.BUSY)

    def test_get_metrics_empty_fleet(self, orchestrator):
        """Metrics for empty fleet are zeros."""
        metrics = orchestrator.get_metrics()
        assert isinstance(metrics, FleetMetrics)
        assert metrics.total_robots == 0
        assert metrics.active_robots == 0
        assert metrics.idle_robots == 0
        assert metrics.tasks_pending == 0

    @pytest.mark.asyncio
    async def test_get_metrics_with_robots_and_tasks(self, orchestrator, sample_robot, sample_task):
        """Metrics reflect fleet state."""
        await orchestrator.add_robot(sample_robot)
        await orchestrator.submit_task(sample_task)

        metrics = orchestrator.get_metrics()
        assert metrics.total_robots == 1
        assert metrics.idle_robots == 1
        assert metrics.tasks_pending == 1

    @pytest.mark.asyncio
    async def test_task_lifecycle_callbacks(self, orchestrator, sample_robot, sample_task):
        """Task lifecycle callbacks are invoked."""
        assigned_callback = Mock()
        completed_callback = Mock()
        failed_callback = Mock()

        orchestrator.on_task_assigned = assigned_callback
        orchestrator.on_task_completed = completed_callback
        orchestrator.on_task_failed = failed_callback

        await orchestrator.add_robot(sample_robot)
        task_id = await orchestrator.submit_task(sample_task)

        # Manually trigger callbacks for testing
        task = orchestrator.tasks[task_id]
        robot = orchestrator.robots["tb4_01"]

        if orchestrator.on_task_assigned:
            orchestrator.on_task_assigned(task, robot)
        assigned_callback.assert_called_once()


class TestTaskStatusEnum:
    """Test TaskStatus enum."""

    def test_task_status_values(self):
        """TaskStatus has all expected values."""
        assert TaskStatus.PENDING is not None
        assert TaskStatus.ASSIGNED is not None
        assert TaskStatus.EXECUTING is not None
        assert TaskStatus.COMPLETED is not None
        assert TaskStatus.FAILED is not None
        assert TaskStatus.CANCELLED is not None


class TestRobotStatusEnum:
    """Test RobotStatus enum."""

    def test_robot_status_values(self):
        """RobotStatus has all expected values."""
        assert RobotStatus.IDLE is not None
        assert RobotStatus.BUSY is not None
        assert RobotStatus.CHARGING is not None
        assert RobotStatus.OFFLINE is not None
        assert RobotStatus.ERROR is not None
