"""Tests for autonomous behaviors module."""

import json
from unittest.mock import AsyncMock, MagicMock, Mock, patch

import pytest

from agent_ros_bridge.integrations.autonomous_behaviors import (
    AutonomousBehaviorManager,
    BehaviorStatus,
    ExplorationArea,
    Mission,
    MissionPlanner,
    MissionStatus,
    MissionStep,
    PatrolRoute,
    Waypoint,
    explore_autonomously,
    patrol_route,
)


class TestMissionStatus:
    """Test MissionStatus enum."""

    def test_status_values(self):
        """Test status enum values."""
        assert MissionStatus.PENDING.name == "PENDING"
        assert MissionStatus.RUNNING.name == "RUNNING"
        assert MissionStatus.COMPLETED.name == "COMPLETED"
        assert MissionStatus.FAILED.name == "FAILED"


class TestBehaviorStatus:
    """Test BehaviorStatus enum."""

    def test_status_values(self):
        """Test behavior status values."""
        assert BehaviorStatus.IDLE.name == "IDLE"
        assert BehaviorStatus.EXECUTING.name == "EXECUTING"
        assert BehaviorStatus.COMPLETED.name == "COMPLETED"


class TestWaypoint:
    """Test Waypoint dataclass."""

    def test_waypoint_creation(self):
        """Test creating waypoint."""
        wp = Waypoint(x=1.0, y=2.0, z=0.5, name="kitchen")
        assert wp.x == 1.0
        assert wp.y == 2.0
        assert wp.z == 0.5
        assert wp.name == "kitchen"
        assert wp.actions == []


class TestMissionStep:
    """Test MissionStep dataclass."""

    def test_step_creation(self):
        """Test creating mission step."""
        step = MissionStep(
            step_id="nav_1",
            action="navigate",
            parameters={"x": 10, "y": 5},
        )
        assert step.step_id == "nav_1"
        assert step.action == "navigate"
        assert step.retry_count == 0
        assert step.max_retries == 3


class TestMission:
    """Test Mission dataclass."""

    def test_mission_creation(self):
        """Test creating mission."""
        mission = Mission(
            mission_id="test_1",
            name="Test Mission",
            description="A test mission",
            steps=[],
        )
        assert mission.mission_id == "test_1"
        assert mission.status == MissionStatus.PENDING
        assert mission.current_step_index == 0


class TestExplorationArea:
    """Test ExplorationArea dataclass."""

    def test_area_creation(self):
        """Test creating exploration area."""
        area = ExplorationArea(
            bounds={"min_x": 0, "max_x": 10, "min_y": 0, "max_y": 10},
            resolution=1.0,
        )
        assert area.resolution == 1.0
        assert area.priority == "normal"


class TestPatrolRoute:
    """Test PatrolRoute dataclass."""

    def test_route_creation(self):
        """Test creating patrol route."""
        route = PatrolRoute(
            name="perimeter",
            waypoints=[Waypoint(x=0, y=0), Waypoint(x=10, y=10)],
            interval_minutes=30,
        )
        assert route.name == "perimeter"
        assert len(route.waypoints) == 2
        assert route.interval_minutes == 30


class TestMissionPlannerInitialization:
    """Test mission planner initialization."""

    def test_init_default(self):
        """Test default initialization."""
        planner = MissionPlanner()
        assert planner.execute_action is None
        assert planner.active_missions == {}
        assert planner._running is False

    def test_init_with_callback(self):
        """Test initialization with callback."""
        callback = AsyncMock()
        planner = MissionPlanner(callback)
        assert planner.execute_action is callback


class TestPlanExploration:
    """Test exploration planning."""

    def test_plan_exploration_systematic(self):
        """Test systematic exploration planning."""
        planner = MissionPlanner()
        area = ExplorationArea(
            bounds={"min_x": 0, "max_x": 5, "min_y": 0, "max_y": 5},
            resolution=2.0,
        )
        mission = planner.plan_exploration(area, strategy="systematic")

        assert mission.name.startswith("Explore")
        assert len(mission.steps) > 0
        assert "systematic" in mission.description

    def test_plan_exploration_spiral(self):
        """Test spiral exploration planning."""
        planner = MissionPlanner()
        area = ExplorationArea(
            bounds={"min_x": -5, "max_x": 5, "min_y": -5, "max_y": 5},
            resolution=1.0,
        )
        mission = planner.plan_exploration(area, strategy="spiral")

        assert mission.name.startswith("Explore")
        assert len(mission.steps) > 0


class TestPlanPatrol:
    """Test patrol planning."""

    def test_plan_patrol(self):
        """Test patrol mission planning."""
        planner = MissionPlanner()
        route = PatrolRoute(
            name="test_route",
            waypoints=[
                Waypoint(x=0, y=0),
                Waypoint(x=10, y=0),
            ],
            interval_minutes=15,
        )
        mission = planner.plan_patrol(route)

        assert "Patrol" in mission.name
        assert len(mission.steps) > 0
        assert mission.metadata["interval_minutes"] == 15


class TestPlanSearch:
    """Test search planning."""

    def test_plan_search(self):
        """Test search mission planning."""
        planner = MissionPlanner()
        area = ExplorationArea(
            bounds={"min_x": 0, "max_x": 5, "min_y": 0, "max_y": 5},
        )
        mission = planner.plan_search("red ball", area)

        assert "Search for" in mission.name
        assert "red ball" in mission.description
        assert mission.metadata["target"] == "red ball"


class TestExecuteMission:
    """Test mission execution."""

    @pytest.mark.asyncio
    async def test_execute_mission_simulation(self):
        """Test mission execution in simulation mode."""
        planner = MissionPlanner()
        mission = Mission(
            mission_id="test",
            name="Test",
            description="Test mission",
            steps=[
                MissionStep(step_id="step1", action="navigate", parameters={}),
            ],
        )

        result = await planner.execute_mission(mission)

        assert result["mission_id"] == "test"
        assert result["steps_total"] == 1
        assert result["steps_completed"] == 1
        assert mission.status == MissionStatus.COMPLETED

    @pytest.mark.asyncio
    async def test_execute_mission_with_action(self):
        """Test mission execution with action callback."""
        callback = AsyncMock(return_value={"success": True})
        planner = MissionPlanner(callback)
        mission = Mission(
            mission_id="test",
            name="Test",
            description="Test mission",
            steps=[
                MissionStep(step_id="step1", action="navigate", parameters={}),
            ],
        )

        result = await planner.execute_mission(mission)

        assert result["steps_completed"] == 1
        callback.assert_called_once()

    @pytest.mark.asyncio
    async def test_execute_mission_with_failure(self):
        """Test mission execution with step failure."""
        callback = AsyncMock(side_effect=Exception("Action failed"))
        planner = MissionPlanner(callback)
        mission = Mission(
            mission_id="test",
            name="Test",
            description="Test mission",
            steps=[
                MissionStep(step_id="step1", action="navigate", parameters={}),
            ],
        )

        result = await planner.execute_mission(mission)

        assert result["steps_failed"] == 1
        assert len(result["errors"]) == 1


class TestCancelMission:
    """Test mission cancellation."""

    def test_cancel_active_mission(self):
        """Test cancelling active mission."""
        planner = MissionPlanner()
        mission = Mission(
            mission_id="test",
            name="Test",
            description="Test mission",
            steps=[],
        )
        planner.active_missions["test"] = mission

        result = planner.cancel_mission("test")

        assert result is True
        assert mission.status == MissionStatus.CANCELLED

    def test_cancel_nonexistent_mission(self):
        """Test cancelling non-existent mission."""
        planner = MissionPlanner()
        result = planner.cancel_mission("nonexistent")
        assert result is False


class TestGetMissionStatus:
    """Test getting mission status."""

    def test_get_active_mission_status(self):
        """Test getting active mission status."""
        planner = MissionPlanner()
        mission = Mission(
            mission_id="test",
            name="Test",
            description="Test mission",
            steps=[
                MissionStep(step_id="step1", action="navigate", parameters={}),
                MissionStep(step_id="step2", action="capture", parameters={}),
            ],
            status=MissionStatus.RUNNING,
            current_step_index=1,
        )
        planner.active_missions["test"] = mission

        status = planner.get_mission_status("test")

        assert status["mission_id"] == "test"
        assert status["status"] == "RUNNING"
        assert status["progress"] == "1/2"

    def test_get_completed_mission_status(self):
        """Test getting completed mission status."""
        planner = MissionPlanner()
        from datetime import datetime

        mission = Mission(
            mission_id="test",
            name="Test",
            description="Test mission",
            steps=[],
            status=MissionStatus.COMPLETED,
            completed_at=datetime.now(),
        )
        planner.mission_history.append(mission)

        status = planner.get_mission_status("test")

        assert status["mission_id"] == "test"
        assert status["status"] == "COMPLETED"

    def test_get_nonexistent_mission_status(self):
        """Test getting non-existent mission status."""
        planner = MissionPlanner()
        status = planner.get_mission_status("nonexistent")
        assert status is None


class TestAutonomousBehaviorManager:
    """Test autonomous behavior manager."""

    def test_init(self):
        """Test initialization."""
        planner = MissionPlanner()
        manager = AutonomousBehaviorManager(planner)
        assert manager.planner is planner
        assert manager.active_behaviors == {}

    @pytest.mark.asyncio
    async def test_start_patrol(self):
        """Test starting patrol."""
        planner = MissionPlanner()
        manager = AutonomousBehaviorManager(planner)
        manager._running = True

        route = PatrolRoute(
            name="test",
            waypoints=[Waypoint(x=0, y=0)],
            interval_minutes=1,
        )

        behavior_id = await manager.start_patrol(route, "robot1")

        assert behavior_id.startswith("patrol_test")
        assert behavior_id in manager.active_behaviors
        assert manager.active_behaviors[behavior_id]["type"] == "patrol"

    @pytest.mark.asyncio
    async def test_start_exploration(self):
        """Test starting exploration."""
        planner = MissionPlanner()
        manager = AutonomousBehaviorManager(planner)

        area = ExplorationArea(
            bounds={"min_x": 0, "max_x": 5, "min_y": 0, "max_y": 5},
        )

        behavior_id = await manager.start_exploration(area, "robot1")

        assert behavior_id.startswith("explore_")
        assert behavior_id in manager.active_behaviors
        assert manager.active_behaviors[behavior_id]["type"] == "exploration"

    def test_stop_behavior(self):
        """Test stopping behavior."""
        planner = MissionPlanner()
        manager = AutonomousBehaviorManager(planner)
        manager.active_behaviors["test"] = {
            "type": "patrol",
            "status": BehaviorStatus.EXECUTING,
        }

        result = manager.stop_behavior("test")

        assert result is True
        assert manager.active_behaviors["test"]["status"] == BehaviorStatus.COMPLETED

    def test_stop_nonexistent_behavior(self):
        """Test stopping non-existent behavior."""
        planner = MissionPlanner()
        manager = AutonomousBehaviorManager(planner)
        result = manager.stop_behavior("nonexistent")
        assert result is False

    def test_get_behavior_status(self):
        """Test getting behavior status."""
        planner = MissionPlanner()
        manager = AutonomousBehaviorManager(planner)
        from datetime import datetime

        manager.active_behaviors["test"] = {
            "type": "patrol",
            "status": BehaviorStatus.EXECUTING,
            "started_at": datetime.now(),
            "iterations": 2,
            "robot_id": "robot1",
        }

        status = manager.get_behavior_status("test")

        assert status["behavior_id"] == "test"
        assert status["type"] == "patrol"
        assert status["iterations"] == 2

    def test_get_nonexistent_behavior_status(self):
        """Test getting non-existent behavior status."""
        planner = MissionPlanner()
        manager = AutonomousBehaviorManager(planner)
        status = manager.get_behavior_status("nonexistent")
        assert status is None


class TestConvenienceFunctions:
    """Test convenience functions."""

    def test_explore_autonomously(self):
        """Test explore function."""
        planner = MissionPlanner()
        manager = AutonomousBehaviorManager(planner)

        behavior_id = explore_autonomously(manager, {"min_x": 0, "max_x": 10}, "robot1")

        assert behavior_id.startswith("explore_")

    def test_patrol_route(self):
        """Test patrol function."""
        planner = MissionPlanner()
        manager = AutonomousBehaviorManager(planner)

        behavior_id = patrol_route(
            manager,
            [{"x": 0, "y": 0}, {"x": 10, "y": 10}],
            interval_minutes=30,
        )

        assert behavior_id.startswith("patrol_")
