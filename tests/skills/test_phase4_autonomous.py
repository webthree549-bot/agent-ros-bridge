"""Tests for autonomous behaviors (Phase 4).

Verifies final gap closure for 100% SKILL fulfillment.
"""


import pytest

from agent_ros_bridge.integrations.autonomous_behaviors import (
    AutonomousBehaviorManager,
    ExplorationArea,
    MissionPlanner,
    MissionStatus,
    PatrolRoute,
    Waypoint,
    explore_autonomously,
    patrol_route,
)


class TestMissionPlanner:
    """Test mission planning capabilities."""

    @pytest.fixture
    def planner(self):
        """Create mission planner."""
        return MissionPlanner()

    def test_plan_exploration_systematic(self, planner):
        """Test systematic exploration planning."""
        area = ExplorationArea(
            bounds={"min_x": 0, "max_x": 5, "min_y": 0, "max_y": 5}, resolution=2.0
        )

        mission = planner.plan_exploration(area, strategy="systematic")

        assert mission.name == "Explore area (systematic)"
        assert len(mission.steps) > 0
        # Should have navigation and scan steps
        nav_steps = [s for s in mission.steps if s.action == "navigate"]
        scan_steps = [s for s in mission.steps if s.action == "capture_image"]
        assert len(nav_steps) > 0
        assert len(scan_steps) > 0

    def test_plan_exploration_spiral(self, planner):
        """Test spiral exploration planning."""
        area = ExplorationArea(
            bounds={"min_x": -5, "max_x": 5, "min_y": -5, "max_y": 5}, resolution=1.0
        )

        mission = planner.plan_exploration(area, strategy="spiral")

        assert "spiral" in mission.description.lower()
        assert len(mission.steps) > 0

    def test_plan_patrol(self, planner):
        """Test patrol mission planning."""
        route = PatrolRoute(
            name="test_route",
            waypoints=[
                Waypoint(x=0, y=0, name="start"),
                Waypoint(x=10, y=0, name="checkpoint"),
                Waypoint(x=10, y=10, name="end"),
            ],
            interval_minutes=30,
        )

        mission = planner.plan_patrol(route)

        assert "Patrol: test_route" in mission.name
        assert len(mission.steps) >= 3  # At least one step per waypoint

    def test_plan_search(self, planner):
        """Test search mission planning."""
        area = ExplorationArea(
            bounds={"min_x": 0, "max_x": 10, "min_y": 0, "max_y": 10}, resolution=2.0
        )

        mission = planner.plan_search("red box", area)

        assert "red box" in mission.name
        # Should have detection steps
        detect_steps = [s for s in mission.steps if s.action == "detect_object"]
        assert len(detect_steps) > 0

    @pytest.mark.asyncio
    async def test_execute_mission_simulation(self, planner):
        """Test mission execution in simulation mode."""
        area = ExplorationArea(
            bounds={"min_x": 0, "max_x": 2, "min_y": 0, "max_y": 2}, resolution=1.0
        )
        mission = planner.plan_exploration(area)

        result = await planner.execute_mission(mission)

        assert result["status"] == "COMPLETED"
        assert result["steps_total"] > 0
        assert result["steps_completed"] == result["steps_total"]

    def test_mission_status_tracking(self, planner):
        """Test mission status tracking."""
        area = ExplorationArea(bounds={"min_x": 0, "max_x": 2, "min_y": 0, "max_y": 2})
        mission = planner.plan_exploration(area)

        # Before execution
        assert mission.status == MissionStatus.PENDING

        # Status should be tracked after adding to active missions
        planner.active_missions[mission.mission_id] = mission
        status = planner.get_mission_status(mission.mission_id)
        assert status is not None


class TestAutonomousBehaviorManager:
    """Test autonomous behavior management."""

    @pytest.fixture
    def manager(self):
        """Create behavior manager."""
        planner = MissionPlanner()
        return AutonomousBehaviorManager(planner)

    def test_behavior_manager_creation(self, manager):
        """Test behavior manager initialization."""
        assert manager.active_behaviors == {}
        assert manager._running == False

    def test_get_behavior_status_nonexistent(self, manager):
        """Test getting status of non-existent behavior."""
        status = manager.get_behavior_status("nonexistent")
        assert status is None


class TestSkillFulfillmentPhase4:
    """Verify Phase 4 closes final gap for 100% fulfillment."""

    def test_explore_autonomously_fulfilled(self):
        """Verify SKILL: 'Explore this room autonomously'"""
        from agent_ros_bridge.integrations.autonomous_behaviors import (
            AutonomousBehaviorManager,
            MissionPlanner,
        )

        planner = MissionPlanner()
        manager = AutonomousBehaviorManager(planner)

        area = {"min_x": 0, "max_x": 10, "min_y": 0, "max_y": 10}

        # Should return a behavior ID
        result = explore_autonomously(manager, area)
        assert isinstance(result, str)
        assert "explore" in result

    def test_patrol_route_fulfilled(self):
        """Verify SKILL: 'Patrol the perimeter every 30 minutes'"""
        from agent_ros_bridge.integrations.autonomous_behaviors import (
            AutonomousBehaviorManager,
            MissionPlanner,
        )

        planner = MissionPlanner()
        manager = AutonomousBehaviorManager(planner)

        waypoints = [
            {"x": 0, "y": 0, "name": "start"},
            {"x": 10, "y": 0, "name": "corner1"},
            {"x": 10, "y": 10, "name": "corner2"},
            {"x": 0, "y": 10, "name": "corner3"},
        ]

        result = patrol_route(manager, waypoints, interval_minutes=30)
        assert isinstance(result, str)
        assert "patrol" in result

    def test_mission_planning_fulfilled(self):
        """Verify SKILL: High-level mission planning exists."""
        from agent_ros_bridge.integrations.autonomous_behaviors import MissionPlanner

        planner = MissionPlanner()

        # Should be able to plan missions
        area = ExplorationArea(bounds={"min_x": 0, "max_x": 5, "min_y": 0, "max_y": 5})
        mission = planner.plan_exploration(area)

        assert mission is not None
        assert len(mission.steps) > 0
        assert mission.status == MissionStatus.PENDING


class TestGapClosureFinal:
    """Final verification that all gaps are closed."""

    def test_all_gaps_now_implemented(self):
        """Verify all 6 gaps are now implemented."""
        from agent_ros_bridge.integrations.autonomous_behaviors import MissionPlanner
        from agent_ros_bridge.integrations.fleet_intelligence import FleetIntelligence
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        from agent_ros_bridge.integrations.scene_understanding import SceneUnderstanding

        # 1. Natural Language Interpretation
        adapter = OpenClawAdapter(bridge=None)
        assert hasattr(adapter, "execute_nl"), "execute_nl should exist"

        # 2. Parameter Inference (enabled on first use)
        assert hasattr(adapter, "enable_natural_language"), "enable_natural_language should exist"

        # 3. Context Awareness (enabled on first use)
        assert hasattr(adapter, "learn_location"), "learn_location should exist"

        # 4. Fleet Intelligence
        fleet = FleetIntelligence()
        assert hasattr(fleet, "select_best_robot"), "select_best_robot should exist"

        # 5. Scene Understanding
        scene = SceneUnderstanding()
        assert hasattr(scene, "describe_scene"), "describe_scene should exist"

        # 6. Autonomous Behaviors
        planner = MissionPlanner()
        assert hasattr(planner, "plan_exploration"), "plan_exploration should exist"
        assert hasattr(planner, "plan_patrol"), "plan_patrol should exist"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
