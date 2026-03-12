"""Tests for fleet intelligence and scene understanding.

Verifies Phase 3 capabilities are implemented correctly.
"""

import pytest
from agent_ros_bridge.integrations.fleet_intelligence import (
    FleetIntelligence,
    RobotState,
    RobotStatus,
    RobotCapabilities,
    find_closest_robot_to_location,
    find_robot_with_most_battery,
    send_best_robot,
)
from agent_ros_bridge.integrations.scene_understanding import (
    SceneUnderstanding,
    PerceptionBackend,
    what_do_you_see,
    describe_the_room,
    is_path_clear,
)


class TestFleetIntelligence:
    """Test fleet intelligence capabilities."""

    @pytest.fixture
    def fleet(self):
        """Create fleet with test robots."""
        fleet = FleetIntelligence()

        # Learn locations
        fleet.learn_location("kitchen", {"x": 10, "y": 10})
        fleet.learn_location("office", {"x": 20, "y": 20})
        fleet.learn_location("home", {"x": 0, "y": 0})

        # Add robots
        fleet.update_robot_state(
            RobotState(
                robot_id="robot-1",
                name="Alpha",
                status=RobotStatus.IDLE,
                position={"x": 0, "y": 0},
                battery_percent=80,
                capabilities=RobotCapabilities(can_navigate=True, max_speed=1.0),
            )
        )

        fleet.update_robot_state(
            RobotState(
                robot_id="robot-2",
                name="Beta",
                status=RobotStatus.IDLE,
                position={"x": 15, "y": 15},  # Closer to kitchen
                battery_percent=60,
                capabilities=RobotCapabilities(can_navigate=True, max_speed=1.2),
            )
        )

        fleet.update_robot_state(
            RobotState(
                robot_id="robot-3",
                name="Gamma",
                status=RobotStatus.BUSY,
                position={"x": 5, "y": 5},
                battery_percent=90,
                capabilities=RobotCapabilities(can_navigate=True, max_speed=0.8),
            )
        )

        return fleet

    def test_calculate_distance(self, fleet):
        """Test distance calculation."""
        pos1 = {"x": 0, "y": 0}
        pos2 = {"x": 3, "y": 4}

        distance = fleet.calculate_distance(pos1, pos2)
        assert distance == 5.0  # 3-4-5 triangle

    def test_get_distance_to_location(self, fleet):
        """Test distance from robot to named location."""
        # robot-1 is at (0,0), kitchen is at (10,10)
        distance = fleet.get_distance_to_location("robot-1", "kitchen")
        expected = pytest.approx(14.14, 0.01)  # sqrt(200)
        assert distance == expected

    def test_find_closest_robot(self, fleet):
        """Test finding closest robot to location."""
        result = fleet.find_closest_robot("kitchen")

        assert result is not None
        robot_id, distance = result
        # robot-2 at (15,15) is closest to kitchen at (10,10)
        assert robot_id == "robot-2"
        assert distance == pytest.approx(7.07, 0.01)

    def test_select_best_robot_distance(self, fleet):
        """Test robot selection based on distance."""
        task = {"type": "navigate", "to": "kitchen"}

        best = fleet.select_best_robot(task, criteria=["distance"])

        # robot-2 is closest to kitchen
        assert best == "robot-2"

    def test_select_best_robot_battery(self, fleet):
        """Test robot selection based on battery."""
        task = {"type": "navigate"}

        best = fleet.select_best_robot(task, criteria=["battery"])

        # robot-3 has most battery (90%) but is busy
        # robot-1 has 80% and is idle
        assert best == "robot-1"

    def test_get_robot_with_most_battery(self, fleet):
        """Test finding robot with highest battery."""
        best = fleet.get_robot_with_most_battery()

        # robot-3 has 90% (even though busy)
        assert best == "robot-3"

    def test_allocate_task(self, fleet):
        """Test task allocation."""
        task = {"type": "navigate", "to": "kitchen"}

        result = fleet.allocate_task(task)

        assert result["success"] is True
        # Should select one of the available robots (robot-1 or robot-2)
        assert result["robot_id"] in ["robot-1", "robot-2"]
        assert "confidence" in result

    def test_allocate_task_no_robots_available(self, fleet):
        """Test allocation when no robots available."""
        # Make all robots busy
        for rid in list(fleet.robot_states.keys()):
            fleet.robot_states[rid].status = RobotStatus.BUSY

        task = {"type": "navigate"}
        result = fleet.allocate_task(task)

        assert result["success"] is False
        assert "error" in result

    def test_plan_coordination_search(self, fleet):
        """Test planning parallel search."""
        plan = fleet.plan_coordination("search", robots=["robot-1", "robot-2"])

        assert plan["type"] == "search"
        assert len(plan["sectors"]) == 2
        assert plan["strategy"] == "parallel_sector_search"

    def test_plan_coordination_convoy(self, fleet):
        """Test planning convoy."""
        plan = fleet.plan_coordination("convoy", robots=["robot-1", "robot-2"])

        assert plan["type"] == "convoy"
        assert plan["leader"] == "robot-1"
        assert "robot-2" in plan["followers"]

    def test_fleet_summary(self, fleet):
        """Test fleet summary generation."""
        summary = fleet.get_fleet_summary()

        assert summary["total_robots"] == 3
        assert summary["available"] == 2  # 2 idle, 1 busy
        assert summary["ready_for_task"] is True


class TestSceneUnderstanding:
    """Test scene understanding capabilities."""

    def test_basic_mode_without_api(self):
        """Test that basic mode works without API."""
        scene = SceneUnderstanding(backend=PerceptionBackend.NONE)

        status = scene.get_perception_status()
        assert status["backend"] == "none"
        assert status["available"] is False

    @pytest.mark.asyncio
    async def test_describe_scene_basic(self):
        """Test basic scene description."""
        scene = SceneUnderstanding(backend=PerceptionBackend.NONE)

        camera_info = {"topic": "/camera/front", "timestamp": "2024-01-01"}
        # Pass None for image_data to test basic mode
        desc = await scene.describe_scene(image_data=None, camera_info=camera_info)

        assert "Camera image from /camera/front" in desc.summary
        assert desc.confidence == 0.0

    @pytest.mark.asyncio
    async def test_answer_query_what_do_you_see(self):
        """Test 'What do you see?' query."""
        scene = SceneUnderstanding(backend=PerceptionBackend.NONE)

        answer = await scene.answer_query("What do you see?")

        assert "camera feed" in answer.lower()

    @pytest.mark.asyncio
    async def test_answer_query_in_front(self):
        """Test 'What's in front?' query."""
        scene = SceneUnderstanding(backend=PerceptionBackend.NONE)

        answer = await scene.answer_query("What's in front?")

        assert "sensor data" in answer.lower() or "forward" in answer.lower()

    def test_is_path_clear(self):
        """Test path clearance check."""
        # Clear path (all distances > 0.5m)
        lidar_clear = [1.0, 2.0, 1.5, 3.0, 2.0]
        assert is_path_clear(lidar_clear, threshold=0.5) is True

        # Blocked path (something at 0.3m)
        lidar_blocked = [1.0, 0.3, 1.5, 3.0, 2.0]
        assert is_path_clear(lidar_blocked, threshold=0.5) is False

        # Empty data
        assert is_path_clear([], threshold=0.5) is True


class TestSkillFulfillmentPhase3:
    """Verify Phase 3 capabilities fulfill SKILL promises."""

    def test_closest_robot_selection_fulfilled(self):
        """Verify SKILL: 'Which robot is closest?'"""
        fleet = FleetIntelligence()
        fleet.learn_location("kitchen", {"x": 10, "y": 10})

        fleet.update_robot_state(
            RobotState(
                robot_id="r1",
                name="R1",
                status=RobotStatus.IDLE,
                position={"x": 0, "y": 0},
                battery_percent=50,
                capabilities=RobotCapabilities(),
            )
        )

        result = find_closest_robot_to_location(fleet, "kitchen")
        assert result == "r1"

    def test_best_battery_selection_fulfilled(self):
        """Verify SKILL: 'Which robot has most battery?'"""
        fleet = FleetIntelligence()

        fleet.update_robot_state(
            RobotState(
                robot_id="r1",
                name="R1",
                status=RobotStatus.IDLE,
                position={"x": 0, "y": 0},
                battery_percent=50,
                capabilities=RobotCapabilities(),
            )
        )
        fleet.update_robot_state(
            RobotState(
                robot_id="r2",
                name="R2",
                status=RobotStatus.IDLE,
                position={"x": 0, "y": 0},
                battery_percent=80,
                capabilities=RobotCapabilities(),
            )
        )

        result = find_robot_with_most_battery(fleet)
        assert result == "r2"

    def test_send_best_robot_fulfilled(self):
        """Verify SKILL: 'Send the best robot'"""
        fleet = FleetIntelligence()
        fleet.learn_location("kitchen", {"x": 10, "y": 10})

        fleet.update_robot_state(
            RobotState(
                robot_id="r1",
                name="R1",
                status=RobotStatus.IDLE,
                position={"x": 0, "y": 0},
                battery_percent=50,
                capabilities=RobotCapabilities(),
            )
        )

        result = send_best_robot(fleet, {"type": "navigate", "to": "kitchen"})

        assert result["success"] is True
        assert result["robot_id"] == "r1"

    @pytest.mark.asyncio
    async def test_scene_understanding_fulfilled(self):
        """Verify SKILL: 'What do you see?'"""
        scene = SceneUnderstanding(backend=PerceptionBackend.NONE)

        result = await what_do_you_see(scene)

        assert "camera" in result.lower()

    @pytest.mark.asyncio
    async def test_describe_room_fulfilled(self):
        """Verify SKILL: 'Describe the room'"""
        scene = SceneUnderstanding(backend=PerceptionBackend.NONE)

        result = await describe_the_room(scene)

        assert "description" in result.lower() or "camera" in result.lower()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
