"""Tests for fleet intelligence module."""

import pytest

from agent_ros_bridge.integrations.fleet_intelligence import (
    FleetIntelligence,
    RobotCapabilities,
    RobotState,
    RobotStatus,
    find_closest_robot_to_location,
    find_robot_with_most_battery,
    send_best_robot,
)


class TestRobotState:
    """Test RobotState dataclass."""

    def test_robot_state_creation(self):
        """Test creating a robot state."""
        caps = RobotCapabilities(can_navigate=True, can_manipulate=False, max_payload=5.0)
        state = RobotState(
            robot_id="r1",
            name="Robot 1",
            status=RobotStatus.IDLE,
            position={"x": 0, "y": 0, "z": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        assert state.robot_id == "r1"
        assert state.status == RobotStatus.IDLE


class TestFleetIntelligenceInitialization:
    """Test FleetIntelligence initialization."""

    def test_init_creates_empty_state(self):
        """Test that initialization creates empty state."""
        fleet = FleetIntelligence()
        assert fleet.robot_states == {}
        assert fleet.known_locations == {}


class TestUpdateRobotState:
    """Test updating robot state."""

    def test_update_robot_state(self):
        """Test updating robot state."""
        fleet = FleetIntelligence()
        caps = RobotCapabilities()
        state = RobotState(
            robot_id="r1",
            name="Robot 1",
            status=RobotStatus.IDLE,
            position={"x": 1.0, "y": 2.0},
            battery_percent=80.0,
            capabilities=caps,
        )
        fleet.update_robot_state(state)
        assert "r1" in fleet.robot_states
        assert fleet.robot_states["r1"].battery_percent == 80.0

    def test_update_overwrites_existing(self):
        """Test that update overwrites existing state."""
        fleet = FleetIntelligence()
        caps = RobotCapabilities()
        state1 = RobotState(
            robot_id="r1",
            name="Robot 1",
            status=RobotStatus.IDLE,
            position={"x": 0, "y": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        state2 = RobotState(
            robot_id="r1",
            name="Robot 1",
            status=RobotStatus.BUSY,
            position={"x": 1, "y": 1},
            battery_percent=50.0,
            capabilities=caps,
        )
        fleet.update_robot_state(state1)
        fleet.update_robot_state(state2)
        assert fleet.robot_states["r1"].battery_percent == 50.0
        assert fleet.robot_states["r1"].status == RobotStatus.BUSY


class TestLearnLocation:
    """Test learning locations."""

    def test_learn_location(self):
        """Test learning a location."""
        fleet = FleetIntelligence()
        fleet.learn_location("kitchen", {"x": 10.0, "y": 5.0, "z": 0.0})
        assert "kitchen" in fleet.known_locations
        assert fleet.known_locations["kitchen"]["x"] == 10.0

    def test_learn_location_case_insensitive(self):
        """Test that locations are stored lowercase."""
        fleet = FleetIntelligence()
        fleet.learn_location("KITCHEN", {"x": 10.0, "y": 5.0})
        assert "kitchen" in fleet.known_locations


class TestCalculateDistance:
    """Test distance calculations."""

    def test_calculate_distance_same_point(self):
        """Test distance between same point is 0."""
        fleet = FleetIntelligence()
        pos = {"x": 0, "y": 0}
        assert fleet.calculate_distance(pos, pos) == 0.0

    def test_calculate_distance_simple(self):
        """Test simple distance calculation."""
        fleet = FleetIntelligence()
        pos1 = {"x": 0, "y": 0}
        pos2 = {"x": 3, "y": 4}
        assert fleet.calculate_distance(pos1, pos2) == 5.0

    def test_calculate_distance_with_z(self):
        """Test distance ignores z coordinate."""
        fleet = FleetIntelligence()
        pos1 = {"x": 0, "y": 0, "z": 100}
        pos2 = {"x": 3, "y": 4, "z": 0}
        assert fleet.calculate_distance(pos1, pos2) == 5.0


class TestGetDistanceToLocation:
    """Test getting distance to named location."""

    def test_distance_to_known_location(self):
        """Test distance to known location."""
        fleet = FleetIntelligence()
        caps = RobotCapabilities()
        fleet.learn_location("kitchen", {"x": 10.0, "y": 0})
        state = RobotState(
            robot_id="r1",
            name="Robot 1",
            status=RobotStatus.IDLE,
            position={"x": 0, "y": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        fleet.update_robot_state(state)
        distance = fleet.get_distance_to_location("r1", "kitchen")
        assert distance == 10.0

    def test_distance_unknown_robot(self):
        """Test distance returns None for unknown robot."""
        fleet = FleetIntelligence()
        fleet.learn_location("kitchen", {"x": 10.0, "y": 0})
        assert fleet.get_distance_to_location("unknown", "kitchen") is None

    def test_distance_unknown_location(self):
        """Test distance returns None for unknown location."""
        fleet = FleetIntelligence()
        caps = RobotCapabilities()
        state = RobotState(
            robot_id="r1",
            name="Robot 1",
            status=RobotStatus.IDLE,
            position={"x": 0, "y": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        fleet.update_robot_state(state)
        assert fleet.get_distance_to_location("r1", "unknown") is None


class TestSelectBestRobot:
    """Test robot selection."""

    def test_select_best_no_robots(self):
        """Test selection with no robots returns None."""
        fleet = FleetIntelligence()
        assert fleet.select_best_robot({"type": "navigate"}) is None

    def test_select_best_only_idle(self):
        """Test only idle robots are considered."""
        fleet = FleetIntelligence()
        caps = RobotCapabilities()
        idle = RobotState(
            robot_id="idle_robot",
            name="Idle",
            status=RobotStatus.IDLE,
            position={"x": 0, "y": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        busy = RobotState(
            robot_id="busy_robot",
            name="Busy",
            status=RobotStatus.BUSY,
            position={"x": 0, "y": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        fleet.update_robot_state(idle)
        fleet.update_robot_state(busy)
        result = fleet.select_best_robot({"type": "navigate"})
        assert result == "idle_robot"

    def test_select_best_by_battery(self):
        """Test selection prefers higher battery."""
        fleet = FleetIntelligence()
        caps = RobotCapabilities()
        low_battery = RobotState(
            robot_id="low",
            name="Low Battery",
            status=RobotStatus.IDLE,
            position={"x": 0, "y": 0},
            battery_percent=20.0,
            capabilities=caps,
        )
        high_battery = RobotState(
            robot_id="high",
            name="High Battery",
            status=RobotStatus.IDLE,
            position={"x": 0, "y": 0},
            battery_percent=90.0,
            capabilities=caps,
        )
        fleet.update_robot_state(low_battery)
        fleet.update_robot_state(high_battery)
        result = fleet.select_best_robot({"type": "navigate"}, criteria=["battery"])
        assert result == "high"


class TestFindClosestRobot:
    """Test finding closest robot."""

    def test_find_closest_robot(self):
        """Test finding closest robot to location."""
        fleet = FleetIntelligence()
        caps = RobotCapabilities()
        fleet.learn_location("kitchen", {"x": 10.0, "y": 0})
        close = RobotState(
            robot_id="close",
            name="Close Robot",
            status=RobotStatus.IDLE,
            position={"x": 8.0, "y": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        far = RobotState(
            robot_id="far",
            name="Far Robot",
            status=RobotStatus.IDLE,
            position={"x": 0, "y": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        fleet.update_robot_state(close)
        fleet.update_robot_state(far)
        result = fleet.find_closest_robot("kitchen")
        assert result[0] == "close"
        assert result[1] == 2.0

    def test_find_closest_no_robots(self):
        """Test returns None with no robots."""
        fleet = FleetIntelligence()
        fleet.learn_location("kitchen", {"x": 10.0, "y": 0})
        assert fleet.find_closest_robot("kitchen") is None


class TestGetRobotWithMostBattery:
    """Test finding robot with most battery."""

    def test_most_battery(self):
        """Test finding robot with highest battery."""
        fleet = FleetIntelligence()
        caps = RobotCapabilities()
        low = RobotState(
            robot_id="low",
            name="Low",
            status=RobotStatus.IDLE,
            position={"x": 0, "y": 0},
            battery_percent=20.0,
            capabilities=caps,
        )
        high = RobotState(
            robot_id="high",
            name="High",
            status=RobotStatus.IDLE,
            position={"x": 0, "y": 0},
            battery_percent=95.0,
            capabilities=caps,
        )
        fleet.update_robot_state(low)
        fleet.update_robot_state(high)
        assert fleet.get_robot_with_most_battery() == "high"

    def test_most_battery_empty(self):
        """Test returns None with no robots."""
        fleet = FleetIntelligence()
        assert fleet.get_robot_with_most_battery() is None


class TestAllocateTask:
    """Test task allocation."""

    def test_allocate_task_success(self):
        """Test successful task allocation."""
        fleet = FleetIntelligence()
        caps = RobotCapabilities()
        state = RobotState(
            robot_id="r1",
            name="Robot 1",
            status=RobotStatus.IDLE,
            position={"x": 0, "y": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        fleet.update_robot_state(state)
        result = fleet.allocate_task({"type": "navigate", "to": "kitchen"})
        assert result["success"] is True
        assert result["robot_id"] == "r1"

    def test_allocate_task_no_robots(self):
        """Test allocation fails with no robots."""
        fleet = FleetIntelligence()
        result = fleet.allocate_task({"type": "navigate"})
        assert result["success"] is False
        assert "error" in result


class TestPlanCoordination:
    """Test coordination planning."""

    def test_plan_search(self):
        """Test search coordination plan."""
        fleet = FleetIntelligence()
        caps = RobotCapabilities()
        r1 = RobotState(
            robot_id="r1",
            name="Robot 1",
            status=RobotStatus.IDLE,
            position={"x": 0, "y": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        r2 = RobotState(
            robot_id="r2",
            name="Robot 2",
            status=RobotStatus.IDLE,
            position={"x": 10, "y": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        fleet.update_robot_state(r1)
        fleet.update_robot_state(r2)
        plan = fleet.plan_coordination("search")
        assert plan["type"] == "search"
        assert len(plan["sectors"]) == 2

    def test_plan_convoy(self):
        """Test convoy coordination plan."""
        fleet = FleetIntelligence()
        caps = RobotCapabilities()
        r1 = RobotState(
            robot_id="r1",
            name="Robot 1",
            status=RobotStatus.IDLE,
            position={"x": 0, "y": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        r2 = RobotState(
            robot_id="r2",
            name="Robot 2",
            status=RobotStatus.IDLE,
            position={"x": 10, "y": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        fleet.update_robot_state(r1)
        fleet.update_robot_state(r2)
        plan = fleet.plan_coordination("convoy")
        assert plan["type"] == "convoy"
        assert plan["leader"] == "r1"
        assert "r2" in plan["followers"]

    def test_plan_convoy_insufficient_robots(self):
        """Test convoy fails with less than 2 robots."""
        fleet = FleetIntelligence()
        caps = RobotCapabilities()
        r1 = RobotState(
            robot_id="r1",
            name="Robot 1",
            status=RobotStatus.IDLE,
            position={"x": 0, "y": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        fleet.update_robot_state(r1)
        plan = fleet.plan_coordination("convoy")
        assert "error" in plan

    def test_plan_patrol(self):
        """Test patrol coordination plan."""
        fleet = FleetIntelligence()
        caps = RobotCapabilities()
        r1 = RobotState(
            robot_id="r1",
            name="Robot 1",
            status=RobotStatus.IDLE,
            position={"x": 0, "y": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        fleet.update_robot_state(r1)
        plan = fleet.plan_coordination("patrol")
        assert plan["type"] == "patrol"
        assert plan["shift_duration_minutes"] == 30

    def test_plan_unknown_type(self):
        """Test unknown coordination type returns error."""
        fleet = FleetIntelligence()
        plan = fleet.plan_coordination("unknown")
        assert "error" in plan


class TestGetFleetSummary:
    """Test fleet summary."""

    def test_empty_fleet_summary(self):
        """Test summary of empty fleet."""
        fleet = FleetIntelligence()
        summary = fleet.get_fleet_summary()
        assert summary["total_robots"] == 0
        assert summary["average_battery"] == 0
        assert summary["ready_for_task"] is False

    def test_fleet_summary(self):
        """Test summary with robots."""
        fleet = FleetIntelligence()
        caps = RobotCapabilities()
        r1 = RobotState(
            robot_id="r1",
            name="Robot 1",
            status=RobotStatus.IDLE,
            position={"x": 0, "y": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        r2 = RobotState(
            robot_id="r2",
            name="Robot 2",
            status=RobotStatus.BUSY,
            position={"x": 10, "y": 0},
            battery_percent=50.0,
            capabilities=caps,
        )
        fleet.update_robot_state(r1)
        fleet.update_robot_state(r2)
        summary = fleet.get_fleet_summary()
        assert summary["total_robots"] == 2
        assert summary["available"] == 1
        assert summary["average_battery"] == 75.0
        assert summary["ready_for_task"] is True


class TestConvenienceFunctions:
    """Test convenience functions."""

    def test_find_closest_robot_to_location(self):
        """Test convenience function."""
        fleet = FleetIntelligence()
        caps = RobotCapabilities()
        fleet.learn_location("kitchen", {"x": 10.0, "y": 0})
        state = RobotState(
            robot_id="r1",
            name="Robot 1",
            status=RobotStatus.IDLE,
            position={"x": 8.0, "y": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        fleet.update_robot_state(state)
        result = find_closest_robot_to_location(fleet, "kitchen")
        assert result == "r1"

    def test_find_robot_with_most_battery(self):
        """Test convenience function."""
        fleet = FleetIntelligence()
        caps = RobotCapabilities()
        state = RobotState(
            robot_id="r1",
            name="Robot 1",
            status=RobotStatus.IDLE,
            position={"x": 0, "y": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        fleet.update_robot_state(state)
        result = find_robot_with_most_battery(fleet)
        assert result == "r1"

    def test_send_best_robot(self):
        """Test convenience function."""
        fleet = FleetIntelligence()
        caps = RobotCapabilities()
        state = RobotState(
            robot_id="r1",
            name="Robot 1",
            status=RobotStatus.IDLE,
            position={"x": 0, "y": 0},
            battery_percent=100.0,
            capabilities=caps,
        )
        fleet.update_robot_state(state)
        result = send_best_robot(fleet, {"type": "navigate"})
        assert result["success"] is True
        assert result["robot_id"] == "r1"
