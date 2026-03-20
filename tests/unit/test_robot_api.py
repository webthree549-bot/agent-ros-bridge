"""Tests for Robot API."""

from unittest.mock import MagicMock, Mock, patch

import pytest

from agent_ros_bridge.robot_api import (
    ManipulationGoal,
    NavigationGoal,
    RobotCommandResult,
    RobotController,
)


class TestNavigationGoal:
    """Test NavigationGoal dataclass."""

    def test_goal_creation(self):
        """Test creating navigation goal."""
        goal = NavigationGoal(x=1.0, y=2.0, theta=0.5)
        assert goal.x == 1.0
        assert goal.y == 2.0
        assert goal.theta == 0.5
        assert goal.frame_id == "map"

    def test_goal_defaults(self):
        """Test default values."""
        goal = NavigationGoal(x=0, y=0)
        assert goal.theta == 0.0
        assert goal.timeout_sec == 60.0


class TestManipulationGoal:
    """Test ManipulationGoal dataclass."""

    def test_goal_creation(self):
        """Test creating manipulation goal."""
        goal = ManipulationGoal(object_name="cup", action="pick")
        assert goal.object_name == "cup"
        assert goal.action == "pick"
        assert goal.location is None

    def test_goal_with_location(self):
        """Test goal with location."""
        goal = ManipulationGoal(object_name="cup", action="place", location="table")
        assert goal.location == "table"


class TestRobotCommandResult:
    """Test RobotCommandResult dataclass."""

    def test_result_creation(self):
        """Test creating result."""
        result = RobotCommandResult(success=True, execution_time=5.0)
        assert result.success is True
        assert result.execution_time == 5.0
        assert result.error_message is None

    def test_result_with_error(self):
        """Test result with error."""
        result = RobotCommandResult(
            success=False, execution_time=0.0, error_message="Connection failed"
        )
        assert result.success is False
        assert result.error_message == "Connection failed"


class TestRobotControllerInitialization:
    """Test robot controller initialization."""

    def test_init_default(self):
        """Test default initialization."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            assert robot.ros_master == "localhost:11311"
            assert robot.robot_name == "robot_01"
            assert robot.safety_enabled is True

    def test_init_custom(self):
        """Test custom initialization."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController(
                ros_master="192.168.1.100:11311", robot_name="my_robot", safety_enabled=False
            )
            assert robot.ros_master == "192.168.1.100:11311"
            assert robot.robot_name == "my_robot"
            assert robot.safety_enabled is False


class TestRobotControllerConnection:
    """Test robot connection."""

    def test_is_connected_false(self):
        """Test is_connected returns False when not connected."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = False
            assert robot.is_connected() is False

    def test_is_connected_true(self):
        """Test is_connected returns True when connected."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = True
            assert robot.is_connected() is True


class TestRobotControllerState:
    """Test getting robot state."""

    def test_get_state_disconnected(self):
        """Test getting state when disconnected."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = False
            state = robot.get_state()
            assert state["connected"] is False
            assert state["status"] == "disconnected"

    def test_get_state_connected(self):
        """Test getting state when connected."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = True
            robot._battery_level = 85.0
            state = robot.get_state()
            assert state["connected"] is True
            assert state["battery_level"] == 85.0
            assert state["status"] == "idle"


class TestRobotControllerNavigation:
    """Test navigation."""

    def test_navigate_to_not_connected(self):
        """Test navigation when not connected."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = False
            goal = NavigationGoal(x=1.0, y=2.0)
            result = robot.navigate_to(goal)
            assert result.success is False
            assert "Not connected" in result.error_message


class TestRobotControllerManipulation:
    """Test manipulation."""

    def test_pick_up_not_connected(self):
        """Test pick_up when not connected."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = False
            result = robot.pick_up("cup")
            assert result.success is False
            assert "Not connected" in result.error_message

    def test_place_at_not_connected(self):
        """Test place_at when not connected."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = False
            result = robot.place_at("table")
            assert result.success is False


class TestRobotControllerStop:
    """Test emergency stop."""

    def test_stop_not_connected(self):
        """Test stop when not connected."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = False
            result = robot.stop()
            assert result is False

    def test_stop_connected(self):
        """Test stop when connected."""
        with patch.object(RobotController, "_connect"):
            with patch.dict("sys.modules", {"geometry_msgs.msg": MagicMock()}):
                robot = RobotController()
                robot._connected = True
                robot._cmd_vel_pub = Mock()
                result = robot.stop()
                assert result is True
                robot._cmd_vel_pub.publish.assert_called_once()


class TestRobotControllerSay:
    """Test robot speech."""

    def test_say(self):
        """Test say method."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            result = robot.say("Hello world")
            assert result is True


class TestRobotControllerContextManager:
    """Test context manager."""

    def test_context_manager(self):
        """Test using robot as context manager."""
        with patch.object(RobotController, "_connect"):
            with RobotController() as robot:
                assert isinstance(robot, RobotController)

    def test_context_manager_exit(self):
        """Test context manager exit."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._disconnect = Mock()
            robot.__exit__(None, None, None)
            robot._disconnect.assert_called_once()
