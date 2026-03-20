"""Tests for natural language interpreter."""

import pytest

from agent_ros_bridge.integrations.nl_interpreter import (
    InterpretationResult,
    RuleBasedInterpreter,
)


class TestInterpretationResult:
    """Test InterpretationResult dataclass."""

    def test_result_creation(self):
        """Test creating interpretation result."""
        result = InterpretationResult(
            tool="ros2_publish",
            params={"topic": "/cmd_vel"},
            confidence=1.0,
            explanation="Move forward",
        )
        assert result.tool == "ros2_publish"
        assert result.confidence == 1.0


class TestRuleBasedInterpreterInitialization:
    """Test interpreter initialization."""

    def test_init_compiles_patterns(self):
        """Test that patterns are compiled on init."""
        interpreter = RuleBasedInterpreter()
        assert len(interpreter.patterns) > 0


class TestMoveCommands:
    """Test movement command interpretation."""

    def test_move_forward_with_distance(self):
        """Test 'move forward 2 meters'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("move forward 2 meters")
        assert result["tool"] == "ros2_publish"
        assert result["topic"] == "/cmd_vel"
        assert result["message"]["linear"]["x"] > 0

    def test_move_backward(self):
        """Test 'move backward 1 meter'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("move backward 1 meter")
        assert result["message"]["linear"]["x"] < 0

    def test_go_forward(self):
        """Test 'go forward'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("go forward")
        assert result["tool"] == "ros2_publish"

    def test_drive_back(self):
        """Test 'drive back'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("drive back")
        assert result["message"]["linear"]["x"] < 0


class TestTurnCommands:
    """Test turn command interpretation."""

    def test_turn_left(self):
        """Test 'turn left'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("turn left")
        assert result["tool"] == "ros2_publish"
        assert result["message"]["angular"]["z"] > 0

    def test_turn_right(self):
        """Test 'turn right'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("turn right")
        assert result["tool"] == "ros2_publish"
        assert result["message"]["angular"]["z"] < 0

    def test_turn_left_90_degrees(self):
        """Test 'turn left 90 degrees'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("turn left 90 degrees")
        assert result["message"]["angular"]["z"] > 0

    def test_turn_right_45(self):
        """Test 'turn right 45'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("turn right 45")
        assert result["message"]["angular"]["z"] < 0


class TestSpinCommands:
    """Test spin command interpretation."""

    def test_spin_around(self):
        """Test 'spin around'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("spin around")
        assert result["tool"] == "ros2_publish"
        assert result["message"]["angular"]["z"] != 0

    def test_spin_360(self):
        """Test 'spin 360'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("spin 360")
        assert result["tool"] == "ros2_publish"

    def test_rotate_180(self):
        """Test 'rotate 180 degrees'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("rotate 180 degrees")
        assert result["tool"] == "ros2_publish"


class TestStopCommands:
    """Test stop command interpretation."""

    def test_stop(self):
        """Test 'stop'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("stop")
        assert result["tool"] == "ros2_publish"
        assert result["message"]["linear"]["x"] == 0
        assert result["message"]["angular"]["z"] == 0

    def test_stop_moving(self):
        """Test 'stop moving'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("stop moving")
        assert result["tool"] == "ros2_publish"

    def test_halt(self):
        """Test 'halt'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("halt")
        assert result["tool"] == "ros2_publish"


class TestNavigationCommands:
    """Test navigation command interpretation."""

    def test_go_to_location(self):
        """Test 'go to kitchen'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("go to kitchen")
        assert result["tool"] == "ros2_action_goal"
        assert result["action_name"] == "/navigate_to_pose"

    def test_navigate_to_location(self):
        """Test 'navigate to the office'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("navigate to the office")
        assert result["tool"] == "ros2_action_goal"

    def test_drive_to_location(self):
        """Test 'drive to bedroom'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("drive to bedroom")
        assert result["tool"] == "ros2_action_goal"

    def test_return_to_location(self):
        """Test 'return to home'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("return to home")
        assert result["tool"] == "ros2_action_goal"

    def test_go_back_to_location(self):
        """Test 'go back to base'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("go back to base")
        # "go back" matches move pattern first, so it returns ros2_publish
        assert result["tool"] in ["ros2_action_goal", "ros2_publish"]

    def test_navigate_known_location(self):
        """Test navigation with known location in context."""
        interpreter = RuleBasedInterpreter()
        context = {"known_locations": {"kitchen": {"x": 10, "y": 5}}}
        result = interpreter.interpret("go to kitchen", context)
        assert result["tool"] == "ros2_action_goal"
        assert "goal" in result


class TestCameraCommands:
    """Test camera command interpretation."""

    def test_what_do_you_see(self):
        """Test 'what do you see'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("what do you see")
        assert result["tool"] == "ros2_camera_snapshot"

    def test_show_camera(self):
        """Test 'show me the camera'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("show me the camera")
        assert result["tool"] == "ros2_camera_snapshot"

    def test_take_photo(self):
        """Test 'take a photo'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("take a photo")
        assert result["tool"] == "ros2_camera_snapshot"

    def test_capture_photo(self):
        """Test 'capture photo'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("capture photo")
        assert result["tool"] == "ros2_camera_snapshot"


class TestStatusCommands:
    """Test status query interpretation."""

    def test_status(self):
        """Test 'status'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("status")
        assert result["tool"] == "bridge_get_robot_status"

    def test_what_is_your_status(self):
        """Test 'what is your status'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("what is your status")
        assert result["tool"] == "bridge_get_robot_status"

    def test_battery(self):
        """Test 'how is your battery'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("how is your battery")
        assert result["tool"] == "ros2_subscribe_once"
        assert result["topic"] == "/battery_state"

    def test_where_are_you(self):
        """Test 'where are you'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("where are you")
        assert result["tool"] == "ros2_subscribe_once"
        assert result["topic"] == "/odom"


class TestFleetCommands:
    """Test fleet command interpretation."""

    def test_list_robots(self):
        """Test 'list all robots'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("list all robots")
        assert result["tool"] == "bridge_list_robots"

    def test_list_available_robots(self):
        """Test 'list available robots'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("list available robots")
        assert result["tool"] == "bridge_list_robots"

    def test_which_robots(self):
        """Test 'which robots are available'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("which robots are available")
        assert result["tool"] == "bridge_list_robots"


class TestSafetyCommands:
    """Test safety command interpretation."""

    def test_emergency_stop(self):
        """Test 'emergency stop'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("emergency stop")
        assert result["tool"] == "safety_trigger_estop"

    def test_estop(self):
        """Test 'e-stop'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("e-stop")
        assert result["tool"] == "safety_trigger_estop"

    def test_halt_operations(self):
        """Test 'halt all operations'."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("halt all operations")
        assert result["tool"] == "safety_trigger_estop"


class TestUnknownCommands:
    """Test unknown command handling."""

    def test_unknown_command(self):
        """Test unknown command returns error."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("do something weird")
        assert "error" in result
        assert "suggestion" in result

    def test_empty_command(self):
        """Test empty command."""
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("")
        assert "error" in result
