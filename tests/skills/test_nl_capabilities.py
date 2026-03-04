"""Tests for natural language capabilities.

Verifies that the NL interpreter fulfills SKILL promises.
"""

import pytest
from agent_ros_bridge.integrations.nl_params import (
    infer_parameter, 
    infer_speed, 
    infer_distance,
    infer_angle,
    parse_numeric
)
from agent_ros_bridge.integrations.nl_interpreter import RuleBasedInterpreter


class TestParameterInference:
    """Test natural language parameter inference."""
    
    def test_speed_slowly(self):
        """Test 'slowly' → 0.1 m/s"""
        assert infer_speed("slowly") == 0.1
        assert infer_speed("Move slowly") == 0.1
    
    def test_speed_normal(self):
        """Test default/normal speed."""
        assert infer_speed("") == 0.5
        assert infer_speed("normal") == 0.5
    
    def test_speed_fast(self):
        """Test 'fast' → 1.0 m/s"""
        assert infer_speed("fast") == 1.0
        assert infer_speed("quickly") == 1.5
    
    def test_distance_a_bit(self):
        """Test 'a bit' → 0.5m"""
        assert infer_distance("a bit") == 0.5
        assert infer_distance("Move a bit") == 0.5
    
    def test_distance_a_lot(self):
        """Test 'a lot' → 2.0m"""
        assert infer_distance("a lot") == 2.0
    
    def test_angle_90_degrees(self):
        """Test angle extraction from '90 degrees'."""
        assert infer_angle("90 degrees") == 90.0
        assert infer_angle("Turn 90 deg") == 90.0
    
    def test_angle_sharp(self):
        """Test 'sharp' → 90°"""
        assert infer_angle("sharp") == 90
    
    def test_parse_numeric_with_units(self):
        """Test number extraction with units."""
        assert parse_numeric("2 meters") == 2.0
        assert parse_numeric("90 degrees") == 90.0
        assert parse_numeric("about 3") == 3.0


class TestNLInterpreter:
    """Test natural language interpreter."""
    
    @pytest.fixture
    def interpreter(self):
        return RuleBasedInterpreter()
    
    def test_move_forward_precise(self, interpreter):
        """Test 'Move forward 2 meters' interpretation."""
        result = interpreter.interpret("Move forward 2 meters")
        
        assert result["tool"] == "ros2_publish"
        assert result["topic"] == "/cmd_vel"
        assert result["message_type"] == "geometry_msgs/Twist"
        assert result["message"]["linear"]["x"] > 0  # Forward
        assert result["duration"] == 4.0  # 2m / 0.5m/s
        assert "Move forward" in result["explanation"]
    
    def test_move_backward(self, interpreter):
        """Test 'Move backward 1 meter' interpretation."""
        result = interpreter.interpret("Move backward 1 meter")
        
        assert result["tool"] == "ros2_publish"
        assert result["message"]["linear"]["x"] < 0  # Backward
    
    def test_turn_left_90(self, interpreter):
        """Test 'Turn left 90 degrees' interpretation."""
        result = interpreter.interpret("Turn left 90 degrees")
        
        assert result["tool"] == "ros2_publish"
        assert result["message"]["angular"]["z"] > 0  # Positive for left
        assert result["duration"] > 0
    
    def test_turn_right(self, interpreter):
        """Test 'Turn right' interpretation."""
        result = interpreter.interpret("Turn right")
        
        assert result["message"]["angular"]["z"] < 0  # Negative for right
    
    def test_stop(self, interpreter):
        """Test 'Stop' interpretation."""
        result = interpreter.interpret("Stop")
        
        assert result["tool"] == "ros2_publish"
        assert result["message"]["linear"]["x"] == 0
        assert result["message"]["angular"]["z"] == 0
    
    def test_spin_around(self, interpreter):
        """Test 'Spin around' interpretation."""
        result = interpreter.interpret("Spin around")
        
        assert result["tool"] == "ros2_publish"
        assert result["duration"] == pytest.approx(6.28, 0.01)
    
    def test_go_to_location(self, interpreter):
        """Test 'Go to kitchen' interpretation."""
        result = interpreter.interpret("Go to kitchen")
        
        assert result["tool"] == "ros2_action_goal"
        assert result["action_name"] == "/navigate_to_pose"
        assert result["location_name"] == "kitchen"
    
    def test_what_do_you_see(self, interpreter):
        """Test 'What do you see?' interpretation."""
        result = interpreter.interpret("What do you see?")
        
        assert result["tool"] == "ros2_camera_snapshot"
        assert result["topic"] == "/camera/image_raw"
    
    def test_list_robots(self, interpreter):
        """Test 'List robots' interpretation."""
        result = interpreter.interpret("List all robots")
        
        assert result["tool"] == "bridge_list_robots"
    
    def test_emergency_stop(self, interpreter):
        """Test 'Emergency stop' interpretation."""
        result = interpreter.interpret("Emergency stop")
        
        assert result["tool"] == "safety_trigger_estop"
        assert "emergency" in result["reason"].lower()
    
    def test_unknown_command(self, interpreter):
        """Test handling of unknown command."""
        result = interpreter.interpret("Do something weird")
        
        assert "error" in result
        assert "suggestion" in result


class TestNLWithAdapter:
    """Test NL integration with OpenClawAdapter."""
    
    @pytest.mark.asyncio
    async def test_adapter_execute_nl(self):
        """Test that adapter can execute NL commands."""
        from agent_ros_bridge import Bridge
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        bridge = Bridge()
        adapter = bridge.get_openclaw_adapter()
        
        # Test NL execution (will fail to execute but should interpret)
        result = await adapter.execute_nl("Move forward 2 meters")
        
        assert "command" in result
        assert "interpretation" in result
        assert result["command"] == "Move forward 2 meters"
        assert result["interpretation"]["tool"] == "ros2_publish"


class TestSkillFulfillmentNL:
    """Verify NL capabilities fulfill SKILL promises."""
    
    def test_movement_commands_fulfilled(self):
        """Verify SKILL movement commands work."""
        interpreter = RuleBasedInterpreter()
        
        commands = [
            ("Move forward", "ros2_publish"),
            ("Turn left", "ros2_publish"),
            ("Stop", "ros2_publish"),
            ("Spin around", "ros2_publish"),
        ]
        
        for cmd, expected_tool in commands:
            result = interpreter.interpret(cmd)
            assert result["tool"] == expected_tool, f"Command '{cmd}' failed"
    
    def test_sensor_queries_fulfilled(self):
        """Verify SKILL sensor queries work."""
        interpreter = RuleBasedInterpreter()
        
        queries = [
            ("What do you see?", "ros2_camera_snapshot"),
            ("Show me the camera", "ros2_camera_snapshot"),
        ]
        
        for query, expected_tool in queries:
            result = interpreter.interpret(query)
            assert result["tool"] == expected_tool, f"Query '{query}' failed"
    
    def test_navigation_commands_fulfilled(self):
        """Verify SKILL navigation commands work."""
        interpreter = RuleBasedInterpreter()
        
        result = interpreter.interpret("Go to the kitchen")
        assert result["tool"] == "ros2_action_goal"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
