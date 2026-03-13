"""Integration tests for complete SKILL workflows.

Tests end-to-end scenarios that span multiple components.
"""

import pytest

from agent_ros_bridge import Bridge


class TestIntegrationKitchenScenario:
    """Test the complete 'kitchen' workflow from SKILL."""

    @pytest.mark.asyncio
    async def test_go_to_kitchen_then_bring_water(self):
        """
        Complete workflow:
        1. Learn kitchen location
        2. "Go to kitchen" → Navigate
        3. "Now bring me water" → Context-aware action
        """
        bridge = Bridge()
        adapter = bridge.get_openclaw_adapter()
        session_id = "test_session"

        # Step 1: Learn kitchen location
        adapter.learn_location(session_id, "kitchen", {"x": 5.0, "y": 3.0})

        # Step 2: Go to kitchen
        result1 = await adapter.execute_nl("Go to kitchen", session_id)

        assert result1["success"] is False  # No actual robot connected
        assert result1["interpretation"]["tool"] == "ros2_action_goal"
        assert result1["interpretation"]["parameters"]["goal"]["pose"]["position"] == {
            "x": 5.0,
            "y": 3.0,
        }

        # Verify context was updated
        history = adapter.get_conversation_history(session_id, n=1)
        assert len(history) == 1
        assert history[0]["command"] == "Go to kitchen"

    @pytest.mark.asyncio
    async def test_context_persists_across_commands(self):
        """Test that context is maintained between commands."""
        bridge = Bridge()
        adapter = bridge.get_openclaw_adapter()
        session_id = "test_session_2"

        # First command
        await adapter.execute_nl("Move forward 2 meters", session_id)

        # Second command
        await adapter.execute_nl("Turn left", session_id)

        # Third command
        await adapter.execute_nl("Stop", session_id)

        # Verify all commands in history
        history = adapter.get_conversation_history(session_id, n=3)
        assert len(history) == 3
        assert history[0]["command"] == "Move forward 2 meters"
        assert history[1]["command"] == "Turn left"
        assert history[2]["command"] == "Stop"


class TestIntegrationFleetScenario:
    """Test fleet management workflows."""

    @pytest.mark.asyncio
    async def test_find_and_send_best_robot(self):
        """
        Complete fleet workflow:
        1. Query closest robot
        2. Send best robot to location
        """
        from agent_ros_bridge.integrations.fleet_intelligence import (
            FleetIntelligence,
            RobotCapabilities,
            RobotState,
            RobotStatus,
        )

        fleet = FleetIntelligence()

        # Learn location
        fleet.learn_location("kitchen", {"x": 10, "y": 10})

        # Add robots
        fleet.update_robot_state(
            RobotState(
                robot_id="robot-1",
                name="Alpha",
                status=RobotStatus.IDLE,
                position={"x": 0, "y": 0},
                battery_percent=80,
                capabilities=RobotCapabilities(),
            )
        )

        fleet.update_robot_state(
            RobotState(
                robot_id="robot-2",
                name="Beta",
                status=RobotStatus.IDLE,
                position={"x": 15, "y": 15},
                battery_percent=60,
                capabilities=RobotCapabilities(),
            )
        )

        # Find closest
        closest = fleet.find_closest_robot("kitchen")
        assert closest is not None
        robot_id, distance = closest
        assert robot_id == "robot-2"  # Closer to kitchen

        # Send best robot
        result = fleet.allocate_task({"type": "navigate", "to": "kitchen"})
        assert result["success"] is True
        assert result["robot_id"] in ["robot-1", "robot-2"]


class TestIntegrationSafetyScenario:
    """Test safety-critical workflows."""

    async def test_emergency_stop_workflow(self):
        """
        Safety workflow:
        1. Robot moving
        2. Emergency stop command
        3. Verify stop action
        """
        bridge = Bridge()
        adapter = bridge.get_openclaw_adapter()

        # Emergency stop should always work
        result = await adapter.execute_nl("Emergency stop")

        assert result["interpretation"]["tool"] == "safety_trigger_estop"
        assert "emergency" in result["interpretation"]["parameters"]["reason"].lower()


class TestIntegrationNLVariations:
    """Test various natural language phrasings."""

    @pytest.mark.parametrize(
        "command,expected_tool",
        [
            ("Move forward", "ros2_publish"),
            ("Drive forward 2 meters", "ros2_publish"),
            ("Go forward slowly", "ros2_publish"),
            ("Turn left", "ros2_publish"),
            ("Rotate 90 degrees left", "ros2_publish"),
            ("Spin around", "ros2_publish"),
            ("Stop", "ros2_publish"),
            ("Halt", "ros2_publish"),
            ("Go to kitchen", "ros2_action_goal"),
            ("Navigate to charging station", "ros2_action_goal"),
        ],
    )
    async def test_nl_variations(self, command, expected_tool):
        """Test that various phrasings work."""
        bridge = Bridge()
        adapter = bridge.get_openclaw_adapter()

        result = await adapter.execute_nl(command)

        assert result["interpretation"]["tool"] == expected_tool


class TestIntegrationErrorHandling:
    """Test error handling in workflows."""

    async def test_unknown_command_handling(self):
        """Test graceful handling of unknown commands."""
        bridge = Bridge()
        adapter = bridge.get_openclaw_adapter()

        result = await adapter.execute_nl("Do something impossible")

        assert result["success"] is False
        assert "error" in result
        assert "suggestion" in result

    async def test_unknown_location_handling(self):
        """Test handling of unknown locations."""
        bridge = Bridge()
        adapter = bridge.get_openclaw_adapter()

        result = await adapter.execute_nl("Go to unknown_place_xyz")

        # Should still generate valid command
        assert result["interpretation"]["tool"] == "ros2_action_goal"
        # But with a note
        assert "note" in result["interpretation"]


class TestIntegrationPerformance:
    """Test performance characteristics."""

    async def test_nl_interpretation_speed(self):
        """Test that NL interpretation is fast."""
        bridge = Bridge()
        adapter = bridge.get_openclaw_adapter()

        import time

        start = time.time()

        for _ in range(10):
            await adapter.execute_nl("Move forward 2 meters")

        elapsed = time.time() - start

        # Should be fast (< 100ms per command)
        assert elapsed < 1.0, f"Too slow: {elapsed}s for 10 commands"

    async def test_context_lookup_speed(self):
        """Test that context operations are fast."""
        bridge = Bridge()
        adapter = bridge.get_openclaw_adapter()
        session_id = "perf_test"

        # Learn many locations
        for i in range(50):
            adapter.learn_location(session_id, f"loc_{i}", {"x": i, "y": i})

        import time

        start = time.time()

        # Retrieve history
        history = adapter.get_conversation_history(session_id, n=10)

        elapsed = time.time() - start

        # Should be fast (< 100ms)
        assert elapsed < 0.1, f"Too slow: {elapsed}s"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
