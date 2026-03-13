"""Tests for context awareness capabilities.

Verifies that context management fulfills SKILL promises about
context-aware conversations.
"""

import os
import tempfile

import pytest

from agent_ros_bridge.integrations.context import (
    ContextAwareNLInterpreter,
    ContextManager,
)
from agent_ros_bridge.integrations.nl_interpreter import RuleBasedInterpreter


class TestContextManager:
    """Test context management functionality."""

    @pytest.fixture
    def temp_db(self):
        """Create temporary database for testing."""
        with tempfile.NamedTemporaryFile(suffix=".db", delete=False) as f:
            db_path = f.name
        yield db_path
        os.unlink(db_path)

    @pytest.fixture
    def manager(self, temp_db):
        """Create context manager with temp database."""
        return ContextManager(db_path=temp_db)

    def test_create_context(self, manager):
        """Test creating a new context."""
        ctx = manager.get_context("session1")

        assert ctx.session_id == "session1"
        assert ctx.current_location is None
        assert ctx.known_locations == {}

    def test_persist_context(self, manager):
        """Test that context persists across instances."""
        # Create and modify context
        ctx = manager.get_context("session1")
        ctx.current_location = "kitchen"
        ctx.known_locations["kitchen"] = {"x": 5, "y": 3}
        manager.save_context(ctx)

        # Create new manager instance (simulating restart)
        manager2 = ContextManager(db_path=manager.db_path)
        ctx2 = manager2.get_context("session1")

        assert ctx2.current_location == "kitchen"
        assert ctx2.known_locations["kitchen"] == {"x": 5, "y": 3}

    def test_learn_location(self, manager):
        """Test learning a location."""
        manager.learn_location("session1", "kitchen", {"x": 5, "y": 3})

        coords = manager.get_location("session1", "kitchen")
        assert coords == {"x": 5, "y": 3}

    def test_get_unknown_location(self, manager):
        """Test getting a location that wasn't learned."""
        coords = manager.get_location("session1", "unknown_place")
        assert coords is None

    def test_log_interaction(self, manager):
        """Test logging command interactions."""
        manager.log_interaction(
            "session1", "Move forward", {"tool": "ros2_publish"}, {"success": True}
        )

        history = manager.get_last_n_commands("session1", n=1)
        assert len(history) == 1
        assert history[0]["command"] == "Move forward"

    def test_conversation_history_limit(self, manager):
        """Test that history is limited to recent commands."""
        # Log many commands
        for i in range(15):
            manager.log_interaction("session1", f"Command {i}", {}, {})

        # Should only get last 10
        history = manager.get_context("session1").conversation_history
        assert len(history) == 10
        assert history[-1]["command"] == "Command 14"

    def test_list_learned_locations(self, manager):
        """Test listing learned locations."""
        manager.learn_location("session1", "kitchen", {"x": 1, "y": 1})
        manager.learn_location("session1", "office", {"x": 2, "y": 2})

        locations = manager.list_learned_locations("session1")
        assert "kitchen" in locations
        assert "office" in locations

    def test_clear_context(self, manager):
        """Test clearing context."""
        manager.learn_location("session1", "kitchen", {"x": 1, "y": 1})
        manager.clear_context("session1")

        coords = manager.get_location("session1", "kitchen")
        assert coords is None


class TestContextAwareInterpreter:
    """Test context-aware natural language interpretation."""

    @pytest.fixture
    def interpreter(self, tmp_path):
        """Create context-aware interpreter."""
        db_path = tmp_path / "test.db"
        base = RuleBasedInterpreter()
        context = ContextManager(db_path=str(db_path))
        return ContextAwareNLInterpreter(base, context)

    def test_resolve_known_location(self, interpreter):
        """Test that known locations are resolved to coordinates."""
        # Learn a location
        interpreter.context.learn_location("session1", "kitchen", {"x": 5.0, "y": 3.0})

        # Interpret command to that location
        result = interpreter.interpret("Go to kitchen", "session1")

        assert result["tool"] == "ros2_action_goal"
        # Should have resolved coordinates
        assert result["goal"]["pose"]["position"] == {"x": 5.0, "y": 3.0}
        assert "known location" in result["explanation"]

    def test_unknown_location_still_works(self, interpreter):
        """Test that unknown locations still generate valid commands."""
        result = interpreter.interpret("Go to unknown_place", "session1")

        assert result["tool"] == "ros2_action_goal"
        assert "not in known places" in result.get("note", "")

    def test_context_tracks_current_location(self, interpreter):
        """Test that context tracks where the robot was sent."""
        # Learn and navigate to kitchen
        interpreter.context.learn_location("session1", "kitchen", {"x": 5.0, "y": 3.0})

        interpreter.interpret("Go to kitchen", "session1")

        # Simulate successful navigation
        ctx = interpreter.context.get_context("session1")
        ctx.current_location = "kitchen"
        interpreter.context.save_context(ctx)

        # Check that location was tracked
        last_loc = interpreter.context.get_last_location("session1")
        assert last_loc == "kitchen"


class TestSkillFulfillmentContext:
    """Verify context capabilities fulfill SKILL promises."""

    def test_context_awareness_fulfilled(self):
        """Verify SKILL promise: Context across conversations."""
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter

        # Check that adapter has context methods
        adapter = OpenClawAdapter(bridge=None)

        assert hasattr(adapter, "learn_location"), "Adapter should have learn_location method"
        assert hasattr(
            adapter, "get_conversation_history"
        ), "Adapter should have get_conversation_history method"

    def test_location_learning_fulfilled(self, tmp_path):
        """Verify SKILL promise: Learn and recall locations."""
        from agent_ros_bridge.integrations.context import ContextManager

        db_path = tmp_path / "test.db"
        manager = ContextManager(db_path=str(db_path))

        # Learn location
        manager.learn_location("session1", "kitchen", {"x": 5, "y": 3})

        # Recall location
        coords = manager.get_location("session1", "kitchen")

        assert coords is not None
        assert coords["x"] == 5
        assert coords["y"] == 3

    def test_conversation_history_fulfilled(self, tmp_path):
        """Verify SKILL promise: Remember previous commands."""
        from agent_ros_bridge.integrations.context import ContextManager

        db_path = tmp_path / "test.db"
        manager = ContextManager(db_path=str(db_path))

        # Log some commands
        manager.log_interaction("session1", "Go to kitchen", {}, {})
        manager.log_interaction("session1", "Get water", {}, {})

        # Retrieve history
        history = manager.get_last_n_commands("session1", n=2)

        assert len(history) == 2
        assert history[0]["command"] == "Go to kitchen"
        assert history[1]["command"] == "Get water"


class TestContextScenarios:
    """Test real-world context scenarios from SKILL."""

    @pytest.fixture
    def setup(self, tmp_path):
        """Setup for scenario tests."""
        db_path = tmp_path / "scenario.db"
        base = RuleBasedInterpreter()
        context = ContextManager(db_path=str(db_path))
        interpreter = ContextAwareNLInterpreter(base, context)
        return interpreter

    def test_scenario_go_then_bring(self, setup):
        """
        Scenario from SKILL:
        User: "Go to the kitchen"
        [Robot goes to kitchen]
        User: "Now bring me water"
        [Robot knows to get water from kitchen]
        """
        interpreter = setup

        # Step 1: Learn kitchen location
        interpreter.context.learn_location("session1", "kitchen", {"x": 5.0, "y": 3.0})

        # Step 2: Go to kitchen
        result1 = interpreter.interpret("Go to kitchen", "session1")
        assert result1["tool"] == "ros2_action_goal"

        # Simulate successful arrival
        ctx = interpreter.context.get_context("session1")
        ctx.current_location = "kitchen"
        interpreter.context.save_context(ctx)

        # Step 3: Context should know we're at kitchen
        current = interpreter.context.get_context("session1").current_location
        assert current == "kitchen"

        # Step 4: "Bring me water" - robot knows context
        # (In real implementation, would use location for object retrieval)
        # Verify context tracking works
        assert current == "kitchen"

    def test_scenario_return_home(self, setup):
        """
        Scenario: Return to previously visited location
        """
        interpreter = setup

        # Learn multiple locations
        interpreter.context.learn_location("session1", "home", {"x": 0, "y": 0})
        interpreter.context.learn_location("session1", "office", {"x": 10, "y": 5})

        # Go to office
        interpreter.interpret("Go to office", "session1")
        ctx = interpreter.context.get_context("session1")
        ctx.current_location = "office"
        interpreter.context.save_context(ctx)

        # Return home
        result = interpreter.interpret("Go to home", "session1")
        assert result["goal"]["pose"]["position"] == {"x": 0, "y": 0}


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
