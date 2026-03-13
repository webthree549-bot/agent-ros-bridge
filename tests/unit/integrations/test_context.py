"""Unit tests for integrations/context.py.

Tests the context management system without external dependencies.
"""

import pytest
import json
import sqlite3
import tempfile
import os
from pathlib import Path
from datetime import datetime
from unittest.mock import Mock, patch

from agent_ros_bridge.integrations.context import (
    ConversationContext,
    ContextManager,
    ContextAwareNLInterpreter,
)


class TestConversationContext:
    """Test ConversationContext dataclass."""

    def test_context_creation(self):
        """Context can be created."""
        ctx = ConversationContext(session_id="session_123")

        assert ctx.session_id == "session_123"
        assert ctx.current_location is None
        assert ctx.last_action is None
        assert ctx.last_result is None
        assert ctx.known_locations == {}
        assert ctx.pending_task is None
        assert ctx.conversation_history == []

    def test_context_to_dict(self):
        """Context can be converted to dict."""
        ctx = ConversationContext(
            session_id="session_123",
            current_location="kitchen",
            last_action="navigate",
            last_result={"status": "success"},
            known_locations={"kitchen": {"x": 1.0, "y": 2.0}},
            pending_task="fetch_water",
            conversation_history=[{"command": "go to kitchen"}],
        )

        d = ctx.to_dict()

        assert d["session_id"] == "session_123"
        assert d["current_location"] == "kitchen"
        assert d["last_action"] == "navigate"
        assert d["last_result"] == {"status": "success"}
        assert d["known_locations"] == {"kitchen": {"x": 1.0, "y": 2.0}}
        assert d["pending_task"] == "fetch_water"
        assert len(d["conversation_history"]) == 1

    def test_context_to_dict_limits_history(self):
        """to_dict limits history to last 10 items."""
        ctx = ConversationContext(session_id="session_123")
        ctx.conversation_history = [{"command": f"cmd_{i}"} for i in range(15)]

        d = ctx.to_dict()

        assert len(d["conversation_history"]) == 10


class TestContextManager:
    """Test ContextManager class."""

    @pytest.fixture
    def temp_db(self):
        """Create a temporary database file."""
        with tempfile.NamedTemporaryFile(suffix=".db", delete=False) as f:
            db_path = f.name
        yield db_path
        # Cleanup
        if os.path.exists(db_path):
            os.unlink(db_path)

    @pytest.fixture
    def manager(self, temp_db):
        """Create a context manager with temp database."""
        return ContextManager(db_path=temp_db)

    def test_manager_creation(self, temp_db):
        """Manager can be created."""
        manager = ContextManager(db_path=temp_db)

        assert manager.db_path == Path(temp_db)
        assert manager._runtime_contexts == {}

    def test_manager_creates_database(self, temp_db):
        """Manager creates database on initialization."""
        manager = ContextManager(db_path=temp_db)

        assert os.path.exists(temp_db)

        # Check tables exist
        with sqlite3.connect(temp_db) as conn:
            cursor = conn.execute("SELECT name FROM sqlite_master WHERE type='table'")
            tables = [row[0] for row in cursor.fetchall()]

            assert "contexts" in tables
            assert "history" in tables
            assert "locations" in tables

    def test_get_context_new_session(self, manager):
        """Get context creates new context for unknown session."""
        ctx = manager.get_context("new_session")

        assert ctx.session_id == "new_session"
        assert ctx.current_location is None
        assert "new_session" in manager._runtime_contexts

    def test_get_context_caches_in_runtime(self, manager):
        """Get context caches context in runtime."""
        ctx1 = manager.get_context("session_1")
        ctx2 = manager.get_context("session_1")

        assert ctx1 is ctx2  # Same object

    def test_save_context(self, manager):
        """Context can be saved to database."""
        ctx = ConversationContext(
            session_id="session_1",
            current_location="kitchen",
            last_action="navigate",
            last_result={"status": "success"},
            known_locations={"kitchen": {"x": 1.0}},
            pending_task="task_1",
        )

        manager.save_context(ctx)

        # Verify in database
        with sqlite3.connect(manager.db_path) as conn:
            row = conn.execute(
                "SELECT current_location, last_action FROM contexts WHERE session_id = ?",
                ("session_1",),
            ).fetchone()

            assert row[0] == "kitchen"
            assert row[1] == "navigate"

    def test_log_interaction(self, manager):
        """Interaction can be logged."""
        manager.log_interaction(
            session_id="session_1",
            command="go to kitchen",
            interpretation={"action": "navigate"},
            result={"status": "success"},
        )

        # Verify in database
        with sqlite3.connect(manager.db_path) as conn:
            row = conn.execute(
                "SELECT command, interpretation, result FROM history WHERE session_id = ?",
                ("session_1",),
            ).fetchone()

            assert row[0] == "go to kitchen"
            assert json.loads(row[1]) == {"action": "navigate"}
            assert json.loads(row[2]) == {"status": "success"}

    def test_log_interaction_updates_runtime(self, manager):
        """Logging interaction updates runtime context."""
        # Get context first to initialize runtime cache
        ctx = manager.get_context("session_1")

        manager.log_interaction(
            session_id="session_1",
            command="go to kitchen",
            interpretation={"action": "navigate"},
            result={"status": "success"},
        )

        assert len(ctx.conversation_history) == 1
        assert ctx.conversation_history[0]["command"] == "go to kitchen"

    def test_log_interaction_limits_history(self, manager):
        """Logging interaction limits history to last 10."""
        ctx = manager.get_context("session_1")

        for i in range(15):
            manager.log_interaction(
                session_id="session_1",
                command=f"command_{i}",
                interpretation={},
                result={},
            )

        assert len(ctx.conversation_history) == 10

    def test_learn_location(self, manager):
        """Location can be learned."""
        manager.learn_location(
            session_id="session_1",
            name="kitchen",
            coordinates={"x": 1.0, "y": 2.0, "z": 0.0},
        )

        # Verify in database
        with sqlite3.connect(manager.db_path) as conn:
            row = conn.execute(
                "SELECT coordinates FROM locations WHERE session_id = ? AND name = ?",
                ("session_1", "kitchen"),
            ).fetchone()

            assert json.loads(row[0]) == {"x": 1.0, "y": 2.0, "z": 0.0}

    def test_learn_location_updates_runtime(self, manager):
        """Learning location updates runtime context."""
        ctx = manager.get_context("session_1")

        manager.learn_location(
            session_id="session_1",
            name="kitchen",
            coordinates={"x": 1.0, "y": 2.0},
        )

        assert "kitchen" in ctx.known_locations
        assert ctx.known_locations["kitchen"] == {"x": 1.0, "y": 2.0}
        assert ctx.current_location == "kitchen"

    def test_learn_location_case_insensitive(self, manager):
        """Location names are case insensitive."""
        manager.learn_location(
            session_id="session_1",
            name="KITCHEN",
            coordinates={"x": 1.0, "y": 2.0},
        )

        # Should be stored as lowercase
        with sqlite3.connect(manager.db_path) as conn:
            row = conn.execute(
                "SELECT name FROM locations WHERE session_id = ?",
                ("session_1",),
            ).fetchone()

            assert row[0] == "kitchen"

    def test_get_location(self, manager):
        """Location can be retrieved."""
        manager.learn_location(
            session_id="session_1",
            name="kitchen",
            coordinates={"x": 1.0, "y": 2.0},
        )

        coords = manager.get_location("session_1", "kitchen")

        assert coords == {"x": 1.0, "y": 2.0}

    def test_get_location_not_found(self, manager):
        """Getting unknown location returns None."""
        coords = manager.get_location("session_1", "unknown")

        assert coords is None

    def test_get_location_updates_cache(self, manager):
        """Getting location updates runtime cache."""
        # Learn location without using get_context first
        manager.learn_location(
            session_id="session_1",
            name="kitchen",
            coordinates={"x": 1.0, "y": 2.0},
        )

        # Clear runtime cache
        manager._runtime_contexts = {}

        # Get location (should load from DB and update cache)
        coords = manager.get_location("session_1", "kitchen")

        ctx = manager._runtime_contexts.get("session_1")
        assert ctx is not None
        assert "kitchen" in ctx.known_locations

    def test_get_last_n_commands(self, manager):
        """Recent commands can be retrieved."""
        for i in range(5):
            manager.log_interaction(
                session_id="session_1",
                command=f"command_{i}",
                interpretation={},
                result={},
            )

        commands = manager.get_last_n_commands("session_1", n=3)

        assert len(commands) == 3
        assert commands[0]["command"] == "command_2"
        assert commands[2]["command"] == "command_4"

    def test_get_last_location(self, manager):
        """Last location can be retrieved."""
        manager.learn_location(
            session_id="session_1",
            name="kitchen",
            coordinates={"x": 1.0, "y": 2.0},
        )

        location = manager.get_last_location("session_1")

        assert location == "kitchen"

    def test_clear_context(self, manager):
        """Context can be cleared."""
        # Set up some data
        manager.learn_location("session_1", "kitchen", {"x": 1.0})
        manager.log_interaction("session_1", "cmd", {}, {})
        ctx = manager.get_context("session_1")

        manager.clear_context("session_1")

        # Verify runtime cache cleared
        assert "session_1" not in manager._runtime_contexts

        # Verify database cleared
        with sqlite3.connect(manager.db_path) as conn:
            ctx_count = conn.execute(
                "SELECT COUNT(*) FROM contexts WHERE session_id = ?", ("session_1",)
            ).fetchone()[0]
            hist_count = conn.execute(
                "SELECT COUNT(*) FROM history WHERE session_id = ?", ("session_1",)
            ).fetchone()[0]
            loc_count = conn.execute(
                "SELECT COUNT(*) FROM locations WHERE session_id = ?", ("session_1",)
            ).fetchone()[0]

            assert ctx_count == 0
            assert hist_count == 0
            assert loc_count == 0

    def test_list_learned_locations(self, manager):
        """Learned locations can be listed."""
        manager.learn_location("session_1", "kitchen", {"x": 1.0})
        manager.learn_location("session_1", "office", {"x": 2.0})
        manager.learn_location("session_1", "bedroom", {"x": 3.0})

        locations = manager.list_learned_locations("session_1")

        assert len(locations) == 3
        assert "kitchen" in locations
        assert "office" in locations
        assert "bedroom" in locations

    def test_load_history_from_db(self, manager):
        """History is loaded from database when getting context."""
        # Add history directly to database
        with sqlite3.connect(manager.db_path) as conn:
            for i in range(5):
                conn.execute(
                    """INSERT INTO history (session_id, command, interpretation, result)
                       VALUES (?, ?, ?, ?)""",
                    ("session_1", f"cmd_{i}", "{}", "{}"),
                )

        # Get context (should load history)
        ctx = manager.get_context("session_1")

        assert len(ctx.conversation_history) == 5


class TestContextAwareNLInterpreter:
    """Test ContextAwareNLInterpreter class."""

    @pytest.fixture
    def mock_base_interpreter(self):
        """Create a mock base interpreter."""
        interpreter = Mock()
        interpreter.interpret = Mock(return_value={
            "tool": "ros2_action_goal",
            "action": "navigate",
            "goal": {"pose": {"position": {}}},
        })
        return interpreter

    @pytest.fixture
    def manager(self):
        """Create a context manager with temp database."""
        with tempfile.NamedTemporaryFile(suffix=".db", delete=False) as f:
            db_path = f.name
        manager = ContextManager(db_path=db_path)
        yield manager
        if os.path.exists(db_path):
            os.unlink(db_path)

    @pytest.fixture
    def interpreter(self, mock_base_interpreter, manager):
        """Create a context-aware interpreter."""
        return ContextAwareNLInterpreter(mock_base_interpreter, manager)

    def test_interpreter_creation(self, mock_base_interpreter, manager):
        """Interpreter can be created."""
        interpreter = ContextAwareNLInterpreter(mock_base_interpreter, manager)

        assert interpreter.base == mock_base_interpreter
        assert interpreter.context == manager

    def test_interpret_with_context(self, interpreter, mock_base_interpreter):
        """Interpret uses context."""
        result = interpreter.interpret("go to kitchen", session_id="session_1")

        mock_base_interpreter.interpret.assert_called_once()
        # Check that context was passed
        call_args = mock_base_interpreter.interpret.call_args
        assert "context" in call_args.kwargs

    def test_interpret_updates_last_action(self, interpreter):
        """Interpret updates last action in context."""
        interpreter.interpret("go to kitchen", session_id="session_1")

        ctx = interpreter.context.get_context("session_1")
        assert ctx.last_action == "go to kitchen"

    def test_resolve_references_go_back(self, interpreter):
        """Resolve 'go back' references."""
        ctx = interpreter.context.get_context("session_1")
        ctx.current_location = "kitchen"

        resolved = interpreter._resolve_references("go back", ctx)

        # Should pass through (actual navigation handled elsewhere)
        assert "go back" in resolved.lower()

    def test_resolve_references_bring_me(self, interpreter):
        """Resolve 'bring me' references."""
        ctx = interpreter.context.get_context("session_1")
        ctx.current_location = "kitchen"

        resolved = interpreter._resolve_references("bring me water", ctx)

        assert "bring me water" in resolved.lower()

    def test_learn_current_location(self, interpreter):
        """Current location can be learned."""
        interpreter.learn_current_location(
            session_id="session_1",
            name="charging_station",
            coordinates={"x": 5.0, "y": 5.0},
        )

        coords = interpreter.context.get_location("session_1", "charging_station")
        assert coords == {"x": 5.0, "y": 5.0}

    def test_learn_current_location_without_coords(self, interpreter):
        """Current location can be learned without explicit coordinates."""
        # First set a current location
        ctx = interpreter.context.get_context("session_1")
        ctx.current_location = "kitchen"

        # This would normally get coordinates from robot
        interpreter.learn_current_location(
            session_id="session_1",
            name="current_spot",
            coordinates=None,  # Would use robot position
        )

        # Should use placeholder coordinates
        coords = interpreter.context.get_location("session_1", "current_spot")
        assert coords == {"x": 0, "y": 0, "z": 0}


class TestContextManagerEdgeCases:
    """Test ContextManager edge cases."""

    def test_concurrent_sessions(self):
        """Multiple sessions can be managed independently."""
        with tempfile.NamedTemporaryFile(suffix=".db", delete=False) as f:
            db_path = f.name

        try:
            manager = ContextManager(db_path=db_path)

            # Set up data for two sessions
            manager.learn_location("session_1", "kitchen", {"x": 1.0})
            manager.learn_location("session_2", "office", {"x": 2.0})

            # Verify isolation
            assert manager.get_location("session_1", "kitchen") == {"x": 1.0}
            assert manager.get_location("session_2", "office") == {"x": 2.0}
            assert manager.get_location("session_1", "office") is None
            assert manager.get_location("session_2", "kitchen") is None
        finally:
            if os.path.exists(db_path):
                os.unlink(db_path)

    def test_update_existing_location(self):
        """Updating existing location overwrites it."""
        with tempfile.NamedTemporaryFile(suffix=".db", delete=False) as f:
            db_path = f.name

        try:
            manager = ContextManager(db_path=db_path)

            manager.learn_location("session_1", "kitchen", {"x": 1.0})
            manager.learn_location("session_1", "kitchen", {"x": 2.0})

            coords = manager.get_location("session_1", "kitchen")
            assert coords == {"x": 2.0}
        finally:
            if os.path.exists(db_path):
                os.unlink(db_path)

    def test_persistence_across_instances(self):
        """Data persists across manager instances."""
        with tempfile.NamedTemporaryFile(suffix=".db", delete=False) as f:
            db_path = f.name

        try:
            # First instance
            manager1 = ContextManager(db_path=db_path)
            manager1.learn_location("session_1", "kitchen", {"x": 1.0})

            # Second instance (same database)
            manager2 = ContextManager(db_path=db_path)
            coords = manager2.get_location("session_1", "kitchen")

            assert coords == {"x": 1.0}
        finally:
            if os.path.exists(db_path):
                os.unlink(db_path)
