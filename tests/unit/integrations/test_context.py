"""Tests for context module (non-async version)."""

import json
import sqlite3
from unittest.mock import MagicMock, Mock, patch

import pytest

from agent_ros_bridge.integrations.context import (
    ContextAwareNLInterpreter,
    ContextManager,
    ConversationContext,
)


def _create_mock_conn():
    """Create a properly configured mock SQLite connection."""
    mock_conn = MagicMock()
    mock_cursor = MagicMock()
    mock_cursor.fetchone.return_value = None
    mock_cursor.fetchall.return_value = []
    mock_conn.execute.return_value = mock_cursor
    mock_conn.__enter__.return_value = mock_conn
    mock_conn.__exit__.return_value = False
    return mock_conn, mock_cursor


class TestConversationContext:
    """Test ConversationContext dataclass."""

    def test_context_creation(self):
        """Test creating context."""
        ctx = ConversationContext(session_id="test_session")
        assert ctx.session_id == "test_session"
        assert ctx.known_locations == {}
        assert ctx.conversation_history == []

    def test_context_to_dict(self):
        """Test converting to dict."""
        ctx = ConversationContext(
            session_id="test",
            current_location="kitchen",
            known_locations={"kitchen": {"x": 10}},
        )
        data = ctx.to_dict()
        assert data["session_id"] == "test"
        assert data["current_location"] == "kitchen"


class TestContextManager:
    """Test ContextManager."""

    def test_init(self):
        """Test initialization."""
        with patch("sqlite3.connect") as mock_connect:
            mock_conn, _ = _create_mock_conn()
            mock_connect.return_value = mock_conn
            
            manager = ContextManager()
            assert manager.db_path.name == ".agent_ros_context.db"

    def test_get_context_new(self):
        """Test getting new context."""
        with patch("sqlite3.connect") as mock_connect:
            mock_conn, mock_cursor = _create_mock_conn()
            mock_cursor.fetchone.return_value = None
            mock_connect.return_value = mock_conn
            
            manager = ContextManager(":memory:")
            ctx = manager.get_context("new_session")
            
            assert ctx.session_id == "new_session"

    def test_save_context(self):
        """Test saving context."""
        with patch("sqlite3.connect") as mock_connect:
            mock_conn, _ = _create_mock_conn()
            mock_connect.return_value = mock_conn
            
            manager = ContextManager(":memory:")
            ctx = ConversationContext(session_id="test", current_location="kitchen")
            manager.save_context(ctx)
            
            # Check that execute was called with INSERT/REPLACE
            calls = [call for call in mock_conn.execute.call_args_list 
                     if 'INSERT' in str(call) or 'REPLACE' in str(call)]
            assert len(calls) > 0

    def test_learn_location(self):
        """Test learning location."""
        with patch("sqlite3.connect") as mock_connect:
            mock_conn, _ = _create_mock_conn()
            mock_connect.return_value = mock_conn
            
            manager = ContextManager(":memory:")
            manager.learn_location("session1", "kitchen", {"x": 10, "y": 5})
            
            # Check that execute was called with INSERT/REPLACE for locations
            calls = [call for call in mock_conn.execute.call_args_list 
                     if 'locations' in str(call).lower()]
            assert len(calls) > 0

    def test_get_location_found(self):
        """Test getting existing location."""
        with patch("sqlite3.connect") as mock_connect:
            mock_conn, mock_cursor = _create_mock_conn()
            # First call is for get_context (5 columns), second is for location lookup
            mock_cursor.fetchone.side_effect = [
                (None, None, None, None, None),  # get_context row
                (json.dumps({"x": 10, "y": 5}),),  # location row
            ]
            mock_connect.return_value = mock_conn
            
            manager = ContextManager(":memory:")
            coords = manager.get_location("session1", "kitchen")
            
            assert coords == {"x": 10, "y": 5}

    def test_get_location_not_found(self):
        """Test getting non-existent location."""
        with patch("sqlite3.connect") as mock_connect:
            mock_conn, mock_cursor = _create_mock_conn()
            # First call is for get_context, second for location lookup (both return None)
            mock_cursor.fetchone.side_effect = [
                None,  # get_context row
                None,  # location row
            ]
            mock_connect.return_value = mock_conn
            
            manager = ContextManager(":memory:")
            coords = manager.get_location("session1", "unknown")
            
            assert coords is None


class TestContextAwareNLInterpreter:
    """Test ContextAwareNLInterpreter."""

    def test_init(self):
        """Test initialization."""
        mock_nl = MagicMock()
        mock_ctx = MagicMock()
        interpreter = ContextAwareNLInterpreter(mock_nl, mock_ctx)
        
        assert interpreter.base is mock_nl
        assert interpreter.context is mock_ctx

    def test_interpret(self):
        """Test interpretation."""
        mock_nl = MagicMock()
        mock_nl.interpret.return_value = {"tool": "navigate"}
        mock_ctx = MagicMock()
        mock_ctx.get_context.return_value = MagicMock(known_locations={"kitchen": {"x": 10}})
        
        interpreter = ContextAwareNLInterpreter(mock_nl, mock_ctx)
        result = interpreter.interpret("go to kitchen", "session1")
        
        assert result["tool"] == "navigate"
