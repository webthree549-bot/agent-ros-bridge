"""Tests for async context management."""

import json
from unittest.mock import ANY, AsyncMock, MagicMock, Mock, patch

import pytest

from agent_ros_bridge.integrations.context_async import (
    AsyncContextManager,
    ConversationContext,
)


class TestConversationContext:
    """Test ConversationContext dataclass."""

    def test_context_creation(self):
        """Test creating context."""
        ctx = ConversationContext(session_id="test_session")
        assert ctx.session_id == "test_session"
        assert ctx.known_locations == {}
        assert ctx.conversation_history == []

    def test_context_with_data(self):
        """Test creating context with data."""
        ctx = ConversationContext(
            session_id="test_session",
            current_location="kitchen",
            known_locations={"kitchen": {"x": 10, "y": 5}},
        )
        assert ctx.current_location == "kitchen"
        assert "kitchen" in ctx.known_locations

    def test_to_dict(self):
        """Test converting to dict."""
        ctx = ConversationContext(
            session_id="test_session",
            current_location="kitchen",
            conversation_history=[{"command": "go"}],
        )
        data = ctx.to_dict()
        assert data["session_id"] == "test_session"
        assert data["current_location"] == "kitchen"


class TestAsyncContextManagerInitialization:
    """Test async context manager initialization."""

    def test_init_default(self):
        """Test default initialization."""
        manager = AsyncContextManager()
        assert manager.db_path.name == ".agent_ros_context.db"
        assert manager._initialized is False

    def test_init_custom_path(self):
        """Test custom database path."""
        manager = AsyncContextManager("/tmp/test.db")
        assert str(manager.db_path) == "/tmp/test.db"


class TestAsyncContextManager:
    """Test async context manager operations."""

    @pytest.mark.asyncio
    async def test_init_db(self):
        """Test database initialization."""
        with patch("aiosqlite.connect") as mock_connect:
            mock_conn = AsyncMock()
            mock_connect.return_value.__aenter__ = AsyncMock(return_value=mock_conn)
            mock_connect.return_value.__aexit__ = AsyncMock(return_value=False)
            
            manager = AsyncContextManager()
            await manager._init_db()
            
            assert manager._initialized is True
            assert mock_conn.execute.call_count >= 3

    @pytest.mark.asyncio
    async def test_get_context_new(self):
        """Test getting new context."""
        with patch("aiosqlite.connect") as mock_connect:
            mock_conn = AsyncMock()
            mock_cursor = AsyncMock()
            mock_cursor.fetchone = AsyncMock(return_value=None)
            mock_conn.execute = AsyncMock(return_value=mock_cursor)
            mock_connect.return_value.__aenter__ = AsyncMock(return_value=mock_conn)
            mock_connect.return_value.__aexit__ = AsyncMock(return_value=False)
            
            manager = AsyncContextManager()
            ctx = await manager.get_context("new_session")
            
            assert ctx.session_id == "new_session"
            assert ctx.current_location is None

    @pytest.mark.asyncio
    async def test_get_context_existing(self):
        """Test getting existing context."""
        with patch("aiosqlite.connect") as mock_connect:
            mock_conn = AsyncMock()
            mock_cursor = AsyncMock()
            mock_cursor.fetchone = AsyncMock(return_value={
                "current_location": "kitchen",
                "last_action": "navigate",
                "last_result": json.dumps({"success": True}),
                "known_locations": json.dumps({"kitchen": {"x": 10}}),
                "pending_task": None,
            })
            mock_conn.execute = AsyncMock(return_value=mock_cursor)
            mock_connect.return_value.__aenter__ = AsyncMock(return_value=mock_conn)
            mock_connect.return_value.__aexit__ = AsyncMock(return_value=False)
            
            manager = AsyncContextManager()
            ctx = await manager.get_context("existing_session")
            
            assert ctx.session_id == "existing_session"
            assert ctx.current_location == "kitchen"

    @pytest.mark.asyncio
    async def test_save_context(self):
        """Test saving context."""
        with patch("aiosqlite.connect") as mock_connect:
            mock_conn = AsyncMock()
            mock_connect.return_value.__aenter__ = AsyncMock(return_value=mock_conn)
            mock_connect.return_value.__aexit__ = AsyncMock(return_value=False)
            
            manager = AsyncContextManager()
            ctx = ConversationContext(
                session_id="test",
                current_location="kitchen",
                known_locations={"kitchen": {"x": 10}},
            )
            await manager.save_context(ctx)
            
            mock_conn.execute.assert_called()
            mock_conn.commit.assert_called_once()

    @pytest.mark.asyncio
    async def test_log_interaction(self):
        """Test logging interaction."""
        with patch("aiosqlite.connect") as mock_connect:
            mock_conn = AsyncMock()
            mock_cursor = AsyncMock()
            mock_cursor.fetchone = AsyncMock(return_value=None)
            mock_conn.execute = AsyncMock(return_value=mock_cursor)
            mock_connect.return_value.__aenter__ = AsyncMock(return_value=mock_conn)
            mock_connect.return_value.__aexit__ = AsyncMock(return_value=False)
            
            manager = AsyncContextManager()
            await manager.log_interaction(
                "session1",
                "go to kitchen",
                {"tool": "navigate"},
                {"success": True},
            )
            
            mock_conn.execute.assert_called()
            mock_conn.commit.assert_called_once()

    @pytest.mark.asyncio
    async def test_learn_location(self):
        """Test learning location."""
        with patch("aiosqlite.connect") as mock_connect:
            mock_conn = AsyncMock()
            mock_cursor = AsyncMock()
            mock_cursor.fetchone = AsyncMock(return_value=None)
            mock_conn.execute = AsyncMock(return_value=mock_cursor)
            mock_connect.return_value.__aenter__ = AsyncMock(return_value=mock_conn)
            mock_connect.return_value.__aexit__ = AsyncMock(return_value=False)
            
            manager = AsyncContextManager()
            await manager.learn_location("session1", "kitchen", {"x": 10, "y": 5})
            
            mock_conn.execute.assert_called()
            mock_conn.commit.assert_called_once()

    @pytest.mark.asyncio
    async def test_get_location_found(self):
        """Test getting existing location."""
        with patch("aiosqlite.connect") as mock_connect:
            mock_conn = AsyncMock()
            mock_cursor = AsyncMock()
            mock_cursor.fetchone = AsyncMock(return_value={
                "coordinates": json.dumps({"x": 10, "y": 5}),
            })
            mock_conn.execute = AsyncMock(return_value=mock_cursor)
            mock_connect.return_value.__aenter__ = AsyncMock(return_value=mock_conn)
            mock_connect.return_value.__aexit__ = AsyncMock(return_value=False)
            
            manager = AsyncContextManager()
            coords = await manager.get_location("session1", "kitchen")
            
            assert coords == {"x": 10, "y": 5}

    @pytest.mark.asyncio
    async def test_get_location_not_found(self):
        """Test getting non-existent location."""
        with patch("aiosqlite.connect") as mock_connect:
            mock_conn = AsyncMock()
            mock_cursor = AsyncMock()
            mock_cursor.fetchone = AsyncMock(return_value=None)
            mock_conn.execute = AsyncMock(return_value=mock_cursor)
            mock_connect.return_value.__aenter__ = AsyncMock(return_value=mock_conn)
            mock_connect.return_value.__aexit__ = AsyncMock(return_value=False)
            
            manager = AsyncContextManager()
            coords = await manager.get_location("session1", "unknown")
            
            assert coords is None

    @pytest.mark.asyncio
    async def test_get_last_n_commands(self):
        """Test getting recent commands."""
        with patch("aiosqlite.connect") as mock_connect:
            mock_conn = AsyncMock()
            mock_cursor = AsyncMock()
            mock_cursor.fetchone = AsyncMock(return_value=None)
            mock_conn.execute = AsyncMock(return_value=mock_cursor)
            mock_connect.return_value.__aenter__ = AsyncMock(return_value=mock_conn)
            mock_connect.return_value.__aexit__ = AsyncMock(return_value=False)
            
            manager = AsyncContextManager()
            ctx = ConversationContext(session_id="test")
            ctx.conversation_history = [{"command": "go"}, {"command": "stop"}]
            manager._runtime_contexts["test"] = ctx
            
            history = await manager.get_last_n_commands("test", n=5)
            
            assert len(history) == 2

    @pytest.mark.asyncio
    async def test_clear_context(self):
        """Test clearing context."""
        with patch("aiosqlite.connect") as mock_connect:
            mock_conn = AsyncMock()
            mock_connect.return_value.__aenter__ = AsyncMock(return_value=mock_conn)
            mock_connect.return_value.__aexit__ = AsyncMock(return_value=False)
            
            manager = AsyncContextManager()
            manager._runtime_contexts["session1"] = ConversationContext(session_id="session1")
            
            await manager.clear_context("session1")
            
            assert "session1" not in manager._runtime_contexts
            assert mock_conn.execute.call_count == 3
