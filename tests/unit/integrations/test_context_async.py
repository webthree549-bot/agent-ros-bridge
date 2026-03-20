"""Tests for async context management."""

import json
from unittest.mock import ANY, AsyncMock, MagicMock, Mock, patch

import pytest

from agent_ros_bridge.integrations.context_async import (
    AsyncContextManager,
    ConversationContext,
)


class AsyncContextManagerMock:
    """A mock that can be both awaited and used as an async context manager."""

    def __init__(self, return_value=None):
        self._return_value = return_value
        self.call_count = 0
        self.call_args_list = []

    def __call__(self, *args, **kwargs):
        self.call_count += 1
        self.call_args_list.append((args, kwargs))
        return self

    def __await__(self):
        async def _await():
            return self._return_value

        return _await().__await__()

    async def __aenter__(self):
        return self._return_value

    async def __aexit__(self, *args):
        return False

    def assert_called(self):
        assert self.call_count > 0, "Expected mock to be called at least once"


class MockRow:
    """A mock database row that supports dict-like access."""

    def __init__(self, data: dict):
        self._data = data

    def __getitem__(self, key):
        return self._data[key]

    def __contains__(self, key):
        return key in self._data


def create_mock_db_connection(fetchone_return=None):
    """Create a properly mocked aiosqlite connection for testing.

    Returns a tuple of (mock_connect, mock_conn, mock_cursor) that can be used
    with patch("aiosqlite.connect") to mock database operations.
    """
    # Wrap dict in MockRow if needed
    if fetchone_return is not None and isinstance(fetchone_return, dict):
        fetchone_return = MockRow(fetchone_return)

    # Use a simple class instead of MagicMock to avoid attribute access issues
    class MockCursor:
        def __init__(self, return_value):
            self._return_value = return_value

        async def fetchone(self):
            return self._return_value

        def __aiter__(self):
            return self

        async def __anext__(self):
            raise StopAsyncIteration

    mock_cursor = MockCursor(fetchone_return)

    mock_conn = MagicMock()
    # execute returns an object that can be awaited AND used as async context manager
    mock_conn.execute = AsyncContextManagerMock(return_value=mock_cursor)
    mock_conn.commit = AsyncMock()
    mock_conn.__aenter__ = AsyncMock(return_value=mock_conn)
    mock_conn.__aexit__ = AsyncMock(return_value=False)

    mock_connect = MagicMock()
    mock_connect.return_value.__aenter__ = AsyncMock(return_value=mock_conn)
    mock_connect.return_value.__aexit__ = AsyncMock(return_value=False)

    return mock_connect, mock_conn, mock_cursor


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
        mock_connect, mock_conn, _ = create_mock_db_connection()

        with patch("aiosqlite.connect", mock_connect):
            manager = AsyncContextManager()
            await manager._init_db()

            assert manager._initialized is True
            assert mock_conn.execute.call_count >= 3

    @pytest.mark.asyncio
    async def test_get_context_new(self):
        """Test getting new context."""
        mock_connect, mock_conn, mock_cursor = create_mock_db_connection(fetchone_return=None)

        with patch("aiosqlite.connect", mock_connect):
            manager = AsyncContextManager()
            ctx = await manager.get_context("new_session")

            assert ctx.session_id == "new_session"
            assert ctx.current_location is None

    @pytest.mark.asyncio
    async def test_get_context_existing(self):
        """Test getting existing context."""
        mock_connect, mock_conn, mock_cursor = create_mock_db_connection(
            fetchone_return={
                "current_location": "kitchen",
                "last_action": "navigate",
                "last_result": json.dumps({"success": True}),
                "known_locations": json.dumps({"kitchen": {"x": 10}}),
                "pending_task": None,
            }
        )

        with patch("aiosqlite.connect", mock_connect):
            manager = AsyncContextManager()
            ctx = await manager.get_context("existing_session")

            assert ctx.session_id == "existing_session"
            assert ctx.current_location == "kitchen"

    @pytest.mark.asyncio
    async def test_save_context(self):
        """Test saving context."""
        mock_connect, mock_conn, _ = create_mock_db_connection()

        with patch("aiosqlite.connect", mock_connect):
            manager = AsyncContextManager()
            # Pre-initialize to avoid counting init calls
            manager._initialized = True
            ctx = ConversationContext(
                session_id="test",
                current_location="kitchen",
                known_locations={"kitchen": {"x": 10}},
            )
            await manager.save_context(ctx)

            mock_conn.execute.assert_called()
            assert mock_conn.commit.call_count >= 1

    @pytest.mark.asyncio
    async def test_log_interaction(self):
        """Test logging interaction."""
        mock_connect, mock_conn, mock_cursor = create_mock_db_connection(fetchone_return=None)

        with patch("aiosqlite.connect", mock_connect):
            manager = AsyncContextManager()
            # Pre-initialize to avoid counting init calls
            manager._initialized = True
            await manager.log_interaction(
                "session1",
                "go to kitchen",
                {"tool": "navigate"},
                {"success": True},
            )

            mock_conn.execute.assert_called()
            assert mock_conn.commit.call_count >= 1

    @pytest.mark.asyncio
    async def test_learn_location(self):
        """Test learning location."""
        mock_connect, mock_conn, mock_cursor = create_mock_db_connection(fetchone_return=None)

        with patch("aiosqlite.connect", mock_connect):
            manager = AsyncContextManager()
            # Pre-initialize to avoid counting init calls
            manager._initialized = True
            await manager.learn_location("session1", "kitchen", {"x": 10, "y": 5})

            mock_conn.execute.assert_called()
            assert mock_conn.commit.call_count >= 1

    @pytest.mark.asyncio
    async def test_get_location_found(self):
        """Test getting existing location."""
        mock_connect, mock_conn, mock_cursor = create_mock_db_connection(
            fetchone_return={
                "coordinates": json.dumps({"x": 10, "y": 5}),
                "current_location": "kitchen",
                "last_action": None,
                "last_result": None,
                "known_locations": None,
                "pending_task": None,
            }
        )

        with patch("aiosqlite.connect", mock_connect):
            manager = AsyncContextManager()
            coords = await manager.get_location("session1", "kitchen")

            assert coords == {"x": 10, "y": 5}

    @pytest.mark.asyncio
    async def test_get_location_not_found(self):
        """Test getting non-existent location."""
        mock_connect, mock_conn, mock_cursor = create_mock_db_connection(fetchone_return=None)

        with patch("aiosqlite.connect", mock_connect):
            manager = AsyncContextManager()
            coords = await manager.get_location("session1", "unknown")

            assert coords is None

    @pytest.mark.asyncio
    async def test_get_last_n_commands(self):
        """Test getting recent commands."""
        mock_connect, mock_conn, mock_cursor = create_mock_db_connection(fetchone_return=None)

        with patch("aiosqlite.connect", mock_connect):
            manager = AsyncContextManager()
            ctx = ConversationContext(session_id="test")
            ctx.conversation_history = [{"command": "go"}, {"command": "stop"}]
            manager._runtime_contexts["test"] = ctx

            history = await manager.get_last_n_commands("test", n=5)

            assert len(history) == 2

    @pytest.mark.asyncio
    async def test_clear_context(self):
        """Test clearing context."""
        mock_connect, mock_conn, _ = create_mock_db_connection()

        with patch("aiosqlite.connect", mock_connect):
            manager = AsyncContextManager()
            # Pre-initialize to avoid counting init calls
            manager._initialized = True
            manager._runtime_contexts["session1"] = ConversationContext(session_id="session1")

            await manager.clear_context("session1")

            assert "session1" not in manager._runtime_contexts
            # Should execute 3 delete statements
            assert mock_conn.execute.call_count >= 3
