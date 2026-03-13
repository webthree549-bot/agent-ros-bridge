"""Unit tests for integrations/memory.py.

Tests the memory integration without external dependencies.
"""

import pytest
import json
import sqlite3
import tempfile
import os
from unittest.mock import Mock, patch, AsyncMock

from agent_ros_bridge.integrations.memory import AgentMemory, MemoryEntry
from agent_ros_bridge.integrations.safety import SafetyLevel, SafetyManager
from datetime import datetime, timedelta


class TestMemoryEntry:
    """Test MemoryEntry dataclass."""

    def test_entry_creation(self):
        """MemoryEntry can be created."""
        entry = MemoryEntry(
            key="test_key",
            value={"data": "test"},
            created_at=datetime.now(),
        )

        assert entry.key == "test_key"
        assert entry.value == {"data": "test"}
        assert entry.expires_at is None

    def test_entry_with_expiration(self):
        """MemoryEntry can have expiration."""
        expires = datetime.now() + timedelta(hours=1)
        entry = MemoryEntry(
            key="test_key",
            value="test",
            created_at=datetime.now(),
            expires_at=expires,
        )

        assert entry.expires_at == expires

    def test_is_expired_no_expiration(self):
        """Entry without expiration is not expired."""
        entry = MemoryEntry(
            key="test_key",
            value="test",
            created_at=datetime.now(),
            expires_at=None,
        )

        assert entry.is_expired() is False

    def test_is_expired_future(self):
        """Entry with future expiration is not expired."""
        entry = MemoryEntry(
            key="test_key",
            value="test",
            created_at=datetime.now(),
            expires_at=datetime.now() + timedelta(hours=1),
        )

        assert entry.is_expired() is False

    def test_is_expired_past(self):
        """Entry with past expiration is expired."""
        entry = MemoryEntry(
            key="test_key",
            value="test",
            created_at=datetime.now(),
            expires_at=datetime.now() - timedelta(hours=1),
        )

        assert entry.is_expired() is True


class TestAgentMemory:
    """Test AgentMemory class."""

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
    def memory(self, temp_db):
        """Create an AgentMemory instance with SQLite backend."""
        return AgentMemory(backend="sqlite", path=temp_db)

    def test_memory_creation_sqlite(self, temp_db):
        """Memory can be created with SQLite backend."""
        memory = AgentMemory(backend="sqlite", path=temp_db)

        assert memory.backend == "sqlite"
        assert memory.conn is not None

    def test_memory_creation_in_memory(self):
        """Memory can be created with in-memory SQLite."""
        memory = AgentMemory(backend="sqlite", path=":memory:")

        assert memory.backend == "sqlite"
        assert memory.conn is not None

    def test_memory_creation_unknown_backend(self):
        """Unknown backend raises error."""
        with pytest.raises(ValueError, match="Unknown backend"):
            AgentMemory(backend="unknown")

    @pytest.mark.asyncio
    async def test_set_and_get(self, memory):
        """Value can be set and retrieved."""
        await memory.set("key1", {"data": "value1"})

        result = await memory.get("key1")

        assert result == {"data": "value1"}

    @pytest.mark.asyncio
    async def test_get_nonexistent(self, memory):
        """Getting non-existent key returns None."""
        result = await memory.get("nonexistent")

        assert result is None

    @pytest.mark.asyncio
    async def test_set_with_ttl(self, memory):
        """Value can be set with TTL."""
        await memory.set("key1", "value1", ttl=3600)

        result = await memory.get("key1")
        assert result == "value1"

    @pytest.mark.asyncio
    async def test_set_expired(self, memory):
        """Expired value is not returned."""
        await memory.set("key1", "value1", ttl=-1)  # Already expired

        result = await memory.get("key1")
        assert result is None

    @pytest.mark.asyncio
    async def test_delete(self, memory):
        """Value can be deleted."""
        await memory.set("key1", "value1")
        await memory.delete("key1")

        result = await memory.get("key1")
        assert result is None

    @pytest.mark.asyncio
    async def test_append_to_list(self, memory):
        """Values can be appended to list."""
        await memory.append("list_key", "item1")
        await memory.append("list_key", "item2")
        await memory.append("list_key", "item3")

        result = await memory.get("list_key")

        assert result == ["item1", "item2", "item3"]

    @pytest.mark.asyncio
    async def test_append_to_non_list(self, memory):
        """Appending to non-list converts to list."""
        await memory.set("key1", "original")
        await memory.append("key1", "appended")

        result = await memory.get("key1")

        assert result == ["original", "appended"]

    @pytest.mark.asyncio
    async def test_get_list(self, memory):
        """get_list returns value as list."""
        await memory.set("key1", ["item1", "item2"])

        result = await memory.get_list("key1")

        assert result == ["item1", "item2"]

    @pytest.mark.asyncio
    async def test_get_list_non_list(self, memory):
        """get_list wraps non-list in list."""
        await memory.set("key1", "single_value")

        result = await memory.get_list("key1")

        assert result == ["single_value"]

    @pytest.mark.asyncio
    async def test_get_list_empty(self, memory):
        """get_list returns empty list for missing key."""
        result = await memory.get_list("nonexistent")

        assert result == []

    @pytest.mark.asyncio
    async def test_list_keys(self, memory):
        """Keys can be listed."""
        await memory.set("key1", "value1")
        await memory.set("key2", "value2")
        await memory.set("key3", "value3")

        keys = await memory.list_keys()

        assert len(keys) == 3
        assert "key1" in keys
        assert "key2" in keys
        assert "key3" in keys

    @pytest.mark.asyncio
    async def test_list_keys_skips_expired(self, memory):
        """list_keys skips expired entries."""
        await memory.set("key1", "value1")
        await memory.set("key2", "value2", ttl=-1)  # Expired

        keys = await memory.list_keys()

        assert "key1" in keys
        assert "key2" not in keys


class TestAgentMemoryRedis:
    """Test AgentMemory with Redis backend (mocked)."""

    @pytest.mark.asyncio
    async def test_redis_backend_init(self):
        """Redis backend can be initialized."""
        with patch("redis.from_url") as mock_redis:
            mock_redis.return_value = Mock()

            memory = AgentMemory(backend="redis", url="redis://localhost:6379")

            assert memory.backend == "redis"
            mock_redis.assert_called_once_with("redis://localhost:6379", decode_responses=True)

    def test_redis_backend_import_error(self):
        """Redis backend raises error if redis not installed."""
        with patch.dict("sys.modules", {"redis": None}):
            with pytest.raises(ImportError):
                AgentMemory(backend="redis")

    @pytest.mark.asyncio
    async def test_redis_get(self):
        """Redis get works correctly."""
        with patch("redis.from_url") as mock_redis:
            mock_client = Mock()
            mock_client.get = Mock(return_value='{"data": "value"}')
            mock_redis.return_value = mock_client

            memory = AgentMemory(backend="redis")
            result = await memory.get("key1")

            assert result == {"data": "value"}

    @pytest.mark.asyncio
    async def test_redis_get_none(self):
        """Redis get returns None for missing key."""
        with patch("redis.from_url") as mock_redis:
            mock_client = Mock()
            mock_client.get = Mock(return_value=None)
            mock_redis.return_value = mock_client

            memory = AgentMemory(backend="redis")
            result = await memory.get("key1")

            assert result is None

    @pytest.mark.asyncio
    async def test_redis_set(self):
        """Redis set works correctly."""
        with patch("redis.from_url") as mock_redis:
            mock_client = Mock()
            mock_client.set = Mock()
            mock_redis.return_value = mock_client

            memory = AgentMemory(backend="redis")
            await memory.set("key1", {"data": "value"}, ttl=3600)

            mock_client.set.assert_called_once()

    @pytest.mark.asyncio
    async def test_redis_delete(self):
        """Redis delete works correctly."""
        with patch("redis.from_url") as mock_redis:
            mock_client = Mock()
            mock_client.delete = Mock()
            mock_redis.return_value = mock_client

            memory = AgentMemory(backend="redis")
            await memory.delete("key1")

            mock_client.delete.assert_called_once_with("key1")

    @pytest.mark.asyncio
    async def test_redis_list_keys(self):
        """Redis list_keys works correctly."""
        with patch("redis.from_url") as mock_redis:
            mock_client = Mock()
            mock_client.scan_iter = Mock(return_value=["key1", "key2"])
            mock_redis.return_value = mock_client

            memory = AgentMemory(backend="redis")
            keys = await memory.list_keys()

            assert keys == ["key1", "key2"]


class TestSafetyLevel:
    """Test SafetyLevel enum."""

    def test_safety_levels_exist(self):
        """Safety levels are defined."""
        assert SafetyLevel.SAFE is not None
        assert SafetyLevel.NORMAL is not None
        assert SafetyLevel.DANGEROUS is not None

    def test_safety_level_values(self):
        """Safety levels have correct values."""
        assert SafetyLevel.SAFE.value == "safe"
        assert SafetyLevel.NORMAL.value == "normal"
        assert SafetyLevel.DANGEROUS.value == "dangerous"


class TestSafetyManager:
    """Test SafetyManager class."""

    @pytest.fixture
    def manager(self):
        """Create a SafetyManager instance."""
        return SafetyManager()

    def test_manager_creation(self, manager):
        """Manager can be created."""
        assert manager is not None
        assert manager.policies == {}
        assert manager.pending == {}
        assert manager.emergency_stop is False

    def test_register_policy(self, manager):
        """Policy can be registered."""
        manager.register_policy(
            "emergency_stop",
            SafetyLevel.DANGEROUS,
            "Emergency stop - requires confirmation",
        )

        assert "emergency_stop" in manager.policies
        policy = manager.policies["emergency_stop"]
        assert policy.level == SafetyLevel.DANGEROUS
        assert policy.description == "Emergency stop - requires confirmation"

    def test_register_policy_with_timeout(self, manager):
        """Policy can be registered with custom timeout."""
        manager.register_policy(
            "move_arm",
            SafetyLevel.DANGEROUS,
            "Arm movement",
            timeout=60,
        )

        assert manager.policies["move_arm"].timeout_seconds == 60

    def test_requires_confirmation_dangerous(self, manager):
        """Dangerous actions require confirmation."""
        manager.register_policy("move_arm", SafetyLevel.DANGEROUS, "Arm movement")

        assert manager.requires_confirmation("move_arm") is True

    def test_requires_confirmation_safe(self, manager):
        """Safe actions don't require confirmation."""
        manager.register_policy("get_status", SafetyLevel.SAFE, "Get status")

        assert manager.requires_confirmation("get_status") is False

    def test_requires_confirmation_normal(self, manager):
        """Normal actions don't require confirmation."""
        manager.register_policy("list_topics", SafetyLevel.NORMAL, "List topics")

        assert manager.requires_confirmation("list_topics") is False

    def test_requires_confirmation_unknown(self, manager):
        """Unknown actions don't require confirmation by default."""
        assert manager.requires_confirmation("unknown_action") is False

    def test_requires_confirmation_during_emergency(self, manager):
        """All actions require confirmation during emergency stop."""
        manager.register_policy("get_status", SafetyLevel.SAFE, "Get status")
        manager.trigger_emergency_stop("Test")

        assert manager.requires_confirmation("get_status") is True

    def test_get_safety_level(self, manager):
        """Safety level can be retrieved."""
        manager.register_policy("move_arm", SafetyLevel.DANGEROUS, "Arm movement")

        level = manager.get_safety_level("move_arm")

        assert level == SafetyLevel.DANGEROUS

    def test_get_safety_level_unknown(self, manager):
        """Unknown actions default to SAFE."""
        level = manager.get_safety_level("unknown")

        assert level == SafetyLevel.SAFE

    @pytest.mark.asyncio
    async def test_request_confirmation(self, manager):
        """Confirmation can be requested."""
        request = await manager.request_confirmation(
            action="move_arm",
            message="Move the arm?",
            timeout=30,
        )

        assert request is not None
        assert request.action == "move_arm"
        assert request.message == "Move the arm?"
        assert request.timeout == 30
        assert request.resolved is False

    @pytest.mark.asyncio
    async def test_confirm(self, manager):
        """Request can be confirmed."""
        request = await manager.request_confirmation(
            action="move_arm",
            message="Move the arm?",
        )

        result = await manager.confirm(request.id)

        assert result is True
        assert request.resolved is True
        assert request.approved is True

    @pytest.mark.asyncio
    async def test_confirm_not_found(self, manager):
        """Confirming unknown request returns False."""
        result = await manager.confirm("unknown_id")

        assert result is False

    @pytest.mark.asyncio
    async def test_reject(self, manager):
        """Request can be rejected."""
        request = await manager.request_confirmation(
            action="move_arm",
            message="Move the arm?",
        )

        result = await manager.reject(request.id)

        assert result is True
        assert request.resolved is True
        assert request.approved is False

    @pytest.mark.asyncio
    async def test_reject_not_found(self, manager):
        """Rejecting unknown request returns False."""
        result = await manager.reject("unknown_id")

        assert result is False

    @pytest.mark.asyncio
    async def test_wait_for_confirmation_approved(self, manager):
        """Wait for confirmation returns True when approved."""
        request = await manager.request_confirmation(
            action="move_arm",
            message="Move the arm?",
            timeout=30,
        )

        # Simulate approval
        await manager.confirm(request.id)

        result = await manager.wait_for_confirmation(request.id, timeout=1)

        assert result is True

    @pytest.mark.asyncio
    async def test_wait_for_confirmation_rejected(self, manager):
        """Wait for confirmation returns False when rejected."""
        request = await manager.request_confirmation(
            action="move_arm",
            message="Move the arm?",
            timeout=30,
        )

        # Simulate rejection
        await manager.reject(request.id)

        result = await manager.wait_for_confirmation(request.id, timeout=1)

        assert result is False

    @pytest.mark.asyncio
    async def test_wait_for_confirmation_timeout(self, manager):
        """Wait for confirmation returns False on timeout."""
        request = await manager.request_confirmation(
            action="move_arm",
            message="Move the arm?",
            timeout=30,
        )

        # Don't approve, let it timeout
        result = await manager.wait_for_confirmation(request.id, timeout=0.05)

        assert result is False
        assert request.resolved is True  # Auto-rejected

    @pytest.mark.asyncio
    async def test_wait_for_confirmation_not_found(self, manager):
        """Waiting for invalid request ID returns False."""
        result = await manager.wait_for_confirmation("invalid_id", timeout=0.1)

        assert result is False

    def test_trigger_emergency_stop(self, manager):
        """Emergency stop can be triggered."""
        manager.trigger_emergency_stop("Test emergency stop")

        assert manager.emergency_stop is True

    def test_clear_emergency_stop(self, manager):
        """Emergency stop can be cleared."""
        manager.trigger_emergency_stop("Test")
        manager.clear_emergency_stop()

        assert manager.emergency_stop is False

    def test_get_audit_log(self, manager):
        """Audit log can be retrieved."""
        manager.trigger_emergency_stop("Test")

        log = manager.get_audit_log()

        assert len(log) == 1
        assert log[0]["type"] == "emergency_stop"

    def test_on_confirmation_request_callback(self, manager):
        """Callback is triggered on confirmation request."""
        callback_called = False
        received_request = None

        def callback(request):
            nonlocal callback_called, received_request
            callback_called = True
            received_request = request

        manager.on_confirmation_request(callback)

        # Create request
        import asyncio
        asyncio.run(manager.request_confirmation("action", "message"))

        assert callback_called is True
        assert received_request is not None


class TestSafetyManagerEdgeCases:
    """Test SafetyManager edge cases."""

    def test_register_duplicate_policy(self):
        """Registering duplicate policy overwrites."""
        manager = SafetyManager()

        manager.register_policy("action", SafetyLevel.SAFE, "Original")
        manager.register_policy("action", SafetyLevel.DANGEROUS, "Updated")

        policy = manager.policies["action"]
        assert policy.level == SafetyLevel.DANGEROUS
        assert policy.description == "Updated"

    def test_audit_log_is_copied(self):
        """get_audit_log returns a copy."""
        manager = SafetyManager()
        manager.trigger_emergency_stop("Test")

        log1 = manager.get_audit_log()
        log1.append({"type": "tampered"})

        log2 = manager.get_audit_log()
        assert len(log2) == 1  # Original unchanged
