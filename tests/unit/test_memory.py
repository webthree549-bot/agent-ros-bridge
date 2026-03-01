"""
Unit tests for Agent Memory System.
Tests SQLite/Redis backends with TTL functionality.
Uses synchronous wrappers for async methods.
"""

import asyncio

# Check optional dependencies
import importlib.util
import json
import sqlite3
from datetime import datetime, timedelta
from unittest import mock

import pytest

from agent_ros_bridge.integrations.memory import AgentMemory, MemoryEntry

REDIS_AVAILABLE = importlib.util.find_spec("redis") is not None


def run_async(coro):
    """Helper to run async function synchronously"""
    return asyncio.run(coro)


class TestMemoryEntry:
    """Test MemoryEntry dataclass"""

    def test_memory_entry_creation(self):
        """Test basic memory entry creation"""
        entry = MemoryEntry(
            key="test_key",
            value={"data": "value"},
            created_at=datetime.now(),
            expires_at=None,
            metadata={"source": "test"},
        )

        assert entry.key == "test_key"
        assert entry.value == {"data": "value"}
        assert entry.metadata == {"source": "test"}
        assert entry.is_expired() is False

    def test_memory_entry_expired(self):
        """Test expired entry detection"""
        past = datetime.now() - timedelta(hours=1)
        entry = MemoryEntry(
            key="expired_key", value="data", created_at=datetime.now(), expires_at=past
        )

        assert entry.is_expired() is True

    def test_memory_entry_not_expired(self):
        """Test non-expired entry"""
        future = datetime.now() + timedelta(hours=1)
        entry = MemoryEntry(
            key="valid_key", value="data", created_at=datetime.now(), expires_at=future
        )

        assert entry.is_expired() is False

    def test_memory_entry_no_expiry(self):
        """Test entry without expiration never expires"""
        entry = MemoryEntry(
            key="permanent_key", value="data", created_at=datetime.now(), expires_at=None
        )

        assert entry.is_expired() is False


class TestAgentMemorySQLite:
    """Test AgentMemory with SQLite backend"""

    @pytest.fixture
    def memory(self):
        """Create in-memory SQLite backend"""
        mem = AgentMemory(backend="sqlite", path=":memory:")
        yield mem
        mem.conn.close()

    def test_set_and_get(self, memory):
        """Test basic set and get operations"""
        run_async(memory.set("key1", {"test": "value"}))
        result = run_async(memory.get("key1"))

        assert result == {"test": "value"}

    def test_get_nonexistent(self, memory):
        """Test getting non-existent key returns None"""
        result = run_async(memory.get("nonexistent"))

        assert result is None

    def test_update_existing(self, memory):
        """Test updating existing key"""
        run_async(memory.set("key1", "original"))
        run_async(memory.set("key1", "updated"))
        result = run_async(memory.get("key1"))

        assert result == "updated"

    def test_delete(self, memory):
        """Test delete operation"""
        run_async(memory.set("key1", "value"))
        run_async(memory.delete("key1"))
        result = run_async(memory.get("key1"))

        assert result is None

    def test_delete_nonexistent(self, memory):
        """Test deleting non-existent key doesn't error"""
        run_async(memory.delete("nonexistent"))  # Should not raise

    def test_list_keys(self, memory):
        """Test listing all keys"""
        run_async(memory.set("key1", "value1"))
        run_async(memory.set("key2", "value2"))
        run_async(memory.set("key3", "value3"))

        keys = run_async(memory.list_keys())

        assert sorted(keys) == ["key1", "key2", "key3"]

    def test_list_keys_empty(self, memory):
        """Test listing keys when empty"""
        keys = run_async(memory.list_keys())

        assert keys == []

    def test_ttl_expiration(self, memory):
        """Test TTL expiration"""
        # Set with very short TTL
        run_async(memory.set("temp", "value", ttl=0.001))  # 1ms

        # Should exist immediately
        result = run_async(memory.get("temp"))
        assert result == "value"

        # Wait for expiration
        import time

        time.sleep(0.01)

        # Should be expired now
        result = run_async(memory.get("temp"))
        assert result is None

    def test_complex_data_types(self, memory):
        """Test storing complex data types"""
        test_data = {
            "list": [1, 2, 3],
            "dict": {"nested": "value"},
            "tuple": (4, 5, 6),
            "datetime": datetime.now().isoformat(),
            "none": None,
            "bool": True,
        }

        run_async(memory.set("complex", test_data))
        result = run_async(memory.get("complex"))

        # Note: tuples become lists after JSON round-trip
        assert result["list"] == [1, 2, 3]
        assert result["dict"] == {"nested": "value"}
        assert result["tuple"] == [4, 5, 6]  # JSON converts tuples to lists
        assert result["none"] is None
        assert result["bool"] is True


@pytest.mark.skipif(not REDIS_AVAILABLE, reason="Redis not installed")
class TestAgentMemoryRedis:
    """Test AgentMemory with Redis backend (mocked)"""

    def test_redis_backend_init(self):
        """Test Redis backend initialization"""
        with mock.patch("agent_ros_bridge.integrations.memory.redis") as mock_redis_module:
            mock_client = mock.MagicMock()
            mock_redis_module.from_url.return_value = mock_client

            memory = AgentMemory(backend="redis", url="redis://localhost:6379")

            assert memory.backend == "redis"
            assert hasattr(memory, "redis")

    def test_redis_get(self):
        """Test Redis get operation"""
        with mock.patch("agent_ros_bridge.integrations.memory.redis") as mock_redis_module:
            mock_client = mock.MagicMock()
            mock_client.get.return_value = json.dumps({"test": "value"})
            mock_redis_module.from_url.return_value = mock_client

            memory = AgentMemory(backend="redis", url="redis://localhost:6379")
            result = run_async(memory.get("key1"))

            mock_client.get.assert_called_once_with("key1")
            assert result == {"test": "value"}

    def test_redis_get_none(self):
        """Test Redis get returns None for missing key"""
        with mock.patch("agent_ros_bridge.integrations.memory.redis") as mock_redis_module:
            mock_client = mock.MagicMock()
            mock_client.get.return_value = None
            mock_redis_module.from_url.return_value = mock_client

            memory = AgentMemory(backend="redis", url="redis://localhost:6379")
            result = run_async(memory.get("missing"))

            assert result is None

    def test_redis_set(self):
        """Test Redis set operation"""
        with mock.patch("agent_ros_bridge.integrations.memory.redis") as mock_redis_module:
            mock_client = mock.MagicMock()
            mock_redis_module.from_url.return_value = mock_client

            memory = AgentMemory(backend="redis", url="redis://localhost:6379")
            run_async(memory.set("key1", {"test": "value"}))

            mock_client.set.assert_called_once()
            call_args = mock_client.set.call_args
            assert call_args[0][0] == "key1"
            assert json.loads(call_args[0][1]) == {"test": "value"}

    def test_redis_set_with_ttl(self):
        """Test Redis set with TTL"""
        with mock.patch("agent_ros_bridge.integrations.memory.redis") as mock_redis_module:
            mock_client = mock.MagicMock()
            mock_redis_module.from_url.return_value = mock_client

            memory = AgentMemory(backend="redis", url="redis://localhost:6379")
            run_async(memory.set("key1", "value", ttl=3600))

            # Should be called with ex parameter for expiry
            mock_client.setex.assert_called_once()

    def test_redis_delete(self):
        """Test Redis delete operation"""
        with mock.patch("agent_ros_bridge.integrations.memory.redis") as mock_redis_module:
            mock_client = mock.MagicMock()
            mock_client.delete.return_value = 1
            mock_redis_module.from_url.return_value = mock_client

            memory = AgentMemory(backend="redis", url="redis://localhost:6379")
            run_async(memory.delete("key1"))

            mock_client.delete.assert_called_once_with("key1")

    def test_redis_list_keys(self):
        """Test Redis list keys"""
        with mock.patch("agent_ros_bridge.integrations.memory.redis") as mock_redis_module:
            mock_client = mock.MagicMock()
            mock_client.keys.return_value = ["key1", "key2"]
            mock_redis_module.from_url.return_value = mock_client

            memory = AgentMemory(backend="redis", url="redis://localhost:6379")
            keys = run_async(memory.list_keys())

            assert keys == ["key1", "key2"]


class TestAgentMemoryErrors:
    """Test error handling"""

    def test_unknown_backend(self):
        """Test unknown backend raises error"""
        with pytest.raises(ValueError, match="Unknown backend"):
            AgentMemory(backend="unknown")

    @mock.patch("agent_ros_bridge.integrations.memory.sqlite3.connect")
    def test_sqlite_connection_error(self, mock_connect):
        """Test SQLite connection error handling"""
        mock_connect.side_effect = sqlite3.Error("Connection failed")

        with pytest.raises(sqlite3.Error):
            AgentMemory(backend="sqlite", path="/invalid/path")


class TestAgentMemoryConcurrency:
    """Test concurrent access patterns"""

    @pytest.fixture
    def memory(self):
        """Create in-memory SQLite backend"""
        mem = AgentMemory(backend="sqlite", path=":memory:")
        yield mem
        mem.conn.close()

    def test_concurrent_reads(self, memory):
        """Test concurrent read operations"""
        run_async(memory.set("shared", {"data": "value"}))

        async def read_multiple():
            tasks = [memory.get("shared") for _ in range(10)]
            return await asyncio.gather(*tasks)

        results = run_async(read_multiple())

        assert all(r == {"data": "value"} for r in results)

    def test_concurrent_writes(self, memory):
        """Test concurrent write operations"""

        async def write_multiple():
            tasks = [memory.set(f"key_{i}", f"value_{i}") for i in range(10)]
            await asyncio.gather(*tasks)

        run_async(write_multiple())

        # Verify all writes succeeded
        keys = run_async(memory.list_keys())
        assert len(keys) == 10
