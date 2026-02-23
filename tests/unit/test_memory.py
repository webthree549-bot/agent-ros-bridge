"""Unit tests for AgentMemory."""

import pytest
import asyncio

from agent_ros_bridge.memory import AgentMemory, MemoryEntry, SQLiteBackend


@pytest.mark.unit
class TestAgentMemory:
    """Test AgentMemory functionality."""
    
    @pytest.fixture
    def memory(self):
        """Create memory with SQLite backend."""
        return AgentMemory(backend="sqlite", db_path=":memory:")
    
    @pytest.mark.asyncio
    async def test_set_and_get(self, memory):
        """Test basic set and get."""
        await memory.set("key1", "value1")
        result = await memory.get("key1")
        assert result == "value1"
    
    @pytest.mark.asyncio
    async def test_get_default(self, memory):
        """Test get with default value."""
        result = await memory.get("nonexistent", "default")
        assert result == "default"
    
    @pytest.mark.asyncio
    async def test_set_with_ttl(self, memory):
        """Test set with TTL."""
        await memory.set("key1", "value1", ttl=1)
        
        # Should exist immediately
        result = await memory.get("key1")
        assert result == "value1"
        
        # Wait for expiry
        await asyncio.sleep(1.1)
        
        # Should be expired
        result = await memory.get("key1")
        assert result is None
    
    @pytest.mark.asyncio
    async def test_delete(self, memory):
        """Test delete."""
        await memory.set("key1", "value1")
        await memory.delete("key1")
        
        result = await memory.get("key1")
        assert result is None
    
    @pytest.mark.asyncio
    async def test_append_to_list(self, memory):
        """Test appending to list."""
        await memory.append("list", "item1")
        await memory.append("list", "item2")
        await memory.append("list", "item3")
        
        result = await memory.get_list("list")
        assert result == ["item1", "item2", "item3"]
    
    @pytest.mark.asyncio
    async def test_list_max_items(self, memory):
        """Test list max items limit."""
        for i in range(5):
            await memory.append("list", f"item{i}", max_items=3)
        
        result = await memory.get_list("list")
        assert len(result) == 3
        assert result == ["item2", "item3", "item4"]
    
    @pytest.mark.asyncio
    async def test_keys(self, memory):
        """Test listing keys."""
        await memory.set("prefix:key1", "value1")
        await memory.set("prefix:key2", "value2")
        await memory.set("other:key3", "value3")
        
        all_keys = await memory.keys()
        assert len(all_keys) == 3
        
        prefix_keys = await memory.keys("prefix:")
        assert len(prefix_keys) == 2
    
    @pytest.mark.asyncio
    async def test_clear(self, memory):
        """Test clear all."""
        await memory.set("key1", "value1")
        await memory.set("key2", "value2")
        
        await memory.clear()
        
        assert await memory.get("key1") is None
        assert await memory.get("key2") is None


@pytest.mark.unit
class TestMemoryEntry:
    """Test MemoryEntry dataclass."""
    
    def test_is_expired_no_ttl(self):
        """Test expiry check with no TTL."""
        from datetime import datetime
        entry = MemoryEntry(
            key="test",
            value="value",
            timestamp=datetime.utcnow(),
            ttl=None
        )
        assert not entry.is_expired()
    
    def test_is_expired_with_ttl(self):
        """Test expiry check with TTL."""
        from datetime import datetime, timedelta
        entry = MemoryEntry(
            key="test",
            value="value",
            timestamp=datetime.utcnow() - timedelta(seconds=10),
            ttl=5
        )
        assert entry.is_expired()
    
    def test_to_dict(self):
        """Test serialization to dict."""
        from datetime import datetime
        entry = MemoryEntry(
            key="test",
            value={"nested": "data"},
            timestamp=datetime(2024, 1, 1, 12, 0, 0),
            ttl=3600,
            metadata={"source": "test"}
        )
        
        data = entry.to_dict()
        assert data["key"] == "test"
        assert data["value"]["nested"] == "data"
        assert data["ttl"] == 3600
        assert data["metadata"]["source"] == "test"
