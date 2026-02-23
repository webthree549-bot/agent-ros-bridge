"""Tests for AgentMemory."""

import pytest
import asyncio
from agent_ros_bridge.integrations.memory import AgentMemory


@pytest.fixture
def memory():
    """Create memory with SQLite backend."""
    return AgentMemory(backend="sqlite", path=":memory:")


class TestAgentMemory:
    """Test AgentMemory functionality."""
    
    @pytest.mark.asyncio
    async def test_set_and_get(self, memory):
        """Test basic set and get."""
        await memory.set("key1", "value1")
        result = await memory.get("key1")
        assert result == "value1"
    
    @pytest.mark.asyncio
    async def test_get_nonexistent(self, memory):
        """Test get returns None for missing key."""
        result = await memory.get("missing")
        assert result is None
    
    @pytest.mark.asyncio
    async def test_ttl_expiration(self, memory):
        """Test TTL expiration."""
        await memory.set("key", "value", ttl=1)
        
        # Should exist immediately
        result = await memory.get("key")
        assert result == "value"
        
        # Wait for expiration
        await asyncio.sleep(1.1)
        
        # Should be expired
        result = await memory.get("key")
        assert result is None
    
    @pytest.mark.asyncio
    async def test_append(self, memory):
        """Test append to list."""
        await memory.append("list", "item1")
        await memory.append("list", "item2")
        
        result = await memory.get_list("list")
        assert result == ["item1", "item2"]
    
    @pytest.mark.asyncio
    async def test_delete(self, memory):
        """Test delete."""
        await memory.set("key", "value")
        await memory.delete("key")
        
        result = await memory.get("key")
        assert result is None
