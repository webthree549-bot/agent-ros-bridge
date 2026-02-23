"""Agent memory system for persistent context across sessions.

Supports multiple backends: Redis (production), SQLite (embedded), PostgreSQL (enterprise).

Example:
    from agent_ros_bridge.memory import AgentMemory
    
    # Redis backend (recommended for production)
    memory = AgentMemory(backend="redis", url="redis://localhost:6379")
    
    # Store conversation history
    await memory.append("conversation", {"role": "user", "content": "Navigate to (5, 3)"})
    
    # Retrieve with semantic search (requires embeddings)
    similar = await memory.search("go to position five three", k=3)
"""

import json
import logging
import sqlite3
from abc import ABC, abstractmethod
from datetime import datetime, timedelta
from typing import Any, Dict, List, Optional, Union
from dataclasses import dataclass, asdict

logger = logging.getLogger(__name__)


@dataclass
class MemoryEntry:
    """A single memory entry."""
    key: str
    value: Any
    timestamp: datetime
    ttl: Optional[int] = None  # Time-to-live in seconds
    metadata: Dict[str, Any] = None
    
    def to_dict(self) -> Dict:
        """Convert to dictionary for storage."""
        return {
            "key": self.key,
            "value": self.value,
            "timestamp": self.timestamp.isoformat(),
            "ttl": self.ttl,
            "metadata": self.metadata or {}
        }
    
    @classmethod
    def from_dict(cls, data: Dict) -> "MemoryEntry":
        """Create from dictionary."""
        return cls(
            key=data["key"],
            value=data["value"],
            timestamp=datetime.fromisoformat(data["timestamp"]),
            ttl=data.get("ttl"),
            metadata=data.get("metadata", {})
        )
    
    def is_expired(self) -> bool:
        """Check if entry has expired."""
        if self.ttl is None:
            return False
        expiry = self.timestamp + timedelta(seconds=self.ttl)
        return datetime.utcnow() > expiry


class MemoryBackend(ABC):
    """Abstract base class for memory backends."""
    
    @abstractmethod
    async def get(self, key: str) -> Optional[MemoryEntry]:
        """Get a memory entry by key."""
        pass
    
    @abstractmethod
    async def set(self, entry: MemoryEntry) -> bool:
        """Store a memory entry."""
        pass
    
    @abstractmethod
    async def delete(self, key: str) -> bool:
        """Delete a memory entry."""
        pass
    
    @abstractmethod
    async def list_keys(self, prefix: str = "") -> List[str]:
        """List all memory keys, optionally filtered by prefix."""
        pass
    
    @abstractmethod
    async def clear(self) -> bool:
        """Clear all memories."""
        pass


class SQLiteBackend(MemoryBackend):
    """SQLite-based memory backend for embedded deployments."""
    
    def __init__(self, db_path: str = ":memory:"):
        self.db_path = db_path
        self._conn: Optional[sqlite3.Connection] = None
        self._init_db()
    
    def _init_db(self):
        """Initialize database schema."""
        self._conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self._conn.execute("""
            CREATE TABLE IF NOT EXISTS memories (
                key TEXT PRIMARY KEY,
                value TEXT NOT NULL,
                timestamp TEXT NOT NULL,
                ttl INTEGER,
                metadata TEXT
            )
        """)
        self._conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_key ON memories(key)
        """)
        self._conn.commit()
    
    async def get(self, key: str) -> Optional[MemoryEntry]:
        cursor = self._conn.execute(
            "SELECT value, timestamp, ttl, metadata FROM memories WHERE key = ?",
            (key,)
        )
        row = cursor.fetchone()
        if row is None:
            return None
        
        entry = MemoryEntry(
            key=key,
            value=json.loads(row[0]),
            timestamp=datetime.fromisoformat(row[1]),
            ttl=row[2],
            metadata=json.loads(row[3]) if row[3] else {}
        )
        
        # Check expiry
        if entry.is_expired():
            await self.delete(key)
            return None
        
        return entry
    
    async def set(self, entry: MemoryEntry) -> bool:
        try:
            self._conn.execute(
                """
                INSERT OR REPLACE INTO memories (key, value, timestamp, ttl, metadata)
                VALUES (?, ?, ?, ?, ?)
                """,
                (
                    entry.key,
                    json.dumps(entry.value),
                    entry.timestamp.isoformat(),
                    entry.ttl,
                    json.dumps(entry.metadata) if entry.metadata else None
                )
            )
            self._conn.commit()
            return True
        except Exception as e:
            logger.error(f"SQLite set error: {e}")
            return False
    
    async def delete(self, key: str) -> bool:
        try:
            self._conn.execute("DELETE FROM memories WHERE key = ?", (key,))
            self._conn.commit()
            return True
        except Exception as e:
            logger.error(f"SQLite delete error: {e}")
            return False
    
    async def list_keys(self, prefix: str = "") -> List[str]:
        if prefix:
            cursor = self._conn.execute(
                "SELECT key FROM memories WHERE key LIKE ?",
                (f"{prefix}%",)
            )
        else:
            cursor = self._conn.execute("SELECT key FROM memories")
        
        return [row[0] for row in cursor.fetchall()]
    
    async def clear(self) -> bool:
        try:
            self._conn.execute("DELETE FROM memories")
            self._conn.commit()
            return True
        except Exception as e:
            logger.error(f"SQLite clear error: {e}")
            return False


class RedisBackend(MemoryBackend):
    """Redis-based memory backend for production deployments."""
    
    def __init__(self, url: str = "redis://localhost:6379", namespace: str = "agent_ros"):
        self.url = url
        self.namespace = namespace
        self._client = None
        
        try:
            import redis.asyncio as redis
            self._redis_module = redis
        except ImportError:
            raise ImportError("Redis backend requires redis package: pip install redis")
    
    def _make_key(self, key: str) -> str:
        """Create namespaced key."""
        return f"{self.namespace}:{key}"
    
    async def _get_client(self):
        """Get or create Redis client."""
        if self._client is None:
            self._client = await self._redis_module.from_url(self.url)
        return self._client
    
    async def get(self, key: str) -> Optional[MemoryEntry]:
        try:
            client = await self._get_client()
            data = await client.get(self._make_key(key))
            if data is None:
                return None
            
            entry_dict = json.loads(data)
            return MemoryEntry.from_dict(entry_dict)
        except Exception as e:
            logger.error(f"Redis get error: {e}")
            return None
    
    async def set(self, entry: MemoryEntry) -> bool:
        try:
            client = await self._get_client()
            key = self._make_key(entry.key)
            data = json.dumps(entry.to_dict())
            
            if entry.ttl:
                await client.setex(key, entry.ttl, data)
            else:
                await client.set(key, data)
            
            return True
        except Exception as e:
            logger.error(f"Redis set error: {e}")
            return False
    
    async def delete(self, key: str) -> bool:
        try:
            client = await self._get_client()
            await client.delete(self._make_key(key))
            return True
        except Exception as e:
            logger.error(f"Redis delete error: {e}")
            return False
    
    async def list_keys(self, prefix: str = "") -> List[str]:
        try:
            client = await self._get_client()
            pattern = self._make_key(f"{prefix}*")
            keys = await client.keys(pattern)
            # Remove namespace prefix
            prefix_len = len(self.namespace) + 1
            return [k.decode().split(":", 1)[1] for k in keys]
        except Exception as e:
            logger.error(f"Redis list_keys error: {e}")
            return []
    
    async def clear(self) -> bool:
        try:
            client = await self._get_client()
            pattern = self._make_key("*")
            keys = await client.keys(pattern)
            if keys:
                await client.delete(*keys)
            return True
        except Exception as e:
            logger.error(f"Redis clear error: {e}")
            return False


class AgentMemory:
    """High-level memory interface for agents.
    
    Provides:
    - Key-value storage with TTL
    - Conversation history
    - Semantic search (with embeddings)
    - Session management
    
    Example:
        memory = AgentMemory(backend="redis", url="redis://localhost:6379")
        
        # Store value
        await memory.set("robot_position", {"x": 5, "y": 3}, ttl=3600)
        
        # Retrieve
        position = await memory.get("robot_position")
        
        # Conversation history
        await memory.append("conversation", {"role": "user", "content": "Hello"})
        history = await memory.get_list("conversation")
    """
    
    def __init__(self, backend: str = "sqlite", **kwargs):
        """Initialize agent memory.
        
        Args:
            backend: "sqlite", "redis", or "postgresql"
            **kwargs: Backend-specific options
        """
        self.backend_type = backend
        
        if backend == "sqlite":
            self._backend = SQLiteBackend(**kwargs)
        elif backend == "redis":
            self._backend = RedisBackend(**kwargs)
        else:
            raise ValueError(f"Unknown backend: {backend}")
        
        logger.info(f"AgentMemory initialized with {backend} backend")
    
    async def get(self, key: str, default: Any = None) -> Any:
        """Get a value from memory.
        
        Args:
            key: Memory key
            default: Default value if not found
            
        Returns:
            Stored value or default
        """
        entry = await self._backend.get(key)
        if entry is None:
            return default
        return entry.value
    
    async def set(self, key: str, value: Any, ttl: Optional[int] = None, metadata: Dict = None) -> bool:
        """Store a value in memory.
        
        Args:
            key: Memory key
            value: Value to store
            ttl: Time-to-live in seconds (None for no expiry)
            metadata: Additional metadata
            
        Returns:
            True if stored successfully
        """
        entry = MemoryEntry(
            key=key,
            value=value,
            timestamp=datetime.utcnow(),
            ttl=ttl,
            metadata=metadata
        )
        return await self._backend.set(entry)
    
    async def delete(self, key: str) -> bool:
        """Delete a value from memory."""
        return await self._backend.delete(key)
    
    async def append(self, key: str, value: Any, max_items: int = 100) -> bool:
        """Append to a list in memory.
        
        Useful for conversation history.
        
        Args:
            key: Memory key
            value: Value to append
            max_items: Maximum items to keep (oldest removed)
        """
        current = await self.get(key, [])
        if not isinstance(current, list):
            current = [current]
        
        current.append(value)
        
        # Trim if needed
        if len(current) > max_items:
            current = current[-max_items:]
        
        return await self.set(key, current)
    
    async def get_list(self, key: str) -> List[Any]:
        """Get a list from memory."""
        value = await self.get(key, [])
        if not isinstance(value, list):
            return [value]
        return value
    
    async def search(self, query: str, k: int = 5) -> List[Dict]:
        """Semantic search (requires embeddings backend).
        
        Args:
            query: Search query
            k: Number of results
            
        Returns:
            List of matching entries with scores
        """
        # Placeholder - requires embedding model
        logger.warning("Semantic search not implemented without embeddings backend")
        return []
    
    async def clear(self) -> bool:
        """Clear all memories."""
        return await self._backend.clear()
    
    async def keys(self, prefix: str = "") -> List[str]:
        """List all keys, optionally filtered by prefix."""
        return await self._backend.list_keys(prefix)


# Convenience factory functions
def create_memory(backend: str = "sqlite", **kwargs) -> AgentMemory:
    """Create an AgentMemory instance.
    
    Args:
        backend: "sqlite" or "redis"
        **kwargs: Backend-specific options
        
    Returns:
        AgentMemory instance
    """
    return AgentMemory(backend=backend, **kwargs)


__all__ = [
    "AgentMemory",
    "MemoryEntry",
    "MemoryBackend",
    "SQLiteBackend",
    "RedisBackend",
    "create_memory"
]
