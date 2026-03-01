"""Agent Memory System - SQLite/Redis backends with TTL."""

import json
import logging
import sqlite3
from dataclasses import dataclass
from datetime import datetime, timedelta
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


@dataclass
class MemoryEntry:
    """Single memory entry."""

    key: str
    value: Any
    created_at: datetime
    expires_at: Optional[datetime] = None
    metadata: Dict[str, Any] = None

    def is_expired(self) -> bool:
        """Check if the memory entry has expired."""
        if self.expires_at is None:
            return False
        return datetime.now() > self.expires_at


class AgentMemory:
    """Agent memory with TTL support.

    Supports SQLite (default) and Redis (optional) backends.

    Example:
        memory = AgentMemory(backend="sqlite", path="memory.db")
        await memory.set("conversation", messages, ttl=3600)
        messages = await memory.get("conversation")
    """

    def __init__(self, backend: str = "sqlite", **kwargs):
        self.backend = backend
        self._init_backend(**kwargs)
        logger.info(f"AgentMemory initialized with {backend} backend")

    def _init_backend(self, **kwargs):
        if self.backend == "sqlite":
            self._init_sqlite(**kwargs)
        elif self.backend == "redis":
            self._init_redis(**kwargs)
        else:
            raise ValueError(f"Unknown backend: {self.backend}")

    def _init_sqlite(self, path: str = ":memory:"):
        self.conn = sqlite3.connect(path, check_same_thread=False)
        self._create_tables()

    def _create_tables(self):
        cursor = self.conn.cursor()
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS memories (
                key TEXT PRIMARY KEY,
                value TEXT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                expires_at TIMESTAMP,
                metadata TEXT
            )
        """)
        self.conn.commit()

    def _init_redis(self, url: str = "redis://localhost:6379"):
        try:
            import redis

            self.redis = redis.from_url(url, decode_responses=True)
        except ImportError as e:
            raise ImportError("redis package required for Redis backend") from e

    async def get(self, key: str) -> Optional[Any]:
        """Get value by key."""
        if self.backend == "sqlite":
            return await self._get_sqlite(key)
        return await self._get_redis(key)

    async def _get_sqlite(self, key: str) -> Optional[Any]:
        cursor = self.conn.cursor()
        cursor.execute("SELECT value, expires_at FROM memories WHERE key = ?", (key,))
        row = cursor.fetchone()

        if row is None:
            return None

        value, expires_at = row

        # Check expiration
        if expires_at:
            expires = datetime.fromisoformat(expires_at)
            if datetime.now() > expires:
                await self.delete(key)
                return None

        return json.loads(value)

    async def _get_redis(self, key: str) -> Optional[Any]:
        value = self.redis.get(key)
        if value is None:
            return None
        return json.loads(value)

    async def set(self, key: str, value: Any, ttl: Optional[int] = None):
        """Set value with optional TTL (seconds)."""
        if self.backend == "sqlite":
            await self._set_sqlite(key, value, ttl)
        else:
            await self._set_redis(key, value, ttl)

    async def _set_sqlite(self, key: str, value: Any, ttl: Optional[int]):
        expires_at = None
        if ttl:
            expires_at = datetime.now() + timedelta(seconds=ttl)

        cursor = self.conn.cursor()
        cursor.execute(
            """INSERT OR REPLACE INTO memories
               (key, value, expires_at, metadata)
               VALUES (?, ?, ?, ?)""",
            (
                key,
                json.dumps(value),
                expires_at.isoformat() if expires_at else None,
                json.dumps({}),
            ),
        )
        self.conn.commit()

    async def _set_redis(self, key: str, value: Any, ttl: Optional[int]):
        self.redis.set(key, json.dumps(value), ex=ttl)

    async def delete(self, key: str):
        """Delete a key."""
        if self.backend == "sqlite":
            cursor = self.conn.cursor()
            cursor.execute("DELETE FROM memories WHERE key = ?", (key,))
            self.conn.commit()
        else:
            self.redis.delete(key)

    async def append(self, key: str, value: Any):
        """Append to a list."""
        existing = await self.get(key)
        if existing is None:
            existing = []
        if not isinstance(existing, list):
            existing = [existing]
        existing.append(value)
        await self.set(key, existing)

    async def get_list(self, key: str) -> List[Any]:
        """Get value as list."""
        value = await self.get(key)
        if value is None:
            return []
        if isinstance(value, list):
            return value
        return [value]

    async def list_keys(self) -> List[str]:
        """List all keys in memory."""
        if self.backend == "sqlite":
            return await self._list_keys_sqlite()
        return await self._list_keys_redis()

    async def _list_keys_sqlite(self) -> List[str]:
        """List all keys from SQLite."""
        cursor = self.conn.cursor()
        cursor.execute("SELECT key, expires_at FROM memories")
        rows = cursor.fetchall()

        keys = []
        for key, expires_at in rows:
            # Skip expired entries
            if expires_at:
                expires = datetime.fromisoformat(expires_at)
                if datetime.now() > expires:
                    continue
            keys.append(key)
        return keys

    async def _list_keys_redis(self) -> List[str]:
        """List all keys from Redis."""
        return list(self.redis.scan_iter())
