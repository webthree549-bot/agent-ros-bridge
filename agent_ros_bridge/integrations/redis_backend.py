"""Redis backend for context management.

Provides distributed, high-performance context storage.
"""

import json
import logging
from typing import Dict, Any, Optional, List
from datetime import datetime

import redis.asyncio as redis

logger = logging.getLogger(__name__)


class RedisContextBackend:
    """Redis backend for context storage.

    Provides:
    - Distributed context across multiple instances
    - High-performance read/write
    - TTL support for automatic expiration
    - Pub/sub for real-time updates
    """

    def __init__(self, redis_url: str = "redis://localhost:6379/0"):
        self.redis_url = redis_url
        self.client: Optional[redis.Redis] = None
        self._connected = False

    async def connect(self) -> None:
        """Connect to Redis."""
        try:
            self.client = await redis.from_url(
                self.redis_url, encoding="utf-8", decode_responses=True
            )
            await self.client.ping()
            self._connected = True
            logger.info(f"Connected to Redis at {self.redis_url}")
        except Exception as e:
            logger.error(f"Failed to connect to Redis: {e}")
            raise

    async def disconnect(self) -> None:
        """Disconnect from Redis."""
        if self.client:
            await self.client.close()
            self._connected = False
            logger.info("Disconnected from Redis")

    async def get_context(self, session_id: str) -> Optional[Dict[str, Any]]:
        """Get context for session.

        Args:
            session_id: Session identifier

        Returns:
            Context dict or None if not found
        """
        if not self._connected:
            raise RuntimeError("Not connected to Redis")

        key = f"context:{session_id}"
        data = await self.client.get(key)

        if data:
            return json.loads(data)
        return None

    async def save_context(self, session_id: str, context: Dict[str, Any], ttl: int = 3600) -> None:
        """Save context for session.

        Args:
            session_id: Session identifier
            context: Context data
            ttl: Time-to-live in seconds (default: 1 hour)
        """
        if not self._connected:
            raise RuntimeError("Not connected to Redis")

        key = f"context:{session_id}"
        data = json.dumps(context, default=str)

        await self.client.setex(key, ttl, data)
        logger.debug(f"Saved context for session {session_id} (TTL: {ttl}s)")

    async def delete_context(self, session_id: str) -> None:
        """Delete context for session.

        Args:
            session_id: Session identifier
        """
        if not self._connected:
            raise RuntimeError("Not connected to Redis")

        key = f"context:{session_id}"
        await self.client.delete(key)
        logger.debug(f"Deleted context for session {session_id}")

    async def log_interaction(
        self, session_id: str, command: str, interpretation: Dict, result: Dict, ttl: int = 86400
    ) -> None:
        """Log interaction to history.

        Args:
            session_id: Session identifier
            command: Natural language command
            interpretation: Interpreted command
            result: Execution result
            ttl: Time-to-live in seconds (default: 24 hours)
        """
        if not self._connected:
            raise RuntimeError("Not connected to Redis")

        key = f"history:{session_id}"
        entry = {
            "timestamp": datetime.now().isoformat(),
            "command": command,
            "interpretation": interpretation,
            "result": result,
        }

        # Add to list (keep last 100 entries)
        await self.client.lpush(key, json.dumps(entry, default=str))
        await self.client.ltrim(key, 0, 99)
        await self.client.expire(key, ttl)

        logger.debug(f"Logged interaction for session {session_id}")

    async def get_history(self, session_id: str, n: int = 10) -> List[Dict]:
        """Get recent interaction history.

        Args:
            session_id: Session identifier
            n: Number of entries to retrieve

        Returns:
            List of interaction entries
        """
        if not self._connected:
            raise RuntimeError("Not connected to Redis")

        key = f"history:{session_id}"
        entries = await self.client.lrange(key, 0, n - 1)

        # Parse JSON and reverse (newest first → oldest first)
        history = [json.loads(e) for e in entries]
        history.reverse()

        return history

    async def learn_location(
        self, session_id: str, name: str, coordinates: Dict[str, float]
    ) -> None:
        """Learn a named location.

        Args:
            session_id: Session identifier
            name: Location name
            coordinates: Coordinates dict
        """
        if not self._connected:
            raise RuntimeError("Not connected to Redis")

        key = f"locations:{session_id}"
        await self.client.hset(key, name, json.dumps(coordinates))
        logger.debug(f"Learned location '{name}' for session {session_id}")

    async def get_location(self, session_id: str, name: str) -> Optional[Dict]:
        """Get coordinates for a learned location.

        Args:
            session_id: Session identifier
            name: Location name

        Returns:
            Coordinates dict or None
        """
        if not self._connected:
            raise RuntimeError("Not connected to Redis")

        key = f"locations:{session_id}"
        data = await self.client.hget(key, name)

        if data:
            return json.loads(data)
        return None

    async def get_all_locations(self, session_id: str) -> Dict[str, Dict]:
        """Get all learned locations for session.

        Args:
            session_id: Session identifier

        Returns:
            Dict of location names to coordinates
        """
        if not self._connected:
            raise RuntimeError("Not connected to Redis")

        key = f"locations:{session_id}"
        locations = await self.client.hgetall(key)

        return {name: json.loads(coords) for name, coords in locations.items()}

    async def publish_update(self, channel: str, message: Dict) -> None:
        """Publish update to channel.

        Args:
            channel: Channel name
            message: Message dict
        """
        if not self._connected:
            raise RuntimeError("Not connected to Redis")

        await self.client.publish(channel, json.dumps(message, default=str))

    async def subscribe(self, channel: str):
        """Subscribe to channel for updates.

        Args:
            channel: Channel name

        Yields:
            Message dicts
        """
        if not self._connected:
            raise RuntimeError("Not connected to Redis")

        pubsub = self.client.pubsub()
        await pubsub.subscribe(channel)

        try:
            async for message in pubsub.listen():
                if message["type"] == "message":
                    yield json.loads(message["data"])
        finally:
            await pubsub.unsubscribe(channel)

    async def health_check(self) -> bool:
        """Check Redis health.

        Returns:
            True if healthy
        """
        if not self._connected:
            return False

        try:
            await self.client.ping()
            return True
        except Exception:
            return False

    async def get_stats(self) -> Dict[str, Any]:
        """Get Redis statistics.

        Returns:
            Stats dict
        """
        if not self._connected:
            raise RuntimeError("Not connected to Redis")

        info = await self.client.info()
        return {
            "connected_clients": info.get("connected_clients", 0),
            "used_memory_human": info.get("used_memory_human", "0B"),
            "total_commands_processed": info.get("total_commands_processed", 0),
            "keyspace_hits": info.get("keyspace_hits", 0),
            "keyspace_misses": info.get("keyspace_misses", 0),
        }
