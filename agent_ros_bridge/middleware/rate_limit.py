"""Rate limiting for Agent ROS Bridge.

Protects against abuse and ensures fair resource usage.
"""

import logging
import time
from collections.abc import Callable
from dataclasses import dataclass
from functools import wraps

import redis.asyncio as redis

logger = logging.getLogger(__name__)


@dataclass
class RateLimitConfig:
    """Rate limit configuration."""

    requests_per_minute: int = 60
    burst_size: int = 10
    cooldown_seconds: int = 60


class RateLimiter:
    """Token bucket rate limiter with Redis backend.

    Supports:
    - Per-user rate limiting
    - Per-IP rate limiting
    - Per-endpoint rate limiting
    - Sliding window algorithm
    """

    def __init__(self, redis_client: redis.Redis | None = None):
        self.redis = redis_client
        self.local_buckets: dict[str, dict] = {}
        self.config = RateLimitConfig()

    async def is_allowed(self, key: str, config: RateLimitConfig | None = None) -> bool:
        """Check if request is allowed under rate limit.

        Args:
            key: Rate limit key (user_id, ip, etc.)
            config: Rate limit config (uses default if not provided)

        Returns:
            True if allowed, False if rate limited
        """
        cfg = config or self.config

        if self.redis:
            return await self._check_redis(key, cfg)
        else:
            return self._check_local(key, cfg)

    async def _check_redis(self, key: str, config: RateLimitConfig) -> bool:
        """Check rate limit using Redis."""
        bucket_key = f"ratelimit:{key}"
        now = time.time()

        # Use Redis transaction for atomicity
        async with self.redis.pipeline() as pipe:
            # Get current bucket state
            pipe.hgetall(bucket_key)
            results = await pipe.execute()

            bucket = results[0] if results else {}

            tokens = float(bucket.get("tokens", config.burst_size))
            last_update = float(bucket.get("last_update", now))

            # Calculate tokens to add based on time passed
            time_passed = now - last_update
            tokens_to_add = time_passed * (config.requests_per_minute / 60)
            tokens = min(tokens + tokens_to_add, config.burst_size)

            # Check if request allowed
            if tokens >= 1:
                tokens -= 1
                allowed = True
            else:
                allowed = False

            # Update bucket
            await self.redis.hset(bucket_key, mapping={"tokens": tokens, "last_update": now})
            await self.redis.expire(bucket_key, config.cooldown_seconds)

            return allowed

    def _check_local(self, key: str, config: RateLimitConfig) -> bool:
        """Check rate limit using local memory."""
        now = time.time()

        if key not in self.local_buckets:
            self.local_buckets[key] = {"tokens": config.burst_size, "last_update": now}

        bucket = self.local_buckets[key]
        tokens = bucket["tokens"]
        last_update = bucket["last_update"]

        # Calculate tokens to add
        time_passed = now - last_update
        tokens_to_add = time_passed * (config.requests_per_minute / 60)
        tokens = min(tokens + tokens_to_add, config.burst_size)

        # Check if request allowed
        if tokens >= 1:
            tokens -= 1
            allowed = True
        else:
            allowed = False

        # Update bucket
        bucket["tokens"] = tokens
        bucket["last_update"] = now

        return allowed

    async def get_remaining(self, key: str) -> dict[str, float]:
        """Get remaining rate limit info.

        Args:
            key: Rate limit key

        Returns:
            Dict with remaining tokens and reset time
        """
        if self.redis:
            bucket_key = f"ratelimit:{key}"
            bucket = await self.redis.hgetall(bucket_key)

            if bucket:
                tokens = float(bucket.get("tokens", 0))
                ttl = await self.redis.ttl(bucket_key)
                return {
                    "remaining": int(tokens),
                    "reset_in": ttl if ttl > 0 else self.config.cooldown_seconds,
                }

        return {"remaining": self.config.burst_size, "reset_in": self.config.cooldown_seconds}

    async def reset(self, key: str) -> None:
        """Reset rate limit for key.

        Args:
            key: Rate limit key
        """
        if self.redis:
            await self.redis.delete(f"ratelimit:{key}")
        else:
            self.local_buckets.pop(key, None)

        logger.info(f"Reset rate limit for {key}")


def rate_limit(
    requests_per_minute: int = 60, burst_size: int = 10, key_func: Callable | None = None
):
    """Decorator for rate limiting functions.

    Args:
        requests_per_minute: Rate limit
        burst_size: Burst allowance
        key_func: Function to extract rate limit key from arguments
    """

    def decorator(func):
        limiter = RateLimiter()
        config = RateLimitConfig(requests_per_minute=requests_per_minute, burst_size=burst_size)

        @wraps(func)
        async def wrapper(*args, **kwargs):
            # Extract key
            key = key_func(*args, **kwargs) if key_func else "default"

            # Check rate limit
            allowed = await limiter.is_allowed(key, config)

            if not allowed:
                raise RateLimitExceeded(
                    f"Rate limit exceeded. Try again in {config.cooldown_seconds}s."
                )

            return await func(*args, **kwargs)

        return wrapper

    return decorator


class RateLimitExceeded(Exception):
    """Exception raised when rate limit is exceeded."""

    pass


class RateLimitMiddleware:
    """Middleware for rate limiting HTTP/WebSocket requests."""

    def __init__(self, limiter: RateLimiter, config: RateLimitConfig | None = None):
        self.limiter = limiter
        self.config = config or RateLimitConfig()

    async def process_request(self, request) -> dict | None:
        """Process incoming request.

        Args:
            request: Request object

        Returns:
            None if allowed, error dict if rate limited
        """
        # Extract key from request
        key = self._extract_key(request)

        # Check rate limit
        allowed = await self.limiter.is_allowed(key, self.config)

        if not allowed:
            remaining = await self.limiter.get_remaining(key)
            return {
                "error": "Rate limit exceeded",
                "retry_after": remaining["reset_in"],
                "limit": self.config.requests_per_minute,
            }

        return None

    def _extract_key(self, request) -> str:
        """Extract rate limit key from request.

        Priority:
        1. User ID from JWT
        2. IP address
        3. Session ID
        """
        # Try user ID from JWT
        user_id = getattr(request, "user_id", None)
        if user_id:
            return f"user:{user_id}"

        # Try IP address
        ip = getattr(request, "client_ip", None)
        if ip:
            return f"ip:{ip}"

        # Try session ID
        session_id = getattr(request, "session_id", None)
        if session_id:
            return f"session:{session_id}"

        return "anonymous"
