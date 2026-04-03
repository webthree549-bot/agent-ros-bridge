#!/usr/bin/env python3
"""Rate limiter for ROS2 topics.

Implements token bucket algorithm for smooth rate limiting.
Prevents overwhelming the WebSocket connection with high-frequency data.
"""

import asyncio
import logging
import sys
import time
from collections import defaultdict
from typing import Callable, Any

sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("rate_limiter")


class TokenBucket:
    """Token bucket for rate limiting."""
    
    def __init__(self, rate: float, burst: int = 1):
        """
        Initialize token bucket.
        
        Args:
            rate: Tokens per second (message rate limit)
            burst: Maximum burst size
        """
        self.rate = rate
        self.burst = burst
        self.tokens = burst
        self.last_update = time.monotonic()
    
    def consume(self, tokens: int = 1) -> bool:
        """Try to consume tokens. Returns True if allowed."""
        now = time.monotonic()
        elapsed = now - self.last_update
        self.last_update = now
        
        # Add tokens based on elapsed time
        self.tokens = min(self.burst, self.tokens + elapsed * self.rate)
        
        if self.tokens >= tokens:
            self.tokens -= tokens
            return True
        return False
    
    def get_wait_time(self, tokens: int = 1) -> float:
        """Get time to wait before tokens available."""
        if self.tokens >= tokens:
            return 0.0
        return (tokens - self.tokens) / self.rate


class TopicRateLimiter:
    """Rate limiter for ROS2 topics."""
    
    # Default rate limits (messages per second)
    DEFAULT_LIMITS = {
        '/tf': 10.0,           # 10 Hz for transforms
        '/odom': 10.0,         # 10 Hz for odometry
        '/scan': 5.0,          # 5 Hz for laser scans (downsample)
        '/camera/image_raw': 5.0,  # 5 Hz for video
        '/cmd_vel': 10.0,      # 10 Hz for commands
        '/map': 0.1,           # 0.1 Hz for map (static)
    }
    
    def __init__(self, limits: dict[str, float] | None = None):
        """
        Initialize rate limiter.
        
        Args:
            limits: Dict of topic_name -> max_rate (Hz)
        """
        self.limits = limits or self.DEFAULT_LIMITS
        self.buckets: dict[str, TokenBucket] = {}
        self.message_counts: defaultdict[str, int] = defaultdict(int)
        self.dropped_counts: defaultdict[str, int] = defaultdict(int)
        self.last_stats_time = time.monotonic()
        
        # Initialize buckets
        for topic, rate in self.limits.items():
            self.buckets[topic] = TokenBucket(rate=rate, burst=1)
        
        logger.info(f"Rate limiter initialized with {len(self.limits)} topics")
    
    def should_send(self, topic: str) -> bool:
        """Check if message should be sent for topic."""
        self.message_counts[topic] += 1
        
        # No limit for this topic
        if topic not in self.buckets:
            return True
        
        bucket = self.buckets[topic]
        
        if bucket.consume():
            return True
        else:
            self.dropped_counts[topic] += 1
            return False
    
    def get_stats(self) -> dict[str, dict]:
        """Get rate limiting statistics."""
        stats = {}
        for topic in self.limits:
            total = self.message_counts[topic]
            dropped = self.dropped_counts[topic]
            stats[topic] = {
                'total': total,
                'dropped': dropped,
                'rate_limit': self.limits[topic],
                'drop_rate': dropped / total if total > 0 else 0.0
            }
        return stats
    
    def print_stats(self):
        """Print rate limiting statistics."""
        stats = self.get_stats()
        logger.info("=== Rate Limiter Stats ===")
        for topic, stat in stats.items():
            logger.info(f"{topic}: {stat['total']} msgs, "
                       f"{stat['dropped']} dropped, "
                       f"limit={stat['rate_limit']}Hz")
    
    def set_limit(self, topic: str, rate: float):
        """Set rate limit for topic."""
        self.limits[topic] = rate
        self.buckets[topic] = TokenBucket(rate=rate, burst=1)
        logger.info(f"Set rate limit for {topic} to {rate} Hz")
    
    async def stats_reporter(self, interval: float = 60.0):
        """Periodically report stats."""
        while True:
            await asyncio.sleep(interval)
            self.print_stats()


class RateLimitedPublisher:
    """Wrapper for rate-limited message publishing."""
    
    def __init__(self, limiter: TopicRateLimiter):
        self.limiter = limiter
        self.callbacks: dict[str, list[Callable]] = defaultdict(list)
    
    def subscribe(self, topic: str, callback: Callable[[Any], None]):
        """Subscribe to rate-limited topic."""
        self.callbacks[topic].append(callback)
    
    def publish(self, topic: str, message: Any):
        """Publish message with rate limiting."""
        if self.limiter.should_send(topic):
            for callback in self.callbacks[topic]:
                try:
                    callback(message)
                except Exception as e:
                    logger.error(f"Callback error for {topic}: {e}")


# Global rate limiter instance
_global_limiter: TopicRateLimiter | None = None


def get_rate_limiter() -> TopicRateLimiter:
    """Get or create global rate limiter."""
    global _global_limiter
    if _global_limiter is None:
        _global_limiter = TopicRateLimiter()
    return _global_limiter


if __name__ == '__main__':
    # Test rate limiter
    limiter = TopicRateLimiter({
        '/test': 5.0  # 5 Hz limit
    })
    
    # Simulate messages at 20 Hz
    for i in range(40):
        allowed = limiter.should_send('/test')
        print(f"Message {i}: {'SENT' if allowed else 'DROPPED'}")
        time.sleep(0.05)  # 20 Hz
    
    limiter.print_stats()
