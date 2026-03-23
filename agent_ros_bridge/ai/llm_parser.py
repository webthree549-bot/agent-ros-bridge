#!/usr/bin/env python3
"""
LLM Fallback for Intent Parser
Agent ROS Bridge v0.6.1 - Week 6 Advanced Features

Provides LLM-based intent parsing for complex utterances that rule-based
parsing cannot handle. Uses structured prompting for reliable output.

Features:
- OpenAI/Anthropic API integration
- Structured JSON output
- Confidence scoring
- Timeout handling
- Caching for common complex queries
"""

import hashlib
import json
import time
from dataclasses import dataclass
from typing import Any


@dataclass
class LLMIntentResult:
    """Result from LLM intent parsing."""

    intent_type: str
    confidence: float
    entities: list[dict[str, Any]]
    raw_response: str
    latency_ms: float
    cached: bool = False


class LLMIntentParser:
    """
    LLM-based intent parser for complex utterances.

    Uses structured prompting to get reliable JSON output from LLMs.
    Supports OpenAI GPT and Anthropic Claude models.
    """

    # System prompt for structured intent parsing
    SYSTEM_PROMPT = """You are a robot intent parsing system.
Parse the user's utterance into a structured intent format.

Available intent types:
- NAVIGATE: Move to a location ("go to kitchen", "drive to the office")
- MANIPULATE: Pick/place objects ("pick up the cup", "place book on table")
- SENSE: Perception tasks ("what do you see", "scan the area")
- QUERY: Status queries ("where are you", "battery level")
- SAFETY: Emergency commands ("stop", "emergency stop")
- CONFIGURE: System settings ("set speed to slow", "enable mode")
- UNKNOWN: Cannot determine intent

Respond ONLY with a JSON object in this exact format:
{
    "intent_type": "NAVIGATE|MANIPULATE|SENSE|QUERY|SAFETY|CONFIGURE|UNKNOWN",
    "confidence": 0.0-1.0,
    "entities": [
        {"type": "LOCATION|OBJECT|SPEED|MODE", "value": "extracted value"}
    ],
    "reasoning": "brief explanation of your decision"
}

Rules:
1. Confidence should reflect your certainty (0.0-1.0)
2. Extract all relevant entities
3. Use UNKNOWN only when truly uncertain
4. Be concise in reasoning"""

    # Provider configurations
    PROVIDER_CONFIGS = {
        "openai": {
            "env_key": "OPENAI_API_KEY",
            "base_url": None,
            "default_model": "gpt-3.5-turbo",
        },
        "anthropic": {
            "env_key": "ANTHROPIC_API_KEY",
            "base_url": None,
            "default_model": "claude-3-haiku-20240307",
        },
        "moonshot": {
            "env_key": "MOONSHOT_API_KEY",
            "base_url": "https://api.moonshot.cn/v1",
            "default_model": "kimi-k2.5",
        },
    }

    def __init__(
        self,
        api_key: str | None = None,
        model: str | None = None,
        provider: str = "openai",
        timeout_sec: float = 5.0,
        enable_cache: bool = True,
        cache_size: int = 500,
        rate_limit_calls: int = 100,
        rate_limit_window: int = 60,
    ):
        """
        Initialize LLM intent parser.

        Args:
            api_key: API key for LLM provider (loaded from env if None)
            model: Model name to use (provider default if None)
            provider: "openai", "anthropic", or "moonshot"
            timeout_sec: Timeout for LLM calls
            enable_cache: Whether to cache results
            cache_size: Maximum cache entries
            rate_limit_calls: Max API calls per window
            rate_limit_window: Rate limit window in seconds
        """
        # Import security utilities
        try:
            from ..security_utils import RateLimiter, SecureConfig, sanitize_input

            self._secure_config = SecureConfig
            self._sanitize = sanitize_input
            self._use_security = True
        except ImportError:
            self._use_security = False
            self._sanitize = lambda x, **kw: x

        # Normalize provider
        self._provider = provider.lower()

        # Get provider config
        provider_config = self.PROVIDER_CONFIGS.get(self._provider, self.PROVIDER_CONFIGS["openai"])

        # Load API key from environment if not provided
        if api_key is None:
            import os
            env_key = provider_config["env_key"]
            api_key = os.environ.get(env_key)

        # Use provider default model if not specified
        if model is None:
            model = provider_config["default_model"]

        self._api_key = api_key
        self._model = model
        self._base_url = provider_config.get("base_url")
        self._timeout_sec = timeout_sec
        self._enable_cache = enable_cache
        self._cache_size = cache_size

        # Initialize rate limiter
        if self._use_security:
            self._rate_limiter = RateLimiter(rate_limit_calls, rate_limit_window)
        else:
            self._rate_limiter = None

        # Cache: utterance_hash -> (result, timestamp)
        self._cache: dict[str, tuple] = {}
        self._cache_order: list[str] = []

        # Statistics
        self._calls = 0
        self._cache_hits = 0
        self._timeouts = 0
        self._errors = 0

        # Try to import LLM libraries
        self._openai_available = False
        self._anthropic_available = False

        try:
            import openai

            self._openai = openai
            self._openai_available = True
        except ImportError:
            pass

        try:
            import anthropic

            self._anthropic = anthropic
            self._anthropic_available = True
        except ImportError:
            pass

        # Validate API key format
        if self._api_key and self._use_security:
            if not SecureConfig.validate_api_key(self._api_key, self._provider):
                print(f"Warning: API key format doesn't match expected for {self._provider}")

    def is_available(self) -> bool:
        """Check if LLM parsing is available."""
        if not self._api_key:
            return False
        if self._provider in ("openai", "moonshot") and not self._openai_available:
            return False
        return not (self._provider == "anthropic" and not self._anthropic_available)

    def _compute_hash(self, utterance: str) -> str:
        """Compute hash for utterance."""
        return hashlib.md5(utterance.lower().strip().encode()).hexdigest()

    def _get_cached(self, utterance_hash: str) -> LLMIntentResult | None:
        """Get cached result if available."""
        if not self._enable_cache or utterance_hash not in self._cache:
            return None

        result, timestamp = self._cache[utterance_hash]
        # Cache TTL: 1 hour
        if time.time() - timestamp > 3600:
            del self._cache[utterance_hash]
            self._cache_order.remove(utterance_hash)
            return None

        self._cache_order.remove(utterance_hash)
        self._cache_order.append(utterance_hash)
        self._cache_hits += 1

        # Return copy with cached flag
        return LLMIntentResult(
            intent_type=result.intent_type,
            confidence=result.confidence,
            entities=result.entities,
            raw_response=result.raw_response,
            latency_ms=0.0,
            cached=True,
        )

    def _cache_result(self, utterance_hash: str, result: LLMIntentResult):
        """Cache parsing result."""
        if not self._enable_cache:
            return

        # Evict oldest if needed
        while len(self._cache) >= self._cache_size:
            oldest = self._cache_order.pop(0)
            if oldest in self._cache:
                del self._cache[oldest]

        self._cache[utterance_hash] = (result, time.time())
        self._cache_order.append(utterance_hash)

    def parse(
        self, utterance: str, context: dict[str, Any] | None = None
    ) -> LLMIntentResult | None:
        """
        Parse utterance using LLM.

        Args:
            utterance: Natural language input
            context: Optional context (robot state, environment)

        Returns:
            Parsed intent or None if unavailable/failed
        """
        if not self.is_available():
            return None

        # Check rate limit
        if self._rate_limiter and not self._rate_limiter.is_allowed():
            print("Rate limit exceeded. Try again later.")
            return None

        # Sanitize input
        utterance = self._sanitize(utterance, max_length=500)
        if not utterance:
            return None

        start_time = time.time()
        self._calls += 1

        # Check cache
        utterance_hash = self._compute_hash(utterance)
        cached = self._get_cached(utterance_hash)
        if cached:
            return cached

        try:
            if self._provider in ("openai", "moonshot"):
                # Moonshot uses OpenAI-compatible API
                result = self._call_openai(utterance, context)
            elif self._provider == "anthropic":
                result = self._call_anthropic(utterance, context)
            else:
                return None

            if result:
                result.latency_ms = (time.time() - start_time) * 1000
                self._cache_result(utterance_hash, result)

            return result

        except Exception as e:
            self._errors += 1
            print(f"LLM parsing error: {e}")
            return None

    def _call_openai(
        self, utterance: str, context: dict[str, Any] | None
    ) -> LLMIntentResult | None:
        """Call OpenAI-compatible API (OpenAI, Moonshot, etc.)."""
        import openai

        # Use base_url for providers like Moonshot
        client_kwargs = {"api_key": self._api_key}
        if self._base_url:
            client_kwargs["base_url"] = self._base_url

        client = openai.OpenAI(**client_kwargs)

        messages = [
            {"role": "system", "content": self.SYSTEM_PROMPT},
            {"role": "user", "content": f"Parse this utterance: '{utterance}'"},
        ]

        try:
            response = client.chat.completions.create(
                model=self._model,
                messages=messages,
                temperature=0.0,  # Deterministic
                max_tokens=200,
                timeout=self._timeout_sec,
            )

            content = response.choices[0].message.content
            return self._parse_llm_response(content)

        except Exception as e:
            if "timeout" in str(e).lower():
                self._timeouts += 1
            raise

    def _call_anthropic(
        self, utterance: str, context: dict[str, Any] | None
    ) -> LLMIntentResult | None:
        """Call Anthropic API."""
        import anthropic

        client = anthropic.Anthropic(api_key=self._api_key)

        try:
            response = client.messages.create(
                model=self._model,
                max_tokens=200,
                temperature=0.0,
                system=self.SYSTEM_PROMPT,
                messages=[{"role": "user", "content": utterance}],
            )

            content = response.content[0].text
            return self._parse_llm_response(content)

        except Exception as e:
            if "timeout" in str(e).lower():
                self._timeouts += 1
            raise

    def _parse_llm_response(self, response: str) -> LLMIntentResult | None:
        """Parse LLM JSON response."""
        try:
            # Extract JSON from response
            # Handle cases where LLM adds markdown or extra text
            json_start = response.find("{")
            json_end = response.rfind("}") + 1

            if json_start == -1 or json_end == 0:
                return None

            json_str = response[json_start:json_end]
            data = json.loads(json_str)

            # Validate required fields
            intent_type = data.get("intent_type")
            if not intent_type or intent_type == "UNKNOWN":
                # Empty or unknown intent - treat as parse failure
                if not data.get("entities") and not data.get("reasoning"):
                    return None

            return LLMIntentResult(
                intent_type=intent_type or "UNKNOWN",
                confidence=data.get("confidence", 0.5),
                entities=data.get("entities", []),
                raw_response=response,
                latency_ms=0.0,
                cached=False,
            )

        except json.JSONDecodeError as e:
            print(f"Failed to parse LLM response: {e}")
            self._errors += 1
            return None

    def get_statistics(self) -> dict[str, Any]:
        """Get parser statistics."""
        return {
            "calls": self._calls,
            "cache_hits": self._cache_hits,
            "cache_size": len(self._cache),
            "timeouts": self._timeouts,
            "errors": self._errors,
            "hit_rate": self._cache_hits / max(self._calls, 1),
            "available": self.is_available(),
            "provider": self._provider,
            "model": self._model,
        }


def main():
    """Test LLM parser."""
    import os

    # Get API key from environment
    api_key = os.getenv("OPENAI_API_KEY") or os.getenv("ANTHROPIC_API_KEY")

    if not api_key:
        print("Set OPENAI_API_KEY or ANTHROPIC_API_KEY environment variable")
        return

    # Create parser
    parser = LLMIntentParser(api_key=api_key, provider="openai", model="gpt-3.5-turbo")

    if not parser.is_available():
        print("LLM parser not available. Install openai or anthropic package.")
        return

    # Test utterances
    test_utterances = [
        "I need you to go to the kitchen and grab me a bottle of water",
        "Can you check if there's anyone in the living room?",
        "Move slowly to the charging station",
        "What is the current battery level and where are you located?",
    ]

    print("Testing LLM Intent Parser\n")

    for utterance in test_utterances:
        print(f"Utterance: '{utterance}'")

        result = parser.parse(utterance)

        if result:
            print(f"  Intent: {result.intent_type} (confidence: {result.confidence:.2f})")
            print(f"  Entities: {result.entities}")
            print(f"  Latency: {result.latency_ms:.2f}ms")
            print(f"  Cached: {result.cached}")
        else:
            print("  Failed to parse")

        print()

    # Print statistics
    stats = parser.get_statistics()
    print(f"Statistics: {stats}")


if __name__ == "__main__":
    main()
