#!/usr/bin/env python3
"""
Demo: LLM Intent Parsing with Moonshot (Kimi)

This demo shows how to use the LLM intent parser with Moonshot's Kimi model.

Setup:
    export MOONSHOT_API_KEY="your-api-key-here"
    pip install openai

Usage:
    python examples/demo_llm_moonshot.py
"""

import os
import sys

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from agent_ros_bridge.ai.llm_parser import LLMIntentParser


def main():
    """Run Moonshot LLM intent parsing demo."""
    # Check for API key
    api_key = os.environ.get("MOONSHOT_API_KEY")
    if not api_key:
        print("Error: MOONSHOT_API_KEY environment variable not set")
        print("\nTo get an API key:")
        print("  1. Visit https://platform.moonshot.cn/")
        print("  2. Create an account and get your API key")
        print("  3. Set the environment variable:")
        print("     export MOONSHOT_API_KEY='your-key-here'")
        sys.exit(1)

    # Initialize parser with Moonshot provider
    print("Initializing Moonshot LLM parser...")
    parser = LLMIntentParser(
        provider="moonshot",
        model="kimi-k2.5",  # or "kimi-k1.5", "kimi-k1.5-long"
        timeout_sec=10.0,
        enable_cache=True,
    )

    if not parser.is_available():
        print("Error: LLM parser not available. Make sure 'openai' package is installed.")
        print("  pip install openai")
        sys.exit(1)

    print("✓ Parser ready\n")

    # Test utterances
    test_utterances = [
        "Go to the kitchen",
        "Pick up the red cup from the table",
        "What do you see in front of you?",
        "Stop immediately!",
        "Navigate to the office at slow speed",
        "Can you grab that object over there?",
        "Tell me your battery level",
    ]

    print("Testing intent parsing with Moonshot Kimi:\n")
    print("=" * 60)

    for utterance in test_utterances:
        print(f"\nUser: '{utterance}'")
        result = parser.parse(utterance)

        if result:
            print(f"  Intent: {result.intent_type}")
            print(f"  Confidence: {result.confidence:.2f}")
            if result.entities:
                print(f"  Entities: {result.entities}")
            print(f"  Latency: {result.latency_ms:.1f}ms")
            if result.cached:
                print("  (cached)")
        else:
            print("  Failed to parse")

    print("\n" + "=" * 60)
    print("\nStatistics:")
    stats = parser.get_stats()
    print(f"  Total calls: {stats['calls']}")
    print(f"  Cache hits: {stats['cache_hits']}")
    print(f"  Errors: {stats['errors']}")
    print(f"  Timeouts: {stats['timeouts']}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
