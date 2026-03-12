#!/usr/bin/env python3
"""
Intent Parser Node - Week 2 Implementation.

ROS2 node for intent parsing with rule-based fast path + LLM fallback.

Features:
- Rule-based parsing (fast, deterministic, <10ms)
- LLM fallback (complex utterances, <100ms)
- Confidence scoring
- Entity extraction

Architecture:
    User Utterance → Rule-Based Parser → High Confidence? → Return Intent
                          ↓ No
                   LLM Fallback Parser → Return Intent
"""

import rclpy
from rclpy.node import Node
import re
import time
from typing import Optional, List, Dict, Any, Tuple

from agent_ros_bridge_msgs.srv import ParseIntent
from agent_ros_bridge_msgs.msg import Intent, Entity, Constraint


class IntentParserNode(Node):
    """
    Intent Parser Node - Week 2 Implementation.

    Parses natural language utterances into structured intent representations
    using a hybrid approach: rule-based fast path with LLM fallback.

    Services:
        /ai/parse_intent (ParseIntent): Parse utterance into Intent

    Performance:
        - Rule-based: < 10ms (95th percentile)
        - LLM fallback: < 100ms (with timeout)
    """

    # Performance targets
    TARGET_LATENCY_MS = 10.0  # 95th percentile target
    TARGET_CONFIDENCE = 0.95  # Minimum confidence for rule-based

    # Performance tracking
    _latency_history: List[float] = []
    _max_history_size = 1000

    # Intent patterns (rule-based)
    # Each pattern uses named groups for entity extraction
    PATTERNS: Dict[str, List[str]] = {
        "NAVIGATE": [
            r"go\s+to\s+(?:the\s+)?(?P<location>\w+)",
            r"navigate\s+to\s+(?:the\s+)?(?P<location>\w+)",
            r"move\s+to\s+(?:the\s+)?(?P<location>\w+)",
            r"drive\s+to\s+(?:the\s+)?(?P<location>\w+)",
            r"head\s+to\s+(?:the\s+)?(?P<location>\w+)",
            r"proceed\s+to\s+(?:the\s+)?(?P<location>\w+)",
        ],
        "MANIPULATE": [
            r"pick\s+up\s+(?:the\s+)?(?P<object>\w+)",
            r"grab\s+(?:the\s+)?(?P<object>\w+)",
            r"place\s+(?:the\s+)?(?P<object>\w+)",
            r"put\s+down\s+(?:the\s+)?(?P<object>\w+)",
            r"drop\s+(?:the\s+)?(?P<object>\w+)",
        ],
        "SENSE": [
            r"what\s+do\s+you\s+see",
            r"scan\s+(?:the\s+)?(?P<area>\w+)",
            r"look\s+(?:at\s+)?(?:the\s+)?(?P<object>\w+)",
            r"detect\s+(?:the\s+)?(?P<object>\w+)",
            r"observe\s+(?:the\s+)?(?P<area>\w+)",
        ],
        "QUERY": [
            r"where\s+are\s+you",
            r"what\s+is\s+your\s+status",
            r"battery\s+level",
            r"what\s+is\s+your\s+position",
            r"report\s+status",
        ],
        "SAFETY": [
            r"^stop$",
            r"emergency\s+stop",
            r"^halt$",
            r"^freeze$",
            r"^abort$",
        ],
        "CONFIGURE": [
            r"set\s+speed\s+to\s+(?P<speed>\w+)",
            r"set\s+mode\s+to\s+(?P<mode>\w+)",
            r"enable\s+(?P<feature>\w+)",
            r"disable\s+(?P<feature>\w+)",
        ],
    }

    # Compiled patterns (initialized in __init__)
    _compiled_patterns: Dict[str, List[re.Pattern]] = {}

    def __init__(self):
        super().__init__("intent_parser")

        # Compile regex patterns for performance
        self._compile_patterns()

        # Service for parsing intent
        self.parse_service = self.create_service(
            ParseIntent, "/ai/parse_intent", self.parse_intent_callback
        )

        # Performance monitoring timer (log stats every 60 seconds)
        self._perf_timer = self.create_timer(60.0, self._log_performance_stats)

        self.get_logger().info("Intent Parser Node initialized")
        self.get_logger().info(
            f"Performance target: <{self.TARGET_LATENCY_MS}ms latency, >{self.TARGET_CONFIDENCE} confidence"
        )

    def _compile_patterns(self):
        """Compile regex patterns for faster matching."""
        self._compiled_patterns = {}
        for intent_type, patterns in self.PATTERNS.items():
            self._compiled_patterns[intent_type] = [
                re.compile(pattern, re.IGNORECASE) for pattern in patterns
            ]

    def parse_intent_callback(
        self, request: ParseIntent.Request, response: ParseIntent.Response
    ) -> ParseIntent.Response:
        """
        Parse natural language utterance into structured intent.

        Strategy:
            1. Try rule-based parsing (fast, <10ms)
            2. If low confidence, try LLM fallback (<100ms)
            3. Return UNKNOWN if both fail

        Args:
            request: ParseIntent request with utterance and context
            response: ParseIntent response to populate

        Returns:
            Populated response with parsed intent
        """
        start_time = time.time()
        utterance = request.utterance.lower().strip()

        # Handle empty utterance
        if not utterance:
            response.intent = self._create_unknown_intent("", 0.0)
            response.success = True
            response.latency_ms = (time.time() - start_time) * 1000
            return response

        # Try rule-based parsing first
        intent = self._rule_based_parse(utterance)

        if intent.confidence >= 0.95:
            intent.source = "RULE_BASED"
        elif intent.confidence >= 0.70:
            # Medium confidence - could use LLM for confirmation
            intent.source = "RULE_BASED"
        else:
            # Try LLM fallback (Week 2 stretch goal)
            intent = self._llm_fallback_parse(utterance)
            intent.source = "LLM_ASSISTED"

        # Set latency
        intent.latency_ms = (time.time() - start_time) * 1000
        intent.timestamp = self.get_clock().now().to_msg()
        intent.robot_id = request.robot_id

        response.intent = intent
        response.success = True
        response.latency_ms = intent.latency_ms

        # Track performance
        self._track_latency(intent.latency_ms)

        # Warn if latency target exceeded
        if intent.latency_ms > self.TARGET_LATENCY_MS:
            self.get_logger().warn(
                f"Latency target exceeded: {intent.latency_ms:.2f}ms (target: <{self.TARGET_LATENCY_MS}ms)"
            )

        return response

    def _rule_based_parse(self, utterance: str) -> Intent:
        """
        Parse using regex patterns.

        Args:
            utterance: Lowercase utterance to parse

        Returns:
            Intent with type, confidence, and entities
        """
        intent = Intent()
        intent.type = "UNKNOWN"
        intent.confidence = 0.0
        intent.raw_utterance = utterance

        for intent_type, patterns in self._compiled_patterns.items():
            for pattern in patterns:
                match = pattern.search(utterance)
                if match:
                    intent.type = intent_type
                    intent.confidence = 0.95  # High confidence for pattern match

                    # Extract entities from named groups
                    for key, value in match.groupdict().items():
                        entity = Entity()
                        entity.type = self._map_entity_type(key)
                        entity.value = value
                        entity.confidence = 0.95
                        entity.resolved_from = value
                        intent.entities.append(entity)

                    return intent

        return intent

    def _llm_fallback_parse(self, utterance: str) -> Intent:
        """
        LLM fallback for complex utterances (v0.6.1 Week 6).

        Uses LLM-based parsing for utterances that rule-based parsing
        couldn't handle with high confidence.

        Args:
            utterance: Utterance that rule-based parsing couldn't handle

        Returns:
            Intent from LLM or UNKNOWN if LLM unavailable
        """
        intent = Intent()
        intent.raw_utterance = utterance
        intent.source = "LLM_ASSISTED"

        # Try LLM parsing
        try:
            from .llm_parser import LLMIntentParser

            # Initialize LLM parser (singleton pattern could be used here)
            llm_parser = LLMIntentParser()

            if llm_parser.is_available():
                result = llm_parser.parse(utterance)

                if result:
                    intent.type = result.intent_type
                    intent.confidence = result.confidence

                    # Convert entities
                    for entity_data in result.entities:
                        entity = Entity()
                        entity.type = entity_data.get("type", "UNKNOWN")
                        entity.value = entity_data.get("value", "")
                        entity.confidence = result.confidence
                        entity.resolved_from = entity.value
                        intent.entities.append(entity)

                    self.get_logger().debug(
                        f"LLM parse successful: {intent.type} ({intent.confidence:.2f})"
                    )
                    return intent

        except Exception as e:
            self.get_logger().warn(f"LLM fallback failed: {e}")

        # LLM unavailable or failed - return UNKNOWN
        intent.type = "UNKNOWN"
        intent.confidence = 0.3
        return intent

    def _map_entity_type(self, group_name: str) -> str:
        """
        Map regex group name to entity type.

        Args:
            group_name: Named group from regex pattern

        Returns:
            Entity type string
        """
        mapping = {
            "location": "LOCATION",
            "object": "OBJECT",
            "area": "LOCATION",
            "speed": "SPEED",
            "mode": "MODE",
            "feature": "FEATURE",
        }
        return mapping.get(group_name, group_name.upper())

    def _track_latency(self, latency_ms: float):
        """Track latency for performance monitoring."""
        self._latency_history.append(latency_ms)
        if len(self._latency_history) > self._max_history_size:
            self._latency_history.pop(0)

    def _log_performance_stats(self):
        """Log performance statistics."""
        if not self._latency_history:
            return

        latencies = sorted(self._latency_history)
        p50 = latencies[len(latencies) // 2]
        p95 = latencies[int(len(latencies) * 0.95)]
        p99 = latencies[int(len(latencies) * 0.99)]
        avg = sum(latencies) / len(latencies)

        self.get_logger().info(
            f"Performance Stats (last {len(latencies)} calls): "
            f"avg={avg:.2f}ms, p50={p50:.2f}ms, p95={p95:.2f}ms, p99={p99:.2f}ms"
        )

        # Check if meeting target
        if p95 > self.TARGET_LATENCY_MS:
            self.get_logger().warn(
                f"p95 latency ({p95:.2f}ms) exceeds target ({self.TARGET_LATENCY_MS}ms)"
            )

    def get_performance_stats(self) -> Dict[str, Any]:
        """
        Get current performance statistics.

        Returns:
            Dictionary with performance metrics
        """
        if not self._latency_history:
            return {
                "calls": 0,
                "avg_latency_ms": 0.0,
                "p50_latency_ms": 0.0,
                "p95_latency_ms": 0.0,
                "p99_latency_ms": 0.0,
                "target_met": True,
            }

        latencies = sorted(self._latency_history)
        return {
            "calls": len(latencies),
            "avg_latency_ms": sum(latencies) / len(latencies),
            "p50_latency_ms": latencies[len(latencies) // 2],
            "p95_latency_ms": latencies[int(len(latencies) * 0.95)],
            "p99_latency_ms": latencies[int(len(latencies) * 0.99)],
            "target_met": latencies[int(len(latencies) * 0.95)] <= self.TARGET_LATENCY_MS,
        }

    def _create_unknown_intent(self, utterance: str, confidence: float) -> Intent:
        """
        Create an UNKNOWN intent.

        Args:
            utterance: Original utterance
            confidence: Confidence score

        Returns:
            UNKNOWN Intent
        """
        intent = Intent()
        intent.type = "UNKNOWN"
        intent.confidence = confidence
        intent.raw_utterance = utterance
        intent.source = "UNKNOWN"
        intent.timestamp = self.get_clock().now().to_msg()
        return intent


def main(args=None):
    """Main entry point for the intent parser node."""
    rclpy.init(args=args)
    node = IntentParserNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down intent parser node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
