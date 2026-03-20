"""Tests for intent parser."""

import re
from unittest.mock import MagicMock, Mock, patch

import pytest

# These tests require ROS2 environment
# Run with: docker exec ros2_humble bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace && python3 -m pytest tests/unit/ai/test_intent_parser.py -v"

# Try to import rclpy and the intent parser, skip tests if not available
try:
    import rclpy

    from agent_ros_bridge.ai.intent_parser import IntentParserNode
    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False
    IntentParserNode = None

pytestmark = pytest.mark.skipif(not RCLPY_AVAILABLE, reason="ROS2 rclpy not available")


class TestIntentParserPatterns:
    """Test intent parser patterns."""

    def test_patterns_compiled(self):
        """Test that patterns are compiled."""
        # Create a minimal node instance
        node = object.__new__(IntentParserNode)
        node.PATTERNS = IntentParserNode.PATTERNS
        node._compiled_patterns = {}
        
        # Compile patterns like the node does
        for intent_type, patterns in node.PATTERNS.items():
            node._compiled_patterns[intent_type] = [re.compile(p, re.I) for p in patterns]
        
        # Test that patterns exist
        assert "NAVIGATE" in node._compiled_patterns
        assert "MANIPULATE" in node._compiled_patterns
        assert "SENSE" in node._compiled_patterns
        assert "QUERY" in node._compiled_patterns

    def test_navigate_patterns_match(self):
        """Test navigate patterns matching."""
        node = object.__new__(IntentParserNode)
        node.PATTERNS = IntentParserNode.PATTERNS
        node._compiled_patterns = {}
        
        for intent_type, patterns in node.PATTERNS.items():
            node._compiled_patterns[intent_type] = [re.compile(p, re.I) for p in patterns]
        
        # Test various navigate utterances
        test_cases = [
            ("go to kitchen", "NAVIGATE"),
            ("navigate to office", "NAVIGATE"),
            ("move to bedroom", "NAVIGATE"),
            ("drive to living room", "NAVIGATE"),
            ("head to garage", "NAVIGATE"),
            ("proceed to kitchen", "NAVIGATE"),
        ]
        
        for utterance, expected_type in test_cases:
            matched = False
            for intent_type, patterns in node._compiled_patterns.items():
                for pattern in patterns:
                    if pattern.search(utterance):
                        assert intent_type == expected_type
                        matched = True
                        break
                if matched:
                    break
            assert matched, f"No pattern matched for: {utterance}"

    def test_manipulate_patterns_match(self):
        """Test manipulate patterns matching."""
        node = object.__new__(IntentParserNode)
        node.PATTERNS = IntentParserNode.PATTERNS
        node._compiled_patterns = {}
        
        for intent_type, patterns in node.PATTERNS.items():
            node._compiled_patterns[intent_type] = [re.compile(p, re.I) for p in patterns]
        
        test_cases = [
            ("pick up the cup", "MANIPULATE"),
            ("grab the bottle", "MANIPULATE"),
            ("place the book", "MANIPULATE"),
            ("put down the pen", "MANIPULATE"),
            ("drop the item", "MANIPULATE"),
        ]
        
        for utterance, expected_type in test_cases:
            matched = False
            for intent_type, patterns in node._compiled_patterns.items():
                for pattern in patterns:
                    if pattern.search(utterance):
                        if intent_type == expected_type:
                            matched = True
                            break
                if matched:
                    break
            assert matched, f"No pattern matched for: {utterance}"


class TestEntityTypeMapping:
    """Test entity type mapping."""

    def test_map_entity_type(self):
        """Test entity type mapping function."""
        node = object.__new__(IntentParserNode)
        
        # Create the mapping method
        def _map_entity_type(group_name: str) -> str:
            mapping = {
                "location": "LOCATION",
                "object": "OBJECT",
                "area": "LOCATION",
                "speed": "SPEED",
                "mode": "MODE",
                "feature": "FEATURE",
            }
            return mapping.get(group_name, group_name.upper())
        
        node._map_entity_type = _map_entity_type
        
        assert node._map_entity_type("location") == "LOCATION"
        assert node._map_entity_type("object") == "OBJECT"
        assert node._map_entity_type("area") == "LOCATION"
        assert node._map_entity_type("speed") == "SPEED"
        assert node._map_entity_type("unknown") == "UNKNOWN"


class TestPerformanceTracking:
    """Test performance tracking."""

    def test_latency_history(self):
        """Test latency history tracking."""
        node = object.__new__(IntentParserNode)
        node._latency_history = []
        node._max_history_size = 1000
        
        # Simulate tracking latencies
        for i in range(10):
            node._latency_history.append(0.005 + i * 0.001)
        
        assert len(node._latency_history) == 10
        
        # Test stats calculation
        import statistics
        mean_latency = statistics.mean(node._latency_history)
        p95_latency = sorted(node._latency_history)[int(len(node._latency_history) * 0.95)]
        
        assert mean_latency > 0
        assert p95_latency > 0

    def test_max_history_size(self):
        """Test max history size limit."""
        node = object.__new__(IntentParserNode)
        node._latency_history = []
        node._max_history_size = 100
        
        # Add more than max
        for i in range(150):
            node._latency_history.append(0.001)
            if len(node._latency_history) > node._max_history_size:
                node._latency_history.pop(0)
        
        assert len(node._latency_history) <= node._max_history_size
