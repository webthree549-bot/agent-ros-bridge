"""
Unit tests for Intent Parser Node - Week 2 TDD Implementation.

Test Philosophy (Red → Green → Refactor):
1. Write failing tests first (Red)
2. Implement minimal code to pass (Green)
3. Refactor while keeping tests green
"""

import pytest

# Check if ROS2 is available
try:
    import rclpy
    from rclpy.node import Node

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

import time

# Skip entire module if ROS2 not available
pytestmark = pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")

# Import messages and services - skip if not available
try:
    from agent_ros_bridge_msgs.msg import Entity, Intent
    from agent_ros_bridge_msgs.srv import ParseIntent

    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False
    ParseIntent = None
    Intent = None
    Entity = None

pytestmark = pytest.mark.skipif(not MSGS_AVAILABLE, reason="agent_ros_bridge_msgs not built")


class TestIntentParserNode:
    """TDD tests for Intent Parser Node - Week 2 Sprint"""

    @pytest.fixture(scope="function")
    def intent_parser(self):
        """Create intent parser node for testing."""
        # Initialize rclpy if not already initialized
        if not rclpy.ok():
            rclpy.init()

        # Import here to avoid early import issues
        from agent_ros_bridge.ai.intent_parser import IntentParserNode

        node = IntentParserNode()
        yield node

        # Cleanup
        node.destroy_node()

    def test_intent_parser_node_exists(self, intent_parser):
        """RED: Node should be discoverable and properly initialized."""
        assert intent_parser is not None
        assert intent_parser.get_name() == "intent_parser"

    def test_parse_intent_service_available(self, intent_parser):
        """RED: ParseIntent service should be advertised."""
        # Check that the service exists
        services = intent_parser.get_service_names_and_types()
        service_names = [s[0] for s in services]

        # Service should be available
        assert "/ai/parse_intent" in service_names or "ai/parse_intent" in service_names

    def test_parses_navigate_intent(self, intent_parser):
        """RED: 'go to kitchen' → NAVIGATE intent with LOCATION entity."""
        request = ParseIntent.Request()
        request.utterance = "go to kitchen"
        request.robot_id = "turtlebot_01"

        response = ParseIntent.Response()
        intent_parser.parse_intent_callback(request, response)

        assert response.success is True
        assert response.intent.type == "NAVIGATE"
        assert response.intent.confidence >= 0.95

        # Check entity extraction
        location_entities = [e for e in response.intent.entities if e.type == "LOCATION"]
        assert len(location_entities) == 1
        assert location_entities[0].value == "kitchen"

    def test_parses_manipulate_intent(self, intent_parser):
        """RED: 'pick up the cup' → MANIPULATE intent with OBJECT entity."""
        request = ParseIntent.Request()
        request.utterance = "pick up the cup"
        request.robot_id = "turtlebot_01"

        response = ParseIntent.Response()
        intent_parser.parse_intent_callback(request, response)

        assert response.success is True
        assert response.intent.type == "MANIPULATE"
        assert response.intent.confidence >= 0.95

        # Check entity extraction
        object_entities = [e for e in response.intent.entities if e.type == "OBJECT"]
        assert len(object_entities) == 1
        assert object_entities[0].value == "cup"

    def test_parses_sense_intent(self, intent_parser):
        """RED: 'what do you see' → SENSE intent."""
        request = ParseIntent.Request()
        request.utterance = "what do you see"
        request.robot_id = "turtlebot_01"

        response = ParseIntent.Response()
        intent_parser.parse_intent_callback(request, response)

        assert response.success is True
        assert response.intent.type == "SENSE"
        assert response.intent.confidence >= 0.95

    def test_parses_query_intent(self, intent_parser):
        """RED: 'where are you' → QUERY intent."""
        request = ParseIntent.Request()
        request.utterance = "where are you"
        request.robot_id = "turtlebot_01"

        response = ParseIntent.Response()
        intent_parser.parse_intent_callback(request, response)

        assert response.success is True
        assert response.intent.type == "QUERY"
        assert response.intent.confidence >= 0.95

    def test_parses_safety_intent(self, intent_parser):
        """RED: 'stop' → SAFETY intent."""
        request = ParseIntent.Request()
        request.utterance = "stop"
        request.robot_id = "turtlebot_01"

        response = ParseIntent.Response()
        intent_parser.parse_intent_callback(request, response)

        assert response.success is True
        assert response.intent.type == "SAFETY"
        assert response.intent.confidence >= 0.95

    def test_rule_based_fast_path(self, intent_parser):
        """RED: Rule-based parsing should be < 10ms."""
        request = ParseIntent.Request()
        request.utterance = "go to kitchen"
        request.robot_id = "turtlebot_01"

        response = ParseIntent.Response()

        # Measure latency
        start_time = time.time()
        intent_parser.parse_intent_callback(request, response)
        end_time = time.time()

        latency_ms = (end_time - start_time) * 1000

        assert response.intent.source == "RULE_BASED"
        assert response.intent.latency_ms < 10.0
        assert latency_ms < 10.0  # Local measurement

    def test_llm_fallback_for_complex(self, intent_parser):
        """RED: Complex utterances should use LLM fallback."""
        # This is a stretch goal for Week 2
        # For now, test that complex utterances return UNKNOWN with low confidence
        request = ParseIntent.Request()
        request.utterance = (
            "go to the kitchen but avoid the living room and don't bump into furniture"
        )
        request.robot_id = "turtlebot_01"

        response = ParseIntent.Response()
        intent_parser.parse_intent_callback(request, response)

        # Should still return success but may have lower confidence
        assert response.success is True
        # Complex utterances should either parse with lower confidence or use LLM
        assert response.intent.confidence < 0.95 or response.intent.source == "LLM_ASSISTED"

    def test_low_confidence_returns_unknown(self, intent_parser):
        """RED: Unclear utterances → UNKNOWN with low confidence."""
        request = ParseIntent.Request()
        request.utterance = "xyz abc 123 nonsense"
        request.robot_id = "turtlebot_01"

        response = ParseIntent.Response()
        intent_parser.parse_intent_callback(request, response)

        assert response.success is True
        assert response.intent.type == "UNKNOWN"
        assert response.intent.confidence < 0.70

    def test_entity_extraction(self, intent_parser):
        """RED: Extracts LOCATION, OBJECT, SPEED entities correctly."""
        request = ParseIntent.Request()
        request.utterance = "go to the kitchen"
        request.robot_id = "turtlebot_01"

        response = ParseIntent.Response()
        intent_parser.parse_intent_callback(request, response)

        # Should have at least one entity
        assert len(response.intent.entities) >= 1

        # Check entity structure
        for entity in response.intent.entities:
            assert entity.type != ""
            assert entity.value != ""
            assert entity.confidence > 0.0

    def test_multiple_navigate_patterns(self, intent_parser):
        """RED: Various navigate patterns should all work."""
        patterns = [
            ("go to kitchen", "kitchen"),
            ("navigate to office", "office"),
            ("move to bedroom", "bedroom"),
            ("drive to garage", "garage"),
            ("head to lobby", "lobby"),
        ]

        for utterance, expected_location in patterns:
            request = ParseIntent.Request()
            request.utterance = utterance
            request.robot_id = "turtlebot_01"

            response = ParseIntent.Response()
            intent_parser.parse_intent_callback(request, response)

            assert response.intent.type == "NAVIGATE", f"Failed for: {utterance}"

            location_entities = [e for e in response.intent.entities if e.type == "LOCATION"]
            assert len(location_entities) == 1, f"Failed for: {utterance}"
            assert location_entities[0].value == expected_location, f"Failed for: {utterance}"

    def test_multiple_manipulate_patterns(self, intent_parser):
        """RED: Various manipulate patterns should all work."""
        patterns = [
            ("pick up the cup", "cup"),
            ("grab the box", "box"),
            ("place the book", "book"),
            ("put down the bottle", "bottle"),
        ]

        for utterance, expected_object in patterns:
            request = ParseIntent.Request()
            request.utterance = utterance
            request.robot_id = "turtlebot_01"

            response = ParseIntent.Response()
            intent_parser.parse_intent_callback(request, response)

            assert response.intent.type == "MANIPULATE", f"Failed for: {utterance}"

            object_entities = [e for e in response.intent.entities if e.type == "OBJECT"]
            assert len(object_entities) == 1, f"Failed for: {utterance}"
            assert object_entities[0].value == expected_object, f"Failed for: {utterance}"

    def test_response_includes_latency(self, intent_parser):
        """RED: Response should include latency measurement."""
        request = ParseIntent.Request()
        request.utterance = "go to kitchen"
        request.robot_id = "turtlebot_01"

        response = ParseIntent.Response()
        intent_parser.parse_intent_callback(request, response)

        assert response.intent.latency_ms > 0.0
        assert response.latency_ms > 0.0

    def test_response_includes_raw_utterance(self, intent_parser):
        """RED: Intent should preserve raw utterance."""
        request = ParseIntent.Request()
        request.utterance = "go to kitchen"
        request.robot_id = "turtlebot_01"

        response = ParseIntent.Response()
        intent_parser.parse_intent_callback(request, response)

        assert response.intent.raw_utterance == "go to kitchen"

    def test_response_includes_timestamp(self, intent_parser):
        """RED: Intent should include timestamp."""
        request = ParseIntent.Request()
        request.utterance = "go to kitchen"
        request.robot_id = "turtlebot_01"

        response = ParseIntent.Response()
        intent_parser.parse_intent_callback(request, response)

        # Timestamp should be set (non-zero)
        assert response.intent.timestamp.sec > 0 or response.intent.timestamp.nanosec > 0


class TestIntentParserPatterns:
    """Tests for pattern matching logic in isolation."""

    def test_navigate_patterns_compiled(self):
        """RED: Navigate patterns should be valid regex."""
        # Check that patterns are valid regex
        import re

        from agent_ros_bridge.ai.intent_parser import IntentParserNode

        for pattern in IntentParserNode.PATTERNS["NAVIGATE"]:
            # Should compile without error
            compiled = re.compile(pattern, re.IGNORECASE)
            assert compiled is not None

    def test_manipulate_patterns_compiled(self):
        """RED: Manipulate patterns should be valid regex."""
        import re

        from agent_ros_bridge.ai.intent_parser import IntentParserNode

        for pattern in IntentParserNode.PATTERNS["MANIPULATE"]:
            compiled = re.compile(pattern, re.IGNORECASE)
            assert compiled is not None

    def test_sense_patterns_compiled(self):
        """RED: Sense patterns should be valid regex."""
        import re

        from agent_ros_bridge.ai.intent_parser import IntentParserNode

        for pattern in IntentParserNode.PATTERNS["SENSE"]:
            compiled = re.compile(pattern, re.IGNORECASE)
            assert compiled is not None

    def test_query_patterns_compiled(self):
        """RED: Query patterns should be valid regex."""
        import re

        from agent_ros_bridge.ai.intent_parser import IntentParserNode

        for pattern in IntentParserNode.PATTERNS["QUERY"]:
            compiled = re.compile(pattern, re.IGNORECASE)
            assert compiled is not None

    def test_safety_patterns_compiled(self):
        """RED: Safety patterns should be valid regex."""
        import re

        from agent_ros_bridge.ai.intent_parser import IntentParserNode

        for pattern in IntentParserNode.PATTERNS["SAFETY"]:
            compiled = re.compile(pattern, re.IGNORECASE)
            assert compiled is not None
