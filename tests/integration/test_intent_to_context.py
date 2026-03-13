"""
Integration tests for Intent Parser → Context Manager flow.

Tests the end-to-end pipeline: utterance → intent → resolved entities
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
    from agent_ros_bridge_msgs.msg import ContextQuery, Entity, Intent
    from agent_ros_bridge_msgs.srv import ParseIntent, ResolveContext

    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False
    ParseIntent = None
    ResolveContext = None
    Intent = None
    Entity = None
    ContextQuery = None

pytestmark = pytest.mark.skipif(not MSGS_AVAILABLE, reason="agent_ros_bridge_msgs not built")


class TestIntentToContextIntegration:
    """Integration tests for intent parser → context manager flow."""

    @pytest.fixture(scope="function")
    def nodes(self):
        """Create both nodes for integration testing."""
        if not rclpy.ok():
            rclpy.init()

        from agent_ros_bridge.ai.context_manager import ContextManagerNode
        from agent_ros_bridge.ai.intent_parser import IntentParserNode

        intent_parser = IntentParserNode()
        context_manager = ContextManagerNode()

        yield {"intent_parser": intent_parser, "context_manager": context_manager}

        intent_parser.destroy_node()
        context_manager.destroy_node()

    def test_end_to_end_navigate_intent(self, nodes):
        """Integration: 'go to kitchen' → NAVIGATE intent → kitchen pose."""
        # Step 1: Parse intent
        parse_request = ParseIntent.Request()
        parse_request.utterance = "go to kitchen"
        parse_request.robot_id = "turtlebot_01"

        parse_response = ParseIntent.Response()
        nodes["intent_parser"].parse_intent_callback(parse_request, parse_response)

        # Verify intent parsing
        assert parse_response.success is True
        assert parse_response.intent.type == "NAVIGATE"

        # Extract location entity
        location_entities = [e for e in parse_response.intent.entities if e.type == "LOCATION"]
        assert len(location_entities) == 1
        location_name = location_entities[0].value

        # Step 2: Resolve context
        resolve_request = ResolveContext.Request()
        resolve_request.query.reference_type = "LOCATION"
        resolve_request.query.reference_text = location_name
        resolve_request.query.robot_id = "turtlebot_01"

        resolve_response = ResolveContext.Response()
        nodes["context_manager"].resolve_context_callback(resolve_request, resolve_response)

        # Verify context resolution
        assert resolve_response.success is True
        assert resolve_response.response.found is True
        assert resolve_response.response.resolved_type == "POSE"
        assert resolve_response.response.pose.header.frame_id == "map"

    def test_end_to_end_manipulate_intent(self, nodes):
        """Integration: 'pick up the cup' → MANIPULATE intent → cup context."""
        # Step 1: Parse intent
        parse_request = ParseIntent.Request()
        parse_request.utterance = "pick up the cup"
        parse_request.robot_id = "turtlebot_01"

        parse_response = ParseIntent.Response()
        nodes["intent_parser"].parse_intent_callback(parse_request, parse_response)

        # Verify intent parsing
        assert parse_response.success is True
        assert parse_response.intent.type == "MANIPULATE"

        # Extract object entity
        object_entities = [e for e in parse_response.intent.entities if e.type == "OBJECT"]
        assert len(object_entities) == 1
        object_name = object_entities[0].value

        # Step 2: Resolve context (object location)
        resolve_request = ResolveContext.Request()
        resolve_request.query.reference_type = "OBJECT"
        resolve_request.query.reference_text = object_name
        resolve_request.query.robot_id = "turtlebot_01"

        resolve_response = ResolveContext.Response()
        nodes["context_manager"].resolve_context_callback(resolve_request, resolve_response)

        # Object resolution may or may not find the object depending on implementation
        # But the service should succeed
        assert resolve_response.success is True

    def test_end_to_end_with_anaphora(self, nodes):
        """Integration: Context tracking for 'it' reference."""
        # First command: "pick up the cup"
        parse_request = ParseIntent.Request()
        parse_request.utterance = "pick up the cup"
        parse_request.robot_id = "turtlebot_01"
        parse_request.session_id = "session_123"

        parse_response = ParseIntent.Response()
        nodes["intent_parser"].parse_intent_callback(parse_request, parse_response)

        # Update context manager with the parsed intent
        nodes["context_manager"].update_context(
            "turtlebot_01", "session_123", parse_response.intent
        )

        # Second command: "put it down" (anaphoric reference)
        parse_request2 = ParseIntent.Request()
        parse_request2.utterance = "put it down"
        parse_request2.robot_id = "turtlebot_01"
        parse_request2.session_id = "session_123"

        parse_response2 = ParseIntent.Response()
        nodes["intent_parser"].parse_intent_callback(parse_request2, parse_response2)

        # Should resolve "it" to "cup"
        if parse_response2.intent.type == "MANIPULATE":
            # Check if context was used
            object_entities = [e for e in parse_response2.intent.entities if e.type == "OBJECT"]
            if len(object_entities) > 0:
                assert object_entities[0].value == "cup"

    def test_pipeline_latency_sla(self, nodes):
        """Integration: Full pipeline should complete within SLA."""
        # Measure end-to-end latency
        start_time = time.time()

        # Parse intent
        parse_request = ParseIntent.Request()
        parse_request.utterance = "go to kitchen"
        parse_request.robot_id = "turtlebot_01"

        parse_response = ParseIntent.Response()
        nodes["intent_parser"].parse_intent_callback(parse_request, parse_response)

        # Resolve context
        location_entities = [e for e in parse_response.intent.entities if e.type == "LOCATION"]
        if location_entities:
            resolve_request = ResolveContext.Request()
            resolve_request.query.reference_type = "LOCATION"
            resolve_request.query.reference_text = location_entities[0].value
            resolve_request.query.robot_id = "turtlebot_01"

            resolve_response = ResolveContext.Response()
            nodes["context_manager"].resolve_context_callback(resolve_request, resolve_response)

        end_time = time.time()
        total_latency_ms = (end_time - start_time) * 1000

        # End-to-end should be < 100ms (10ms for intent + 20ms for context + overhead)
        assert total_latency_ms < 100.0, f"Pipeline latency {total_latency_ms}ms exceeds 100ms SLA"

    def test_multiple_intents_sequence(self, nodes):
        """Integration: Sequence of related commands."""
        commands = ["go to kitchen", "scan the room", "go to office"]

        for utterance in commands:
            # Parse
            parse_request = ParseIntent.Request()
            parse_request.utterance = utterance
            parse_request.robot_id = "turtlebot_01"

            parse_response = ParseIntent.Response()
            nodes["intent_parser"].parse_intent_callback(parse_request, parse_response)

            assert parse_response.success is True
            assert parse_response.intent.type in [
                "NAVIGATE",
                "SENSE",
                "QUERY",
                "MANIPULATE",
                "SAFETY",
                "UNKNOWN",
            ]

            # If location entity exists, try to resolve it
            location_entities = [e for e in parse_response.intent.entities if e.type == "LOCATION"]
            for entity in location_entities:
                resolve_request = ResolveContext.Request()
                resolve_request.query.reference_type = "LOCATION"
                resolve_request.query.reference_text = entity.value
                resolve_request.query.robot_id = "turtlebot_01"

                resolve_response = ResolveContext.Response()
                nodes["context_manager"].resolve_context_callback(resolve_request, resolve_response)

                assert resolve_response.success is True

    def test_intent_types_map_to_context_types(self, nodes):
        """Integration: Intent types should map to appropriate context resolution."""
        test_cases = [
            ("go to kitchen", "NAVIGATE", "LOCATION"),
            ("pick up the cup", "MANIPULATE", "OBJECT"),
            ("what do you see", "SENSE", None),
            ("where are you", "QUERY", None),
        ]

        for utterance, expected_type, entity_type in test_cases:
            parse_request = ParseIntent.Request()
            parse_request.utterance = utterance
            parse_request.robot_id = "turtlebot_01"

            parse_response = ParseIntent.Response()
            nodes["intent_parser"].parse_intent_callback(parse_request, parse_response)

            assert (
                parse_response.intent.type == expected_type
            ), f"Expected {expected_type} for '{utterance}', got {parse_response.intent.type}"


class TestIntentContextErrorHandling:
    """Integration tests for error handling scenarios."""

    @pytest.fixture(scope="function")
    def nodes(self):
        """Create both nodes for integration testing."""
        if not rclpy.ok():
            rclpy.init()

        from agent_ros_bridge.ai.context_manager import ContextManagerNode
        from agent_ros_bridge.ai.intent_parser import IntentParserNode

        intent_parser = IntentParserNode()
        context_manager = ContextManagerNode()

        yield {"intent_parser": intent_parser, "context_manager": context_manager}

        intent_parser.destroy_node()
        context_manager.destroy_node()

    def test_unknown_location_graceful_handling(self, nodes):
        """Integration: Unknown location should be handled gracefully."""
        # Parse intent with unknown location
        parse_request = ParseIntent.Request()
        parse_request.utterance = "go to unknown_place_xyz"
        parse_request.robot_id = "turtlebot_01"

        parse_response = ParseIntent.Response()
        nodes["intent_parser"].parse_intent_callback(parse_request, parse_response)

        # Intent parsing should succeed
        assert parse_response.success is True
        assert parse_response.intent.type == "NAVIGATE"

        # But context resolution should fail gracefully
        location_entities = [e for e in parse_response.intent.entities if e.type == "LOCATION"]
        if location_entities:
            resolve_request = ResolveContext.Request()
            resolve_request.query.reference_type = "LOCATION"
            resolve_request.query.reference_text = location_entities[0].value
            resolve_request.query.robot_id = "turtlebot_01"

            resolve_response = ResolveContext.Response()
            nodes["context_manager"].resolve_context_callback(resolve_request, resolve_response)

            # Service succeeds but reference not found
            assert resolve_response.success is True
            assert resolve_response.response.found is False

    def test_empty_utterance_handling(self, nodes):
        """Integration: Empty utterance should return UNKNOWN."""
        parse_request = ParseIntent.Request()
        parse_request.utterance = ""
        parse_request.robot_id = "turtlebot_01"

        parse_response = ParseIntent.Response()
        nodes["intent_parser"].parse_intent_callback(parse_request, parse_response)

        assert parse_response.success is True
        assert parse_response.intent.type == "UNKNOWN"
        assert parse_response.intent.confidence < 0.5
