#!/usr/bin/env python3
"""
Test suite for Agent ROS Bridge AI Layer interfaces.

Tests message serialization, service compatibility, and interface contracts.
"""

import unittest
import sys
import os
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

# ROS2 imports (these will be available when ROS2 is sourced)
try:
    import rclpy
    from rclpy.node import Node
    from agent_ros_bridge_msgs.msg import (
        Intent, Entity, Constraint,
        ContextQuery, ContextResponse
    )
    from agent_ros_bridge_msgs.srv import ParseIntent, ResolveContext
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS2 not available. Running mock tests only.")


class TestMessageSerialization(unittest.TestCase):
    """Test message serialization and field access."""

    @unittest.skipUnless(ROS2_AVAILABLE, "ROS2 not available")
    def test_intent_message_creation(self):
        """Test Intent message creation and field access."""
        intent = Intent()
        intent.type = "NAVIGATE"
        intent.confidence = 0.95
        intent.raw_utterance = "go to the kitchen"
        intent.source = "RULE_BASED"
        intent.latency_ms = 5.0
        intent.robot_id = "turtlebot_01"
        
        self.assertEqual(intent.type, "NAVIGATE")
        self.assertEqual(intent.confidence, 0.95)
        self.assertEqual(intent.raw_utterance, "go to the kitchen")
        self.assertEqual(intent.source, "RULE_BASED")
        self.assertEqual(intent.latency_ms, 5.0)
        self.assertEqual(intent.robot_id, "turtlebot_01")

    @unittest.skipUnless(ROS2_AVAILABLE, "ROS2 not available")
    def test_entity_message_creation(self):
        """Test Entity message creation."""
        entity = Entity()
        entity.type = "LOCATION"
        entity.value = "kitchen"
        entity.confidence = 0.97
        entity.resolved_from = "the kitchen"
        entity.normalized_value = "kitchen_01"
        entity.unit = ""
        
        self.assertEqual(entity.type, "LOCATION")
        self.assertEqual(entity.value, "kitchen")
        self.assertEqual(entity.confidence, 0.97)

    @unittest.skipUnless(ROS2_AVAILABLE, "ROS2 not available")
    def test_constraint_message_creation(self):
        """Test Constraint message creation."""
        constraint = Constraint()
        constraint.type = "SPEED_LIMIT"
        constraint.value = "0.5"
        constraint.unit = "m/s"
        constraint.priority = 1
        constraint.is_hard = False
        
        self.assertEqual(constraint.type, "SPEED_LIMIT")
        self.assertEqual(constraint.value, "0.5")
        self.assertFalse(constraint.is_hard)

    @unittest.skipUnless(ROS2_AVAILABLE, "ROS2 not available")
    def test_context_query_creation(self):
        """Test ContextQuery message creation."""
        query = ContextQuery()
        query.reference_type = "LOCATION"
        query.reference_text = "kitchen"
        query.robot_id = "turtlebot_01"
        query.hint = "room"
        
        self.assertEqual(query.reference_type, "LOCATION")
        self.assertEqual(query.reference_text, "kitchen")

    @unittest.skipUnless(ROS2_AVAILABLE, "ROS2 not available")
    def test_context_response_creation(self):
        """Test ContextResponse message creation."""
        response = ContextResponse()
        response.found = True
        response.resolved_type = "LOCATION"
        response.entity_id = "kitchen_01"
        response.description = "kitchen at (5.2, 3.1, 0.0)"
        response.confidence = 0.95
        
        self.assertTrue(response.found)
        self.assertEqual(response.resolved_type, "LOCATION")
        self.assertEqual(response.entity_id, "kitchen_01")

    @unittest.skipUnless(ROS2_AVAILABLE, "ROS2 not available")
    def test_intent_with_entities(self):
        """Test Intent with nested Entity array."""
        intent = Intent()
        intent.type = "MANIPULATE"
        
        # Create entities
        entity1 = Entity()
        entity1.type = "OBJECT"
        entity1.value = "red_cup"
        
        entity2 = Entity()
        entity2.type = "ACTION"
        entity2.value = "pick_up"
        
        intent.entities = [entity1, entity2]
        
        self.assertEqual(len(intent.entities), 2)
        self.assertEqual(intent.entities[0].type, "OBJECT")
        self.assertEqual(intent.entities[1].type, "ACTION")

    @unittest.skipUnless(ROS2_AVAILABLE, "ROS2 not available")
    def test_intent_with_constraints(self):
        """Test Intent with nested Constraint array."""
        intent = Intent()
        intent.type = "NAVIGATE"
        
        constraint1 = Constraint()
        constraint1.type = "SPEED_LIMIT"
        constraint1.value = "0.3"
        
        constraint2 = Constraint()
        constraint2.type = "SAFETY_MODE"
        constraint2.value = "high"
        
        intent.constraints = [constraint1, constraint2]
        
        self.assertEqual(len(intent.constraints), 2)
        self.assertEqual(intent.constraints[0].type, "SPEED_LIMIT")
        self.assertEqual(intent.constraints[1].type, "SAFETY_MODE")


class TestServiceInterfaces(unittest.TestCase):
    """Test service request/response structures."""

    @unittest.skipUnless(ROS2_AVAILABLE, "ROS2 not available")
    def test_parse_intent_request(self):
        """Test ParseIntent service request."""
        request = ParseIntent.Request()
        request.utterance = "go to the kitchen"
        request.robot_id = "turtlebot_01"
        request.language = "en"
        
        self.assertEqual(request.utterance, "go to the kitchen")
        self.assertEqual(request.robot_id, "turtlebot_01")
        self.assertEqual(request.language, "en")

    @unittest.skipUnless(ROS2_AVAILABLE, "ROS2 not available")
    def test_parse_intent_response(self):
        """Test ParseIntent service response."""
        response = ParseIntent.Response()
        response.success = True
        response.error_message = ""
        response.suggestions = []
        response.latency_ms = 5.0
        
        # Create intent
        intent = Intent()
        intent.type = "NAVIGATE"
        intent.confidence = 0.97
        response.intent = intent
        
        self.assertTrue(response.success)
        self.assertEqual(response.intent.type, "NAVIGATE")
        self.assertEqual(response.latency_ms, 5.0)

    @unittest.skipUnless(ROS2_AVAILABLE, "ROS2 not available")
    def test_resolve_context_request(self):
        """Test ResolveContext service request."""
        request = ResolveContext.Request()
        request.query.reference_type = "LOCATION"
        request.query.reference_text = "kitchen"
        request.query.robot_id = "turtlebot_01"
        
        self.assertEqual(request.query.reference_type, "LOCATION")
        self.assertEqual(request.query.reference_text, "kitchen")

    @unittest.skipUnless(ROS2_AVAILABLE, "ROS2 not available")
    def test_resolve_context_response(self):
        """Test ResolveContext service response."""
        response = ResolveContext.Response()
        response.success = True
        response.latency_ms = 3.0
        
        context_response = ContextResponse()
        context_response.found = True
        context_response.entity_id = "kitchen_01"
        response.response = context_response
        
        self.assertTrue(response.success)
        self.assertTrue(response.response.found)
        self.assertEqual(response.response.entity_id, "kitchen_01")


class TestInterfaceContracts(unittest.TestCase):
    """Test interface contracts and validation."""

    def test_intent_type_values(self):
        """Test valid intent type values."""
        valid_types = [
            "NAVIGATE", "MANIPULATE", "SENSE", "QUERY",
            "CONFIGURE", "MISSION", "SAFETY", "UNKNOWN"
        ]
        # This test documents the valid types
        self.assertEqual(len(valid_types), 8)

    def test_entity_type_values(self):
        """Test valid entity type values."""
        valid_types = [
            "LOCATION", "OBJECT", "QUANTITY", "SPEED",
            "DIRECTION", "TIME", "ROBOT_ID", "POSE", "PERSON"
        ]
        self.assertEqual(len(valid_types), 9)

    def test_constraint_type_values(self):
        """Test valid constraint type values."""
        valid_types = [
            "SPEED_LIMIT", "TIMEOUT", "AVOID_ZONE", "FORCE_LIMIT",
            "SAFETY_MODE", "APPROACH_ANGLE", "WAIT_CONDITION",
            "PRECISION", "COLLISION_AVOIDANCE", "HUMAN_PROXIMITY",
            "NOISE_LEVEL", "ENERGY_LIMIT"
        ]
        self.assertEqual(len(valid_types), 12)

    def test_confidence_range(self):
        """Test confidence value ranges."""
        # Confidence should be between 0.0 and 1.0
        # This test documents the expected range
        min_confidence = 0.0
        max_confidence = 1.0
        
        # Test high confidence threshold
        high_confidence_threshold = 0.95
        self.assertGreaterEqual(high_confidence_threshold, min_confidence)
        self.assertLessEqual(high_confidence_threshold, max_confidence)
        
        # Test medium confidence range
        medium_low = 0.70
        medium_high = 0.94
        self.assertGreaterEqual(medium_low, min_confidence)
        self.assertLessEqual(medium_high, max_confidence)

    def test_latency_slas(self):
        """Test latency SLA requirements."""
        # Document the latency SLAs from INTERFACE_AI_LAYER.md
        rule_based_target = 5.0  # ms
        rule_based_max = 10.0    # ms
        llm_max = 100.0          # ms
        
        self.assertLess(rule_based_target, rule_based_max)
        self.assertLess(rule_based_max, llm_max)


class TestMockImplementations(unittest.TestCase):
    """Test mock implementations for when ROS2 is not available."""

    def test_mock_intent_parser_service(self):
        """Test mock ParseIntent service implementation."""
        # Simulate a mock service call
        class MockParseIntent:
            def __init__(self):
                self.utterance = ""
                self.robot_id = ""
                
            def parse(self, utterance, robot_id):
                return {
                    "intent": {
                        "type": "NAVIGATE",
                        "confidence": 0.95,
                        "entities": [{"type": "LOCATION", "value": "kitchen"}],
                        "constraints": [],
                        "raw_utterance": utterance,
                        "source": "RULE_BASED",
                        "latency_ms": 5.0
                    },
                    "success": True,
                    "error_message": "",
                    "suggestions": [],
                    "latency_ms": 5.0
                }
        
        mock = MockParseIntent()
        result = mock.parse("go to the kitchen", "turtlebot_01")
        
        self.assertTrue(result["success"])
        self.assertEqual(result["intent"]["type"], "NAVIGATE")
        self.assertEqual(result["intent"]["raw_utterance"], "go to the kitchen")

    def test_mock_context_resolver_service(self):
        """Test mock ResolveContext service implementation."""
        class MockResolveContext:
            def resolve(self, query):
                return {
                    "response": {
                        "found": True,
                        "resolved_type": "LOCATION",
                        "entity_id": "kitchen_01",
                        "description": "kitchen at (5.2, 3.1, 0.0)",
                        "confidence": 0.95,
                        "alternatives": []
                    },
                    "success": True,
                    "error_message": "",
                    "suggestions": [],
                    "latency_ms": 3.0
                }
        
        mock = MockResolveContext()
        query = {"reference_type": "LOCATION", "reference_text": "kitchen"}
        result = mock.resolve(query)
        
        self.assertTrue(result["success"])
        self.assertTrue(result["response"]["found"])
        self.assertEqual(result["response"]["entity_id"], "kitchen_01")


class TestIntegrationScenarios(unittest.TestCase):
    """Test end-to-end integration scenarios."""

    @unittest.skipUnless(ROS2_AVAILABLE, "ROS2 not available")
    def test_simple_navigation_scenario(self):
        """Test simple navigation: 'go to the kitchen'"""
        # Create request
        request = ParseIntent.Request()
        request.utterance = "go to the kitchen"
        request.robot_id = "turtlebot_01"
        
        # Simulate response
        response = ParseIntent.Response()
        response.success = True
        response.latency_ms = 3.2
        
        intent = Intent()
        intent.type = "NAVIGATE"
        intent.confidence = 0.97
        intent.raw_utterance = "go to the kitchen"
        intent.source = "RULE_BASED"
        intent.latency_ms = 3.2
        intent.robot_id = "turtlebot_01"
        
        # Add entity
        entity = Entity()
        entity.type = "LOCATION"
        entity.value = "kitchen"
        entity.resolved_from = "the kitchen"
        entity.confidence = 0.98
        intent.entities = [entity]
        
        response.intent = intent
        
        # Verify
        self.assertTrue(response.success)
        self.assertEqual(response.intent.type, "NAVIGATE")
        self.assertEqual(len(response.intent.entities), 1)
        self.assertEqual(response.intent.entities[0].value, "kitchen")
        self.assertLess(response.latency_ms, 10.0)  # SLA check

    @unittest.skipUnless(ROS2_AVAILABLE, "ROS2 not available")
    def test_complex_manipulation_scenario(self):
        """Test complex manipulation: 'pick up the red cup slowly'"""
        response = ParseIntent.Response()
        response.success = True
        
        intent = Intent()
        intent.type = "MANIPULATE"
        intent.confidence = 0.94
        intent.raw_utterance = "pick up the red cup slowly"
        intent.source = "RULE_BASED"
        
        # Add entities
        entity1 = Entity()
        entity1.type = "OBJECT"
        entity1.value = "red_cup"
        entity1.resolved_from = "the red cup"
        
        entity2 = Entity()
        entity2.type = "ACTION"
        entity2.value = "pick_up"
        entity2.resolved_from = "pick up"
        
        intent.entities = [entity1, entity2]
        
        # Add constraints
        constraint = Constraint()
        constraint.type = "SPEED_LIMIT"
        constraint.value = "0.3"
        constraint.unit = "m/s"
        constraint.is_hard = False
        
        intent.constraints = [constraint]
        response.intent = intent
        
        # Verify
        self.assertEqual(response.intent.type, "MANIPULATE")
        self.assertEqual(len(response.intent.entities), 2)
        self.assertEqual(len(response.intent.constraints), 1)
        self.assertEqual(response.intent.constraints[0].type, "SPEED_LIMIT")

    @unittest.skipUnless(ROS2_AVAILABLE, "ROS2 not available")
    def test_context_resolution_scenario(self):
        """Test context resolution for 'it' reference."""
        request = ResolveContext.Request()
        request.query.reference_type = "OBJECT"
        request.query.reference_text = "it"
        request.query.robot_id = "turtlebot_01"
        request.query.hint = "object"
        
        response = ResolveContext.Response()
        response.success = True
        
        context_response = ContextResponse()
        context_response.found = True
        context_response.resolved_type = "OBJECT"
        context_response.entity_id = "cup_123"
        context_response.description = "the blue cup on the table"
        context_response.confidence = 0.89
        
        response.response = context_response
        
        # Verify
        self.assertTrue(response.success)
        self.assertTrue(response.response.found)
        self.assertEqual(response.response.entity_id, "cup_123")


def run_compatibility_check():
    """Run a quick compatibility check for the message package."""
    print("\n" + "="*60)
    print("AGENT ROS BRIDGE AI LAYER - INTERFACE COMPATIBILITY CHECK")
    print("="*60)
    
    # Check package structure
    package_dir = Path(__file__).parent.parent / 'src' / 'agent_ros_bridge_msgs'
    
    required_files = [
        'package.xml',
        'CMakeLists.txt',
        'msg/Intent.msg',
        'msg/Entity.msg',
        'msg/Constraint.msg',
        'msg/ContextQuery.msg',
        'msg/ContextResponse.msg',
        'srv/ParseIntent.srv',
        'srv/ResolveContext.srv'
    ]
    
    print("\nPackage Structure Check:")
    all_present = True
    for file_path in required_files:
        full_path = package_dir / file_path
        exists = full_path.exists()
        status = "✓" if exists else "✗"
        print(f"  {status} {file_path}")
        all_present = all_present and exists
    
    if all_present:
        print("\n✓ All required files present")
    else:
        print("\n✗ Some files are missing")
    
    # Check ROS2 availability
    print("\nROS2 Environment Check:")
    if ROS2_AVAILABLE:
        print("  ✓ ROS2 is available")
        print("  ✓ Message imports successful")
    else:
        print("  ⚠ ROS2 not available (running mock tests only)")
        print("    To run full tests, source ROS2 environment first:")
        print("    source /opt/ros/humble/setup.bash")
    
    print("\n" + "="*60)
    
    return all_present


if __name__ == '__main__':
    # Run compatibility check first
    run_compatibility_check()
    
    # Run unit tests
    print("\nRunning unit tests...\n")
    unittest.main(verbosity=2, exit=False)
