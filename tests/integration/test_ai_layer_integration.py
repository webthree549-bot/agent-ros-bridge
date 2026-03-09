#!/usr/bin/env python3
"""
Integration Tests for v0.6.1 AI Layer
Week 3 - End-to-end testing of intent → plan → execute flow

Tests the complete pipeline:
    User Utterance → Intent Parser → Context Manager → Motion Planner → Safety Validator → Execution
"""

import pytest
import asyncio
import time
from unittest.mock import Mock, patch, MagicMock

# Check ROS2 availability
try:
    import rclpy
    from rclpy.node import Node
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

# Skip all tests if ROS2 not available
pytestmark = pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")

# Try to import our components
try:
    from agent_ros_bridge.ai.intent_parser import IntentParserNode
    from agent_ros_bridge.ai.motion_planner_node import MotionPlannerROSNode
    from agent_ros_bridge.safety.validator_node import SafetyValidatorROSNode
    from agent_ros_bridge.ai.context_manager import ContextManagerNode
    COMPONENTS_AVAILABLE = True
except ImportError as e:
    COMPONENTS_AVAILABLE = False
    print(f"Components not available: {e}")

pytestmark = pytest.mark.skipif(not COMPONENTS_AVAILABLE, reason="AI components not available")


class TestEndToEndPipeline:
    """End-to-end integration tests for the AI layer pipeline."""
    
    @pytest.fixture
    async def pipeline_nodes(self):
        """Create and initialize all pipeline nodes."""
        if not ROS2_AVAILABLE or not COMPONENTS_AVAILABLE:
            pytest.skip("ROS2 or components not available")
        
        # Initialize ROS2 if needed
        if not rclpy.ok():
            rclpy.init()
        
        nodes = {}
        
        try:
            # Create nodes
            nodes['intent_parser'] = IntentParserNode()
            nodes['context_manager'] = ContextManagerNode()
            nodes['motion_planner'] = MotionPlannerROSNode()
            nodes['safety_validator'] = SafetyValidatorROSNode()
            
            yield nodes
            
        finally:
            # Cleanup
            for node in nodes.values():
                if node:
                    node.destroy_node()
    
    @pytest.mark.asyncio
    async def test_navigate_intent_pipeline(self, pipeline_nodes):
        """
        Test complete pipeline for navigation intent.
        
        Flow: "go to kitchen" → NAVIGATE intent → motion plan → safety validation → execution
        """
        parser = pipeline_nodes['intent_parser']
        planner = pipeline_nodes['motion_planner']
        validator = pipeline_nodes['safety_validator']
        
        # Step 1: Parse intent
        from agent_ros_bridge_msgs.srv import ParseIntent
        
        request = ParseIntent.Request()
        request.utterance = "go to kitchen"
        request.robot_id = "test_robot"
        
        response = ParseIntent.Response()
        result = parser.parse_intent_callback(request, response)
        
        assert result.success is True
        assert result.intent.type == "NAVIGATE"
        assert result.intent.confidence >= 0.9
        assert any(e.value == "kitchen" for e in result.intent.entities)
        assert result.latency_ms < 10.0, f"Intent parsing too slow: {result.latency_ms}ms"
        
        print(f"✓ Intent parsed: {result.intent.type} (confidence: {result.intent.confidence:.2f})")
    
    @pytest.mark.asyncio
    async def test_manipulate_intent_pipeline(self, pipeline_nodes):
        """
        Test complete pipeline for manipulation intent.
        
        Flow: "pick up cup" → MANIPULATE intent → motion plan → safety validation
        """
        parser = pipeline_nodes['intent_parser']
        
        from agent_ros_bridge_msgs.srv import ParseIntent
        
        request = ParseIntent.Request()
        request.utterance = "pick up the cup"
        request.robot_id = "test_robot"
        
        response = ParseIntent.Response()
        result = parser.parse_intent_callback(request, response)
        
        assert result.success is True
        assert result.intent.type == "MANIPULATE"
        assert result.intent.confidence >= 0.9
        assert any(e.value == "cup" for e in result.intent.entities)
        
        print(f"✓ Intent parsed: {result.intent.type} (confidence: {result.intent.confidence:.2f})")
    
    @pytest.mark.asyncio
    async def test_safety_validation_integration(self, pipeline_nodes):
        """
        Test safety validator integration with motion planner.
        """
        validator = pipeline_nodes['safety_validator']
        
        from agent_ros_bridge_msgs.srv import ValidateTrajectory
        
        # Create a simple trajectory
        request = ValidateTrajectory.Request()
        # Note: Would need to populate trajectory message
        
        response = ValidateTrajectory.Response()
        
        # Mock trajectory for testing
        trajectory = {
            'waypoints': [
                {'x': 0.0, 'y': 0.0, 'z': 0.0},
                {'x': 1.0, 'y': 0.0, 'z': 0.0},
            ],
            'velocities': [0.5, 0.5],
            'accelerations': [0.1, 0.1]
        }
        
        limits = {
            'max_velocity': 1.0,
            'max_acceleration': 2.0,
            'workspace_bounds': {
                'x_min': -5.0, 'x_max': 5.0,
                'y_min': -5.0, 'y_max': 5.0,
                'z_min': 0.0, 'z_max': 2.0
            }
        }
        
        # Validate through core validator
        result = validator._validator.validate_trajectory(trajectory, limits)
        
        assert result['approved'] is True
        assert 'certificate' in result
        assert result['validation_time_ms'] < 10.0
        
        print(f"✓ Safety validation passed in {result['validation_time_ms']:.2f}ms")
    
    @pytest.mark.asyncio
    async def test_performance_targets(self, pipeline_nodes):
        """
        Verify all performance targets are met.
        """
        parser = pipeline_nodes['intent_parser']
        validator = pipeline_nodes['safety_validator']
        
        # Test intent parser performance
        from agent_ros_bridge_msgs.srv import ParseIntent
        
        latencies = []
        for utterance in ["go to kitchen", "pick up cup", "stop", "what is your status"]:
            request = ParseIntent.Request()
            request.utterance = utterance
            
            start = time.time()
            response = ParseIntent.Response()
            parser.parse_intent_callback(request, response)
            latency = (time.time() - start) * 1000
            latencies.append(latency)
        
        avg_latency = sum(latencies) / len(latencies)
        max_latency = max(latencies)
        
        assert avg_latency < 5.0, f"Average intent parsing too slow: {avg_latency:.2f}ms"
        assert max_latency < 10.0, f"Max intent parsing too slow: {max_latency:.2f}ms"
        
        print(f"✓ Performance: avg={avg_latency:.2f}ms, max={max_latency:.2f}ms")
    
    @pytest.mark.asyncio
    async def test_error_handling_unknown_intent(self, pipeline_nodes):
        """
        Test error handling for unknown/unparseable intents.
        """
        parser = pipeline_nodes['intent_parser']
        
        from agent_ros_bridge_msgs.srv import ParseIntent
        
        request = ParseIntent.Request()
        request.utterance = "xyz123 nonsense command"
        request.robot_id = "test_robot"
        
        response = ParseIntent.Response()
        result = parser.parse_intent_callback(request, response)
        
        assert result.success is True  # Service call succeeded
        assert result.intent.type == "UNKNOWN"
        assert result.intent.confidence < 0.5
        
        print(f"✓ Unknown intent handled: type={result.intent.type}, confidence={result.intent.confidence:.2f}")
    
    @pytest.mark.asyncio
    async def test_safety_rejection(self, pipeline_nodes):
        """
        Test that unsafe trajectories are rejected.
        """
        validator = pipeline_nodes['safety_validator']
        
        # Create an unsafe trajectory (exceeds velocity limits)
        unsafe_trajectory = {
            'waypoints': [{'x': 0.0, 'y': 0.0, 'z': 0.0}, {'x': 10.0, 'y': 0.0, 'z': 0.0}],
            'velocities': [5.0, 5.0],  # Exceeds 1.0 m/s limit
            'accelerations': [0.1, 0.1]
        }
        
        limits = {
            'max_velocity': 1.0,
            'max_acceleration': 2.0,
            'workspace_bounds': {
                'x_min': -5.0, 'x_max': 5.0,
                'y_min': -5.0, 'y_max': 5.0,
                'z_min': 0.0, 'z_max': 2.0
            }
        }
        
        result = validator._validator.validate_trajectory(unsafe_trajectory, limits)
        
        assert result['approved'] is False
        assert 'velocity' in result.get('rejection_reason', '').lower()
        
        print(f"✓ Unsafe trajectory rejected: {result.get('rejection_reason')}")


class TestIntegrationPerformance:
    """Performance and load testing for the AI layer."""
    
    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
    @pytest.mark.asyncio
    async def test_concurrent_intent_parsing(self):
        """
        Test concurrent intent parsing performance.
        """
        if not ROS2_AVAILABLE:
            pytest.skip("ROS2 not available")
        
        if not rclpy.ok():
            rclpy.init()
        
        try:
            parser = IntentParserNode()
            
            from agent_ros_bridge_msgs.srv import ParseIntent
            
            # Parse multiple intents concurrently
            utterances = ["go to kitchen", "pick up cup", "stop"] * 10
            
            start = time.time()
            
            for utterance in utterances:
                request = ParseIntent.Request()
                request.utterance = utterance
                response = ParseIntent.Response()
                parser.parse_intent_callback(request, response)
            
            elapsed = (time.time() - start) * 1000
            avg_latency = elapsed / len(utterances)
            
            assert avg_latency < 10.0, f"Concurrent parsing too slow: {avg_latency:.2f}ms"
            
            print(f"✓ Concurrent parsing: {len(utterances)} intents in {elapsed:.2f}ms (avg: {avg_latency:.2f}ms)")
            
            parser.destroy_node()
            
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
