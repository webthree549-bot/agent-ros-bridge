"""
Unit tests for Context Manager Node - Week 2 TDD Implementation.

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

from unittest.mock import Mock, patch, MagicMock
import time

# Skip entire module if ROS2 not available
pytestmark = pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")

# Import messages and services - skip if not available
try:
    from agent_ros_bridge_msgs.srv import ResolveContext
    from agent_ros_bridge_msgs.msg import ContextQuery, ContextResponse
    from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False
    ResolveContext = None
    ContextQuery = None
    ContextResponse = None
    PoseStamped = None
    Pose = None
    Point = None
    Quaternion = None

pytestmark = pytest.mark.skipif(not MSGS_AVAILABLE, reason="ROS2 messages not available")


class TestContextManagerNode:
    """TDD tests for Context Manager Node - Week 2 Sprint"""
    
    @pytest.fixture(scope='function')
    def context_manager(self):
        """Create context manager node for testing."""
        if not rclpy.ok():
            rclpy.init()
        
        from agent_ros_bridge.ai.context_manager import ContextManagerNode
        
        node = ContextManagerNode()
        yield node
        
        node.destroy_node()
    
    def test_context_manager_node_exists(self, context_manager):
        """RED: Node should be discoverable and properly initialized."""
        assert context_manager is not None
        assert context_manager.get_name() == 'context_manager'
    
    def test_resolve_context_service_available(self, context_manager):
        """RED: ResolveContext service should be advertised."""
        services = context_manager.get_service_names_and_types()
        service_names = [s[0] for s in services]
        
        assert '/ai/resolve_context' in service_names or 'ai/resolve_context' in service_names
    
    def test_resolves_location_to_pose(self, context_manager):
        """RED: 'kitchen' → PoseStamped with coordinates."""
        request = ResolveContext.Request()
        request.query.reference_type = "LOCATION"
        request.query.reference_text = "kitchen"
        request.query.robot_id = "turtlebot_01"
        
        response = ResolveContext.Response()
        context_manager.resolve_context_callback(request, response)
        
        assert response.success is True
        assert response.response.found is True
        assert response.response.resolved_type == "POSE"
        assert response.response.pose.header.frame_id == "map"
        assert response.response.confidence > 0.8
    
    def test_resolves_anaphora_it(self, context_manager):
        """RED: 'it' → previous object from context."""
        # First, set up context with a previous object
        context_manager._conversation_context["turtlebot_01"] = {
            "last_object": "cup",
            "last_location": "kitchen"
        }
        
        request = ResolveContext.Request()
        request.query.reference_type = "OBJECT"
        request.query.reference_text = "it"
        request.query.robot_id = "turtlebot_01"
        
        response = ResolveContext.Response()
        context_manager.resolve_context_callback(request, response)
        
        assert response.success is True
        assert response.response.found is True
        assert response.response.entity_id == "cup"
    
    def test_resolves_anaphora_there(self, context_manager):
        """RED: 'there' → previous location from context."""
        # Set up context with a previous location
        context_manager._conversation_context["turtlebot_01"] = {
            "last_object": "cup",
            "last_location": "kitchen"
        }
        
        request = ResolveContext.Request()
        request.query.reference_type = "LOCATION"
        request.query.reference_text = "there"
        request.query.robot_id = "turtlebot_01"
        
        response = ResolveContext.Response()
        context_manager.resolve_context_callback(request, response)
        
        assert response.success is True
        assert response.response.found is True
        assert response.response.entity_id == "kitchen"
    
    def test_maintains_robot_pose(self, context_manager):
        """RED: Node should maintain current robot pose from /tf."""
        # Simulate receiving a TF message
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        pose.pose.position.z = 0.0
        
        context_manager._robot_poses["turtlebot_01"] = pose
        
        # Check that pose is maintained
        assert "turtlebot_01" in context_manager._robot_poses
        assert context_manager._robot_poses["turtlebot_01"].pose.position.x == 1.0
        assert context_manager._robot_poses["turtlebot_01"].pose.position.y == 2.0
    
    def test_resolves_here_to_robot_pose(self, context_manager):
        """RED: 'here' → current robot pose."""
        # Set up robot pose
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 3.0
        pose.pose.position.y = 4.0
        
        context_manager._robot_poses["turtlebot_01"] = pose
        
        request = ResolveContext.Request()
        request.query.reference_type = "LOCATION"
        request.query.reference_text = "here"
        request.query.robot_id = "turtlebot_01"
        
        response = ResolveContext.Response()
        context_manager.resolve_context_callback(request, response)
        
        assert response.success is True
        assert response.response.found is True
        assert response.response.pose.pose.position.x == 3.0
        assert response.response.pose.pose.position.y == 4.0
    
    def test_resolves_office_location(self, context_manager):
        """RED: 'office' should resolve to known location."""
        request = ResolveContext.Request()
        request.query.reference_type = "LOCATION"
        request.query.reference_text = "office"
        request.query.robot_id = "turtlebot_01"
        
        response = ResolveContext.Response()
        context_manager.resolve_context_callback(request, response)
        
        assert response.success is True
        assert response.response.found is True
        assert response.response.resolved_type == "POSE"
    
    def test_unknown_location_returns_not_found(self, context_manager):
        """RED: Unknown location should return found=False."""
        request = ResolveContext.Request()
        request.query.reference_type = "LOCATION"
        request.query.reference_text = "unknown_place_xyz"
        request.query.robot_id = "turtlebot_01"
        
        response = ResolveContext.Response()
        context_manager.resolve_context_callback(request, response)
        
        assert response.success is True  # Service call succeeded
        assert response.response.found is False  # But reference not found
    
    def test_response_includes_description(self, context_manager):
        """RED: Response should include human-readable description."""
        request = ResolveContext.Request()
        request.query.reference_type = "LOCATION"
        request.query.reference_text = "kitchen"
        request.query.robot_id = "turtlebot_01"
        
        response = ResolveContext.Response()
        context_manager.resolve_context_callback(request, response)
        
        assert response.response.description != ""
        assert "kitchen" in response.response.description.lower()
    
    def test_response_includes_latency(self, context_manager):
        """RED: Response should include latency measurement."""
        request = ResolveContext.Request()
        request.query.reference_type = "LOCATION"
        request.query.reference_text = "kitchen"
        request.query.robot_id = "turtlebot_01"
        
        response = ResolveContext.Response()
        context_manager.resolve_context_callback(request, response)
        
        assert response.latency_ms > 0.0
    
    def test_context_resolution_latency_sla(self, context_manager):
        """RED: Context resolution should be < 20ms."""
        request = ResolveContext.Request()
        request.query.reference_type = "LOCATION"
        request.query.reference_text = "kitchen"
        request.query.robot_id = "turtlebot_01"
        
        response = ResolveContext.Response()
        
        start_time = time.time()
        context_manager.resolve_context_callback(request, response)
        end_time = time.time()
        
        latency_ms = (end_time - start_time) * 1000
        
        assert latency_ms < 20.0
    
    def test_multiple_robots_isolated_context(self, context_manager):
        """RED: Different robots should have isolated context."""
        # Set up different contexts for different robots
        context_manager._conversation_context["robot_1"] = {
            "last_object": "cup",
            "last_location": "kitchen"
        }
        context_manager._conversation_context["robot_2"] = {
            "last_object": "book",
            "last_location": "office"
        }
        
        # Query for robot_1
        request = ResolveContext.Request()
        request.query.reference_type = "OBJECT"
        request.query.reference_text = "it"
        request.query.robot_id = "robot_1"
        
        response = ResolveContext.Response()
        context_manager.resolve_context_callback(request, response)
        
        assert response.response.entity_id == "cup"
        
        # Query for robot_2
        request.query.robot_id = "robot_2"
        response = ResolveContext.Response()
        context_manager.resolve_context_callback(request, response)
        
        assert response.response.entity_id == "book"
    
    def test_session_context_tracking(self, context_manager):
        """RED: Session ID should track conversation context."""
        # Set up session context
        context_manager._session_context["session_123"] = {
            "mentioned_objects": ["cup", "table"],
            "visited_locations": ["kitchen"]
        }
        
        request = ResolveContext.Request()
        request.query.reference_type = "OBJECT"
        request.query.reference_text = "it"
        request.query.robot_id = "turtlebot_01"
        request.query.session_id = "session_123"
        
        response = ResolveContext.Response()
        context_manager.resolve_context_callback(request, response)
        
        # Should use session context
        assert response.success is True


class TestContextManagerLocationDB:
    """Tests for location database functionality."""
    
    def test_location_database_exists(self):
        """RED: Location database should be initialized."""
        from agent_ros_bridge.ai.context_manager import ContextManagerNode
        
        # Check that location database is defined
        assert hasattr(ContextManagerNode, 'LOCATION_DATABASE') or \
               hasattr(ContextManagerNode, '_location_db')
    
    def test_kitchen_in_location_db(self):
        """RED: Kitchen should be in location database."""
        from agent_ros_bridge.ai.context_manager import ContextManagerNode
        
        if hasattr(ContextManagerNode, 'LOCATION_DATABASE'):
            assert "kitchen" in ContextManagerNode.LOCATION_DATABASE
        elif hasattr(ContextManagerNode, '_location_db'):
            assert "kitchen" in ContextManagerNode._location_db
    
    def test_office_in_location_db(self):
        """RED: Office should be in location database."""
        from agent_ros_bridge.ai.context_manager import ContextManagerNode
        
        if hasattr(ContextManagerNode, 'LOCATION_DATABASE'):
            assert "office" in ContextManagerNode.LOCATION_DATABASE
        elif hasattr(ContextManagerNode, '_location_db'):
            assert "office" in ContextManagerNode._location_db
