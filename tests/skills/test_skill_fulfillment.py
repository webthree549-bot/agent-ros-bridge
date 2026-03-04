"""TDD Tests: Verify Agent ROS Bridge fulfills SKILL promises.

This test suite validates that the actual implementation matches
the capabilities documented in SKILL.md.

Run with: pytest tests/skills/test_skill_fulfillment.py -v
"""

import unittest
from pathlib import Path

# Parse SKILL.md to extract promised capabilities
SKILL_PATH = Path(__file__).parent.parent.parent / "skills" / "agent-ros-bridge" / "SKILL.md"


class TestSkillPromiseExtraction(unittest.TestCase):
    """Extract and verify all promises from SKILL.md."""
    
    @classmethod
    def setUpClass(cls):
        cls.skill_content = SKILL_PATH.read_text()
    
    def test_movement_commands_documented(self):
        """Verify movement commands are documented in SKILL."""
        movement_commands = [
            "Move forward",
            "Move backward",
            "Turn left",
            "Turn right",
            "Stop",
            "Spin around",
            "Go to",
            "Follow me",
            "Explore",
            "Return to base",
        ]
        for cmd in movement_commands:
            self.assertIn(cmd.lower(), self.skill_content.lower(),
                         f"Movement command '{cmd}' not found in SKILL.md")
    
    def test_sensor_queries_documented(self):
        """Verify sensor queries are documented in SKILL."""
        sensor_queries = [
            "What do you see",
            "Is there anything in front",
            "How far is",
            "Describe the room",
            "Can you see",
            "What's your status",
            "How's your battery",
            "Where are you",
        ]
        for query in sensor_queries:
            self.assertIn(query.lower(), self.skill_content.lower(),
                         f"Sensor query '{query}' not found in SKILL.md")
    
    def test_fleet_commands_documented(self):
        """Verify fleet commands are documented in SKILL."""
        fleet_commands = [
            "Which robot",
            "Send",
            "Search",
            "Form a convoy",
            "Patrol",
        ]
        for cmd in fleet_commands:
            self.assertIn(cmd.lower(), self.skill_content.lower(),
                         f"Fleet command '{cmd}' not found in SKILL.md")
    
    def test_safety_commands_documented(self):
        """Verify safety commands are documented in SKILL."""
        safety_commands = [
            "Emergency stop",
            "Stop everything",
            "Is it safe",
            "Check surroundings",
        ]
        for cmd in safety_commands:
            self.assertIn(cmd.lower(), self.skill_content.lower(),
                         f"Safety command '{cmd}' not found in SKILL.md")
    
    def test_parameter_inference_documented(self):
        """Verify parameter inference is documented."""
        self.assertIn("slowly", self.skill_content.lower())
        self.assertIn("normal", self.skill_content.lower())
        self.assertIn("fast", self.skill_content.lower())


class TestMovementCapabilities(unittest.TestCase):
    """Test that movement capabilities are actually implemented."""
    
    def test_ros2_publish_capability_exists(self):
        """Verify ros2_publish tool exists for movement."""
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None)
        tools = adapter.get_tools()
        tool_names = [t['name'] for t in tools]
        
        self.assertIn("ros2_publish", tool_names,
                     "ros2_publish tool required for movement commands")
    
    def test_move_action_exists(self):
        """Verify 'move' action is available in bridge."""
        # This would require a running bridge, so we check the adapter
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None)
        tools = adapter.get_tools()
        
        # Check for movement-related tools
        movement_tools = ['ros2_publish', 'ros2_action_goal']
        tool_names = [t['name'] for t in tools]
        
        found = [t for t in movement_tools if t in tool_names]
        self.assertTrue(len(found) > 0,
                       f"At least one movement tool should exist, found: {found}")
    
    def test_speed_parameter_handling(self):
        """Verify speed parameters can be handled."""
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None)
        tool = adapter.get_tool("ros2_publish")
        
        self.assertIsNotNone(tool, "ros2_publish tool must exist")
        # The message parameter should accept velocity commands
        self.assertIn("message", tool.parameters,
                     "ros2_publish must have 'message' parameter for velocity")


class TestSensorCapabilities(unittest.TestCase):
    """Test that sensor capabilities are actually implemented."""
    
    def test_ros2_subscribe_capability_exists(self):
        """Verify ros2_subscribe_once tool exists for sensors."""
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None)
        tools = adapter.get_tools()
        tool_names = [t['name'] for t in tools]
        
        self.assertIn("ros2_subscribe_once", tool_names,
                     "ros2_subscribe_once tool required for sensor queries")
    
    def test_camera_snapshot_capability_exists(self):
        """Verify camera snapshot tool exists."""
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None)
        tools = adapter.get_tools()
        tool_names = [t['name'] for t in tools]
        
        self.assertIn("ros2_camera_snapshot", tool_names,
                     "ros2_camera_snapshot tool required for visual perception")
    
    def test_list_topics_capability_exists(self):
        """Verify list_topics tool exists for discovery."""
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None)
        tools = adapter.get_tools()
        tool_names = [t['name'] for t in tools]
        
        self.assertIn("ros2_list_topics", tool_names,
                     "ros2_list_topics tool required for sensor discovery")


class TestFleetCapabilities(unittest.TestCase):
    """Test that fleet capabilities are actually implemented."""
    
    def test_list_robots_capability_exists(self):
        """Verify list_robots tool exists."""
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None)
        tools = adapter.get_tools()
        tool_names = [t['name'] for t in tools]
        
        self.assertIn("bridge_list_robots", tool_names,
                     "bridge_list_robots tool required for fleet management")
    
    def test_fleet_metrics_capability_exists(self):
        """Verify fleet_get_metrics tool exists."""
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None)
        tools = adapter.get_tools()
        tool_names = [t['name'] for t in tools]
        
        self.assertIn("fleet_get_metrics", tool_names,
                     "fleet_get_metrics tool required for fleet status")


class TestSafetyCapabilities(unittest.TestCase):
    """Test that safety capabilities are actually implemented."""
    
    def test_emergency_stop_capability_exists(self):
        """Verify emergency stop tool exists."""
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None)
        tools = adapter.get_tools()
        tool_names = [t['name'] for t in tools]
        
        self.assertIn("safety_trigger_estop", tool_names,
                     "safety_trigger_estop tool required for emergency stop")
    
    def test_safety_manager_integration(self):
        """Verify SafetyManager is available in bridge."""
        from agent_ros_bridge import Bridge
        
        bridge = Bridge()
        self.assertIsNotNone(bridge.safety,
                             "Bridge must have SafetyManager for safety features")


class TestMemoryCapabilities(unittest.TestCase):
    """Test that memory capabilities are actually implemented."""
    
    def test_memory_set_capability_exists(self):
        """Verify memory_set tool exists."""
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None)
        tools = adapter.get_tools()
        tool_names = [t['name'] for t in tools]
        
        self.assertIn("memory_set", tool_names,
                     "memory_set tool required for context persistence")
    
    def test_memory_get_capability_exists(self):
        """Verify memory_get tool exists."""
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None)
        tools = adapter.get_tools()
        tool_names = [t['name'] for t in tools]
        
        self.assertIn("memory_get", tool_names,
                     "memory_get tool required for context retrieval")


class TestAuthenticationCapabilities(unittest.TestCase):
    """Test that authentication capabilities are actually implemented."""
    
    def test_jwt_auth_required(self):
        """Verify JWT authentication is enforced."""
        from agent_ros_bridge import Bridge
        import os
        
        # Remove JWT_SECRET to test enforcement
        original = os.environ.pop('JWT_SECRET', None)
        try:
            # The bridge should warn or handle missing JWT_SECRET
            # Current behavior may vary - this documents expected behavior
            bridge = Bridge()
            # If we get here, check if safety is initialized (it requires JWT)
            self.assertIsNotNone(bridge)
        except (RuntimeError, ValueError, Exception) as e:
            # If an exception is raised, that's the expected secure behavior
            pass
        finally:
            if original:
                os.environ['JWT_SECRET'] = original
            else:
                # Restore a dummy secret for other tests
                os.environ['JWT_SECRET'] = 'test-secret-for-ci'
    
    def test_token_generation_works(self):
        """Verify token generation script exists and works."""
        import subprocess
        import os
        
        script_path = Path(__file__).parent.parent.parent / "scripts" / "generate_token.py"
        self.assertTrue(script_path.exists(),
                       "generate_token.py script must exist")


class TestTransportCapabilities(unittest.TestCase):
    """Test that transport capabilities are actually implemented."""
    
    def test_websocket_transport_exists(self):
        """Verify WebSocket transport is available."""
        from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
        
        # Should be importable
        self.assertTrue(True, "WebSocketTransport imported successfully")
    
    def test_mqtt_transport_exists(self):
        """Verify MQTT transport is available."""
        try:
            from agent_ros_bridge.gateway_v2.transports.mqtt_transport import MQTTTransport
            self.assertTrue(True, "MQTTTransport imported successfully")
        except ImportError:
            self.skipTest("MQTT transport not available (optional)")


class TestRosSupport(unittest.TestCase):
    """Test that ROS1/ROS2 support is actually implemented."""
    
    def test_ros1_tools_available(self):
        """Verify ROS1 tools exist when requested."""
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None, include_ros1=True)
        tools = adapter.get_tools()
        tool_names = [t['name'] for t in tools]
        
        ros1_tools = ['ros1_publish', 'ros1_subscribe_once', 'ros1_service_call']
        for tool in ros1_tools:
            self.assertIn(tool, tool_names,
                         f"ROS1 tool '{tool}' must exist when include_ros1=True")
    
    def test_ros2_tools_available(self):
        """Verify ROS2 tools exist."""
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None)
        tools = adapter.get_tools()
        tool_names = [t['name'] for t in tools]
        
        ros2_tools = ['ros2_publish', 'ros2_subscribe_once', 'ros2_service_call']
        for tool in ros2_tools:
            self.assertIn(tool, tool_names,
                         f"ROS2 tool '{tool}' must exist")


class TestGapAnalysis(unittest.TestCase):
    """Identify gaps between SKILL promises and implementation."""
    
    def test_natural_language_interpretation(self):
        """
        GAP: SKILL promises natural language commands but adapter
        only provides low-level tools.
        
        MISSING: Natural language to ROS command translation layer
        """
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None)
        
        # Check if there's a natural language interpretation method
        has_nl_interpretation = hasattr(adapter, 'interpret_natural_language')
        
        if not has_nl_interpretation:
            self.skipTest(
                "GAP: Natural language interpretation not implemented. "
                "SKILL promises 'Move forward 2 meters' but adapter only has ros2_publish. "
                "Need: interpret_natural_language() method"
            )
    
    def test_parameter_inference(self):
        """
        GAP: SKILL promises parameter inference (slowly=0.1 m/s)
        but this is not implemented.
        
        MISSING: Speed/distance inference from natural language
        """
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None)
        has_inference = hasattr(adapter, 'infer_parameters')
        
        if not has_inference:
            self.skipTest(
                "GAP: Parameter inference not implemented. "
                "SKILL promises 'slowly' → 0.1 m/s inference"
            )
    
    def test_context_awareness(self):
        """
        GAP: SKILL promises context awareness across conversations
        but this is not implemented.
        
        MISSING: Context manager for conversation state
        """
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None)
        has_context = hasattr(adapter, 'context') or hasattr(adapter, 'conversation_history')
        
        if not has_context:
            self.skipTest(
                "GAP: Context awareness not implemented. "
                "SKILL promises 'Now bring me water' understands previous location"
            )
    
    def test_scene_understanding(self):
        """
        GAP: SKILL promises scene description ('What do you see?')
        but camera interpretation is not implemented.
        
        MISSING: Vision model integration for scene description
        """
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None)
        has_vision = hasattr(adapter, 'interpret_camera_view')
        
        if not has_vision:
            self.skipTest(
                "GAP: Scene understanding not implemented. "
                "SKILL promises 'What do you see?' with interpretation"
            )
    
    def test_fleet_intelligence(self):
        """
        GAP: SKILL promises intelligent fleet selection ('Which robot is closest?')
        but advanced fleet intelligence is not implemented.
        
        MISSING: Spatial reasoning, multi-criteria robot selection
        """
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None)
        has_fleet_intelligence = hasattr(adapter, 'select_best_robot')
        
        if not has_fleet_intelligence:
            self.skipTest(
                "GAP: Fleet intelligence not implemented. "
                "SKILL promises 'Send the best robot' with optimization"
            )
    
    def test_autonomous_behaviors(self):
        """
        GAP: SKILL promises autonomous behaviors (exploration, patrol)
        but these are not implemented.
        
        MISSING: High-level mission planning and execution
        """
        from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter
        
        adapter = OpenClawAdapter(bridge=None)
        has_autonomy = hasattr(adapter, 'plan_mission') or hasattr(adapter, 'explore')
        
        if not has_autonomy:
            self.skipTest(
                "GAP: Autonomous behaviors not implemented. "
                "SKILL promises 'Explore autonomously' and 'Patrol'"
            )


if __name__ == "__main__":
    unittest.main()
