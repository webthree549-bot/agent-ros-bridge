"""
TDD Tests for /safety/limits Node
Agent ROS Bridge v0.6.1 - Week 2 Implementation

Following TDD: Red -> Green -> Refactor
"""

import pytest
import yaml
import time
from unittest.mock import patch, mock_open
from pathlib import Path


class TestLimitsNodeExists:
    """Test that limits node exists and can be instantiated"""
    
    def test_limits_node_module_exists(self):
        """RED: Safety limits module should exist"""
        try:
            from agent_ros_bridge.safety.limits import SafetyLimitsNode
            assert True
        except ImportError as e:
            pytest.fail(f"SafetyLimitsNode module not found: {e}")
    
    def test_limits_node_class_exists(self):
        """RED: SafetyLimitsNode class should exist"""
        from agent_ros_bridge.safety.limits import SafetyLimitsNode
        assert hasattr(SafetyLimitsNode, '__init__')


class TestGetSafetyLimitsService:
    """Test GetSafetyLimits service availability"""
    
    def test_get_safety_limits_service_available(self):
        """RED: GetSafetyLimits service should be available"""
        from agent_ros_bridge.safety.limits import SafetyLimitsNode
        
        # Check that the service name constant exists
        assert hasattr(SafetyLimitsNode, 'GET_LIMITS_SERVICE') or \
               hasattr(SafetyLimitsNode, 'get_limits_service_name'), \
               "Service name should be defined"
    
    def test_service_returns_limits_for_robot(self):
        """RED: Service should return max_velocity, workspace_bounds for robot_id"""
        from agent_ros_bridge.safety.limits import SafetyLimitsNode
        
        # Mock configuration
        config = {
            'robots': {
                'turtlebot_01': {
                    'max_linear_velocity': 0.5,
                    'max_angular_velocity': 1.0,
                    'workspace_bounds': [
                        {'x': -10.0, 'y': -10.0},
                        {'x': 10.0, 'y': -10.0},
                        {'x': 10.0, 'y': 10.0},
                        {'x': -10.0, 'y': 10.0}
                    ],
                    'restricted_zones': []
                }
            }
        }
        
        node = SafetyLimitsNode(config=config)
        limits = node.get_limits_for_robot('turtlebot_01')
        
        assert limits is not None, "Limits should not be None"
        assert 'max_linear_velocity' in limits, "Should have max_linear_velocity"
        assert 'max_angular_velocity' in limits, "Should have max_angular_velocity"
        assert 'workspace_bounds' in limits, "Should have workspace_bounds"
        assert limits['max_linear_velocity'] == 0.5
        assert limits['max_angular_velocity'] == 1.0


class TestLimitsConfigLoading:
    """Test configuration loading from YAML"""
    
    def test_loads_limits_from_config_file(self):
        """RED: Loads from config/safety_limits.yaml"""
        from agent_ros_bridge.safety.limits import SafetyLimitsNode
        
        # Mock YAML content
        yaml_content = """
robots:
  turtlebot_01:
    max_linear_velocity: 0.5
    max_angular_velocity: 1.0
    workspace_bounds:
      - {x: -10.0, y: -10.0}
      - {x: 10.0, y: -10.0}
      - {x: 10.0, y: 10.0}
      - {x: -10.0, y: 10.0}
    restricted_zones: []
  
  ur5_01:
    max_joint_velocity: 1.0
    max_force: 100.0
    workspace_bounds:
      - {x: -1.0, y: -1.0, z: 0.0}
      - {x: 1.0, y: -1.0, z: 0.0}
      - {x: 1.0, y: 1.0, z: 0.0}
      - {x: -1.0, y: 1.0, z: 0.0}
    restricted_zones: []
"""
        
        with patch('builtins.open', mock_open(read_data=yaml_content)):
            with patch.object(Path, 'exists', return_value=True):
                node = SafetyLimitsNode(config_path='config/safety_limits.yaml')
                
                # Test turtlebot limits
                tb_limits = node.get_limits_for_robot('turtlebot_01')
                assert tb_limits is not None
                assert tb_limits['max_linear_velocity'] == 0.5
                
                # Test UR5 limits
                ur5_limits = node.get_limits_for_robot('ur5_01')
                assert ur5_limits is not None
                assert ur5_limits['max_joint_velocity'] == 1.0
                assert ur5_limits['max_force'] == 100.0
    
    def test_returns_none_for_unknown_robot(self):
        """RED: Returns None for unknown robot_id"""
        from agent_ros_bridge.safety.limits import SafetyLimitsNode
        
        config = {'robots': {'turtlebot_01': {'max_linear_velocity': 0.5}}}
        node = SafetyLimitsNode(config=config)
        
        limits = node.get_limits_for_robot('unknown_robot')
        assert limits is None, "Should return None for unknown robot"


class TestLimitsCaching:
    """Test limits caching for performance"""
    
    def test_caches_limits_for_performance(self):
        """RED: Limits should be cached for fast access"""
        from agent_ros_bridge.safety.limits import SafetyLimitsNode
        
        config = {
            'robots': {
                'turtlebot_01': {'max_linear_velocity': 0.5}
            }
        }
        
        node = SafetyLimitsNode(config=config)
        
        # First call - should cache
        limits1 = node.get_limits_for_robot('turtlebot_01')
        
        # Second call - should use cache
        limits2 = node.get_limits_for_robot('turtlebot_01')
        
        assert limits1 is limits2, "Should return cached limits"
    
    def test_cache_response_time_under_1ms(self):
        """RED: Cached limits response should be <1ms"""
        from agent_ros_bridge.safety.limits import SafetyLimitsNode
        
        config = {
            'robots': {
                'turtlebot_01': {'max_linear_velocity': 0.5}
            }
        }
        
        node = SafetyLimitsNode(config=config)
        
        # Warm up cache
        node.get_limits_for_robot('turtlebot_01')
        
        # Measure cached access time
        start = time.time()
        for _ in range(1000):
            node.get_limits_for_robot('turtlebot_01')
        elapsed = time.time() - start
        
        avg_time_ms = (elapsed / 1000) * 1000
        assert avg_time_ms < 1.0, f"Cached access too slow: {avg_time_ms}ms"


class TestLimitsDefaultValues:
    """Test default values for missing configuration"""
    
    def test_uses_conservative_defaults_for_missing_limits(self):
        """RED: Missing limits use conservative defaults (fail-safe)"""
        from agent_ros_bridge.safety.limits import SafetyLimitsNode
        
        config = {
            'robots': {
                'turtlebot_01': {}  # Empty config
            }
        }
        
        node = SafetyLimitsNode(config=config)
        limits = node.get_limits_for_robot('turtlebot_01')
        
        # Should have conservative defaults
        assert 'max_linear_velocity' in limits
        assert 'max_angular_velocity' in limits
        assert limits['max_linear_velocity'] <= 0.5  # Conservative
        assert limits['max_angular_velocity'] <= 1.0  # Conservative
