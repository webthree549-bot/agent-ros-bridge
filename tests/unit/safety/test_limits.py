"""Tests for safety limits node."""

import pytest
import tempfile
import os
from pathlib import Path

from agent_ros_bridge.safety.limits import SafetyLimitsNode


class TestSafetyLimitsNodeInit:
    """Test SafetyLimitsNode initialization."""

    def test_init_with_defaults(self):
        """Node initializes with default limits."""
        node = SafetyLimitsNode()

        assert node._config == {}
        assert node._cache == {}
        assert node.GET_LIMITS_SERVICE == "/safety/get_limits"

    def test_init_with_config_dict(self):
        """Node initializes with config dictionary."""
        config = {"robots": {"robot1": {"max_linear_velocity": 1.0}}}
        node = SafetyLimitsNode(config=config)

        assert node._config == config

    def test_init_with_nonexistent_config_file(self):
        """Node handles nonexistent config file gracefully."""
        node = SafetyLimitsNode(config_path="/nonexistent/path.yaml")

        assert node._config == {"robots": {}}

    def test_init_with_valid_config_file(self):
        """Node loads config from valid YAML file."""
        config_content = """
robots:
  turtlebot1:
    max_linear_velocity: 1.0
    max_angular_velocity: 2.0
"""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            f.write(config_content)
            config_path = f.name

        try:
            node = SafetyLimitsNode(config_path=config_path)

            assert "turtlebot1" in node._config["robots"]
            assert node._config["robots"]["turtlebot1"]["max_linear_velocity"] == 1.0
        finally:
            os.unlink(config_path)

    def test_init_with_invalid_yaml(self):
        """Node handles invalid YAML gracefully."""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            f.write("invalid: yaml: content: [")
            config_path = f.name

        try:
            node = SafetyLimitsNode(config_path=config_path)

            # Should use empty config with defaults
            assert node._config == {"robots": {}}
        finally:
            os.unlink(config_path)


class TestGetLimitsForRobot:
    """Test getting limits for specific robots."""

    def test_get_limits_for_unknown_robot(self):
        """Returns None for unknown robot."""
        node = SafetyLimitsNode()

        limits = node.get_limits_for_robot("unknown_robot")

        assert limits is None

    def test_get_limits_applies_defaults(self):
        """Returns limits with defaults applied."""
        config = {"robots": {"robot1": {"max_linear_velocity": 1.5}}}
        node = SafetyLimitsNode(config=config)

        limits = node.get_limits_for_robot("robot1")

        assert limits is not None
        assert limits["max_linear_velocity"] == 1.5  # From config
        assert limits["max_angular_velocity"] == 1.0  # From defaults
        assert limits["max_force"] == 100.0  # From defaults

    def test_get_limits_caches_result(self):
        """Caches limits for performance."""
        config = {"robots": {"robot1": {"max_linear_velocity": 1.0}}}
        node = SafetyLimitsNode(config=config)

        # First call
        limits1 = node.get_limits_for_robot("robot1")
        # Second call should use cache
        limits2 = node.get_limits_for_robot("robot1")

        assert limits1 is limits2  # Same object from cache
        assert "robot1" in node._cache

    def test_get_limits_for_multiple_robots(self):
        """Handles multiple robots independently."""
        config = {
            "robots": {
                "robot1": {"max_linear_velocity": 1.0},
                "robot2": {"max_linear_velocity": 2.0},
            }
        }
        node = SafetyLimitsNode(config=config)

        limits1 = node.get_limits_for_robot("robot1")
        limits2 = node.get_limits_for_robot("robot2")

        assert limits1["max_linear_velocity"] == 1.0
        assert limits2["max_linear_velocity"] == 2.0


class TestCacheManagement:
    """Test cache clearing functionality."""

    def test_clear_specific_robot_cache(self):
        """Clear cache for specific robot."""
        config = {
            "robots": {
                "robot1": {"max_linear_velocity": 1.0},
                "robot2": {"max_linear_velocity": 2.0},
            }
        }
        node = SafetyLimitsNode(config=config)

        # Populate cache
        node.get_limits_for_robot("robot1")
        node.get_limits_for_robot("robot2")

        # Clear specific robot
        node.clear_cache("robot1")

        assert "robot1" not in node._cache
        assert "robot2" in node._cache

    def test_clear_all_cache(self):
        """Clear all cached limits."""
        config = {
            "robots": {
                "robot1": {"max_linear_velocity": 1.0},
                "robot2": {"max_linear_velocity": 2.0},
            }
        }
        node = SafetyLimitsNode(config=config)

        # Populate cache
        node.get_limits_for_robot("robot1")
        node.get_limits_for_robot("robot2")

        # Clear all
        node.clear_cache()

        assert node._cache == {}
        assert node._cache_timestamp == {}


class TestConfigReload:
    """Test configuration reloading."""

    def test_reload_clears_cache(self):
        """Reload clears the cache."""
        config = {"robots": {"robot1": {"max_linear_velocity": 1.0}}}
        node = SafetyLimitsNode(config=config)

        # Populate cache
        node.get_limits_for_robot("robot1")
        assert "robot1" in node._cache

        # Reload
        node.reload_config()

        assert node._cache == {}

    def test_reload_without_config_path(self):
        """Reload does nothing if no config path."""
        node = SafetyLimitsNode(config={"robots": {}})

        node.reload_config()  # Should not raise

        assert node._config == {"robots": {}}


class TestGetAllRobotIds:
    """Test getting all robot IDs."""

    def test_get_all_robot_ids_empty(self):
        """Returns empty list when no robots configured."""
        node = SafetyLimitsNode()

        robot_ids = node.get_all_robot_ids()

        assert robot_ids == []

    def test_get_all_robot_ids(self):
        """Returns all configured robot IDs."""
        config = {"robots": {"robot1": {}, "robot2": {}, "robot3": {}}}
        node = SafetyLimitsNode(config=config)

        robot_ids = node.get_all_robot_ids()

        assert set(robot_ids) == {"robot1", "robot2", "robot3"}


class TestGetLimitsServiceName:
    """Test service name getter."""

    def test_get_limits_service_name(self):
        """Returns correct service name."""
        node = SafetyLimitsNode()

        service_name = node.get_limits_service_name()

        assert service_name == "/safety/get_limits"


class TestDefaultLimits:
    """Test default limit values."""

    def test_default_limits_values(self):
        """Default limits are conservative."""
        defaults = SafetyLimitsNode.DEFAULT_LIMITS

        assert defaults["max_linear_velocity"] == 0.5
        assert defaults["max_angular_velocity"] == 1.0
        assert defaults["max_joint_velocity"] == 1.0
        assert defaults["max_force"] == 100.0
        assert defaults["workspace_bounds"] == []
        assert defaults["restricted_zones"] == []
