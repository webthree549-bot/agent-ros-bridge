"""
/safety/limits Node
Agent ROS Bridge v0.6.1 - Week 2 Implementation

Provides safety limits for robots.
Loads from config/safety_limits.yaml and caches for performance.
"""

import time
from pathlib import Path
from typing import Any

import yaml  # type: ignore[import-untyped]


class SafetyLimitsNode:
    """
    Safety Limits Node

    Manages and provides safety limits for all robots.
    Loads safety limits from configuration and caches for fast access.
    """

    # Service name constant
    GET_LIMITS_SERVICE = "/safety/get_limits"

    # Default conservative limits (fail-safe)
    DEFAULT_LIMITS = {
        "max_linear_velocity": 0.5,  # m/s - conservative
        "max_angular_velocity": 1.0,  # rad/s - conservative
        "max_joint_velocity": 1.0,  # rad/s - conservative
        "max_force": 100.0,  # N - conservative
        "workspace_bounds": [],
        "restricted_zones": [],
    }

    def __init__(self, config_path: str | None = None, config: dict | None = None):
        """
        Initialize Safety Limits Node

        Args:
            config_path: Path to YAML configuration file
            config: Configuration dictionary (alternative to config_path)
        """
        self._config_path = config_path
        self._config = config or {}
        self._cache: dict[str, dict[str, Any]] = {}
        self._cache_timestamp: dict[str, float] = {}

        # Load configuration
        if config_path and not config:
            self._load_config_from_file()

    def _load_config_from_file(self) -> None:
        """Load configuration from YAML file"""
        config_file = Path(self._config_path)

        if not config_file.exists():
            # Use empty config - defaults will be applied
            self._config = {"robots": {}}
            return

        try:
            with open(config_file) as f:
                self._config = yaml.safe_load(f) or {"robots": {}}
        except Exception as e:
            # Fail-safe: use empty config with conservative defaults
            print(f"Error loading config: {e}. Using conservative defaults.")
            self._config = {"robots": {}}

    def get_limits_for_robot(self, robot_id: str) -> dict[str, Any] | None:
        """
        Get safety limits for a specific robot

        Args:
            robot_id: Unique identifier for the robot

        Returns:
            Dictionary of safety limits, or None if robot not found
        """
        # Check cache first
        if robot_id in self._cache:
            return self._cache[robot_id]

        # Get from config
        robots = self._config.get("robots", {})
        if robot_id not in robots:
            return None

        # Merge with defaults
        robot_config = robots[robot_id]
        limits = self._apply_defaults(robot_config)

        # Cache for performance
        self._cache[robot_id] = limits
        self._cache_timestamp[robot_id] = time.time()

        return limits

    def _apply_defaults(self, robot_config: dict[str, Any]) -> dict[str, Any]:
        """
        Apply default values for missing limits (fail-safe)

        Args:
            robot_config: Robot-specific configuration

        Returns:
            Complete limits with defaults applied
        """
        limits = self.DEFAULT_LIMITS.copy()
        limits.update(robot_config)
        return limits

    def clear_cache(self, robot_id: str | None = None) -> None:
        """
        Clear the limits cache

        Args:
            robot_id: Specific robot to clear, or None for all
        """
        if robot_id:
            self._cache.pop(robot_id, None)
            self._cache_timestamp.pop(robot_id, None)
        else:
            self._cache.clear()
            self._cache_timestamp.clear()

    def reload_config(self) -> None:
        """Reload configuration from file"""
        self._cache.clear()
        self._cache_timestamp.clear()
        if self._config_path:
            self._load_config_from_file()

    def get_all_robot_ids(self) -> list:
        """Get list of all configured robot IDs"""
        return list(self._config.get("robots", {}).keys())

    def get_limits_service_name(self) -> str:
        """Get the service name for GetSafetyLimits"""
        return self.GET_LIMITS_SERVICE
