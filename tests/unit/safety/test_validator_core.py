"""Unit tests for safety/validator.py.

Tests the safety validator core logic.
"""

import pytest
import time
from unittest.mock import Mock, patch
from typing import Any

from agent_ros_bridge.safety.validator import SafetyValidatorNode


class TestSafetyValidatorNode:
    """Test SafetyValidatorNode class."""

    @pytest.fixture
    def validator(self):
        """Create a SafetyValidatorNode instance."""
        return SafetyValidatorNode()

    def test_node_creation(self, validator):
        """Validator node can be created."""
        assert validator is not None
        assert validator._validation_count == 0
        assert validator._rejection_count == 0
        assert validator._enable_cache is True

    def test_node_creation_without_cache(self):
        """Validator node can be created without cache."""
        validator = SafetyValidatorNode(enable_cache=False)
        assert validator._enable_cache is False

    def test_validate_trajectory_empty(self, validator):
        """Empty trajectory passes validation."""
        trajectory = {"waypoints": [], "velocities": [], "accelerations": []}
        limits = {
            "max_linear_velocity": 1.0,
            "max_angular_velocity": 1.0,
            "workspace_bounds": {
                "x_min": -5.0, "x_max": 5.0,
                "y_min": -5.0, "y_max": 5.0,
            },
        }

        result = validator.validate_trajectory(trajectory, limits)

        assert result["approved"] is True
        assert result["certificate"] is not None

    def test_validate_trajectory_valid(self, validator):
        """Valid trajectory passes validation."""
        trajectory = {
            "waypoints": [{"x": 1.0, "y": 1.0}],
            "velocities": [0.5],
            "angular_velocities": [0.3],
        }
        limits = {
            "max_linear_velocity": 1.0,
            "max_angular_velocity": 1.0,
            "workspace_bounds": {
                "x_min": -5.0, "x_max": 5.0,
                "y_min": -5.0, "y_max": 5.0,
            },
        }

        result = validator.validate_trajectory(trajectory, limits)

        assert result["approved"] is True
        assert result.get("rejection_reason") is None

    def test_validate_trajectory_exceeds_velocity(self, validator):
        """Trajectory exceeding velocity limit is rejected."""
        trajectory = {
            "waypoints": [{"x": 1.0, "y": 1.0}],
            "velocities": [1.5],  # Exceeds 1.0 limit
        }
        limits = {
            "max_linear_velocity": 1.0,
        }

        result = validator.validate_trajectory(trajectory, limits)

        assert result["approved"] is False
        assert "velocity" in result["rejection_reason"].lower()

    def test_validate_trajectory_exceeds_angular_velocity(self, validator):
        """Trajectory exceeding angular velocity limit is rejected."""
        trajectory = {
            "waypoints": [{"x": 1.0, "y": 1.0}],
            "angular_velocities": [1.5],  # Exceeds 1.0 limit
        }
        limits = {
            "max_angular_velocity": 1.0,
        }

        result = validator.validate_trajectory(trajectory, limits)

        assert result["approved"] is False
        assert "angular velocity" in result["rejection_reason"].lower()

    def test_validate_trajectory_out_of_workspace(self, validator):
        """Trajectory exceeding workspace bounds is rejected."""
        trajectory = {
            "waypoints": [{"x": 10.0, "y": 1.0}],  # Exceeds x_max
        }
        limits = {
            "workspace_bounds": {
                "x_min": -5.0, "x_max": 5.0,
                "y_min": -5.0, "y_max": 5.0,
            },
        }

        result = validator.validate_trajectory(trajectory, limits)

        assert result["approved"] is False
        assert "bounds" in result["rejection_reason"].lower()

    def test_validate_trajectory_exceeds_joint_velocity(self, validator):
        """Trajectory exceeding joint velocity limit is rejected."""
        trajectory = {
            "waypoints": [{"x": 1.0, "y": 1.0}],
            "joint_velocities": [2.0],  # Exceeds 1.0 limit
        }
        limits = {
            "max_joint_velocity": 1.0,
        }

        result = validator.validate_trajectory(trajectory, limits)

        assert result["approved"] is False
        assert "joint velocity" in result["rejection_reason"].lower()

    def test_validate_trajectory_exceeds_force(self, validator):
        """Trajectory exceeding force limit is rejected."""
        trajectory = {
            "waypoints": [{"x": 1.0, "y": 1.0}],
            "forces": [150.0],  # Exceeds 100.0 limit
        }
        limits = {
            "max_force": 100.0,
        }

        result = validator.validate_trajectory(trajectory, limits)

        assert result["approved"] is False
        assert "force" in result["rejection_reason"].lower()

    def test_validate_trajectory_in_restricted_zone(self, validator):
        """Trajectory entering restricted zone is rejected."""
        trajectory = {
            "waypoints": [{"x": 2.0, "y": 2.0}],  # Inside restricted zone
        }
        limits = {
            "restricted_zones": [
                {
                    "name": "danger_zone",
                    "bounds": [
                        {"x": 1.0, "y": 1.0},
                        {"x": 3.0, "y": 1.0},
                        {"x": 3.0, "y": 3.0},
                        {"x": 1.0, "y": 3.0},
                    ],
                },
            ],
        }

        result = validator.validate_trajectory(trajectory, limits)

        assert result["approved"] is False
        assert "restricted zone" in result["rejection_reason"].lower()

    def test_validation_count_incremented(self, validator):
        """Validation count is incremented."""
        trajectory = {"waypoints": []}
        limits = {}

        assert validator._validation_count == 0

        validator.validate_trajectory(trajectory, limits)

        assert validator._validation_count == 1

    def test_rejection_count_incremented(self, validator):
        """Rejection count is incremented for rejected trajectories."""
        trajectory = {
            "waypoints": [{"x": 10.0, "y": 1.0}],
        }
        limits = {
            "workspace_bounds": {
                "x_min": -5.0, "x_max": 5.0,
                "y_min": -5.0, "y_max": 5.0,
            },
        }

        assert validator._rejection_count == 0

        validator.validate_trajectory(trajectory, limits)

        assert validator._rejection_count == 1

    def test_get_stats(self, validator):
        """Statistics can be retrieved."""
        trajectory = {"waypoints": []}
        limits = {}

        validator.validate_trajectory(trajectory, limits)

        stats = validator.get_stats()

        assert stats["validation_count"] == 1
        assert stats["rejection_count"] == 0
        assert "average_validation_time" in stats
        assert "max_validation_time" in stats

    def test_certificate_structure(self, validator):
        """Certificate has correct structure."""
        trajectory = {"waypoints": []}
        limits = {}

        result = validator.validate_trajectory(trajectory, limits)

        cert = result["certificate"]
        assert "validation_id" in cert
        assert "issued_at" in cert
        assert "expires_at" in cert
        assert "trajectory_hash" in cert

    def test_cache_hit(self, validator):
        """Cached result is returned for identical trajectory."""
        trajectory = {"waypoints": []}
        limits = {}

        # First validation
        result1 = validator.validate_trajectory(trajectory, limits)
        assert result1.get("cached") is False

        # Second validation (should be cached)
        result2 = validator.validate_trajectory(trajectory, limits)
        assert result2.get("cached") is True

    def test_cache_statistics(self, validator):
        """Cache statistics are tracked."""
        trajectory = {"waypoints": []}
        limits = {}

        validator.validate_trajectory(trajectory, limits)
        validator.validate_trajectory(trajectory, limits)

        stats = validator.get_cache_statistics()

        assert stats["cache_hits"] == 1
        assert stats["cache_misses"] == 1
        assert stats["hit_rate"] == 0.5

    def test_clear_cache(self, validator):
        """Cache can be cleared."""
        trajectory = {"waypoints": []}
        limits = {}

        validator.validate_trajectory(trajectory, limits)
        validator.clear_cache()

        assert len(validator._cache) == 0
        assert validator._cache_hits == 0
        assert validator._cache_misses == 0

    def test_compute_trajectory_hash(self, validator):
        """Trajectory hash is computed consistently."""
        trajectory = {"waypoints": [{"x": 1.0, "y": 2.0}]}
        limits = {"max_velocity": 1.0}

        hash1 = validator._compute_trajectory_hash(trajectory, limits)
        hash2 = validator._compute_trajectory_hash(trajectory, limits)

        assert hash1 == hash2

    def test_hash_trajectory(self, validator):
        """Simple trajectory hash is generated."""
        trajectory = {"waypoints": [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0}]}

        hash_val = validator._hash_trajectory(trajectory)

        assert hash_val != "empty"
        assert "2_" in hash_val  # 2 waypoints

    def test_hash_empty_trajectory(self, validator):
        """Empty trajectory hash."""
        trajectory = {"waypoints": []}

        hash_val = validator._hash_trajectory(trajectory)

        assert hash_val == "empty"


class TestSafetyValidatorEdgeCases:
    """Test SafetyValidator edge cases."""

    @pytest.fixture
    def validator(self):
        """Create a SafetyValidatorNode instance."""
        return SafetyValidatorNode()

    def test_workspace_bounds_list_format(self, validator):
        """Workspace bounds as list format."""
        trajectory = {"waypoints": [{"x": 3.0, "y": 3.0}]}
        limits = {
            "workspace_bounds": [0.0, 5.0, 0.0, 5.0],  # [x_min, x_max, y_min, y_max]
        }

        result = validator.validate_trajectory(trajectory, limits)
        assert result["approved"] is True

    def test_workspace_bounds_polygon_format(self, validator):
        """Workspace bounds as polygon format."""
        trajectory = {"waypoints": [{"x": 3.0, "y": 3.0}]}
        limits = {
            "workspace_bounds": [
                {"x": 0.0, "y": 0.0},
                {"x": 5.0, "y": 0.0},
                {"x": 5.0, "y": 5.0},
                {"x": 0.0, "y": 5.0},
            ],
        }

        result = validator.validate_trajectory(trajectory, limits)
        assert result["approved"] is True

    def test_no_limits(self, validator):
        """Validation with no limits specified."""
        trajectory = {"waypoints": [{"x": 100.0, "y": 100.0}]}
        limits = {}

        result = validator.validate_trajectory(trajectory, limits)
        # Should pass when no limits are set
        assert result["approved"] is True

    def test_empty_restricted_zones(self, validator):
        """Validation with empty restricted zones."""
        trajectory = {"waypoints": [{"x": 1.0, "y": 1.0}]}
        limits = {"restricted_zones": []}

        result = validator.validate_trajectory(trajectory, limits)
        assert result["approved"] is True

    def test_restricted_zone_no_bounds(self, validator):
        """Restricted zone without bounds is skipped."""
        trajectory = {"waypoints": [{"x": 1.0, "y": 1.0}]}
        limits = {
            "restricted_zones": [{"name": "empty_zone"}],
        }

        result = validator.validate_trajectory(trajectory, limits)
        assert result["approved"] is True

    def test_cache_ttl_expiration(self, validator):
        """Cached results expire after TTL."""
        trajectory = {"waypoints": []}
        limits = {}

        # First validation
        validator.validate_trajectory(trajectory, limits)
        
        # Manually expire cache entry
        traj_hash = validator._compute_trajectory_hash(trajectory, limits)
        result, timestamp = validator._cache[traj_hash]
        validator._cache[traj_hash] = (result, timestamp - validator.CACHE_TTL_SEC - 1)

        # Second validation should be a miss
        result2 = validator.validate_trajectory(trajectory, limits)
        assert result2.get("cached") is False
