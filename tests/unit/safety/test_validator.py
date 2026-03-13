"""
TDD Tests for /safety/validator Node
Agent ROS Bridge v0.6.1 - Week 2 Implementation

Following TDD: Red -> Green -> Refactor
"""

import time

import pytest


class TestValidatorNodeExists:
    """Test that validator node exists and can be instantiated"""

    def test_validator_node_module_exists(self):
        """RED: Safety validator module should exist"""
        try:
            from agent_ros_bridge.safety.validator import SafetyValidatorNode

            assert True
        except ImportError as e:
            pytest.fail(f"SafetyValidatorNode module not found: {e}")

    def test_validator_node_class_exists(self):
        """RED: SafetyValidatorNode class should exist"""
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        assert hasattr(SafetyValidatorNode, "__init__")


class TestValidateTrajectoryService:
    """Test ValidateTrajectory service availability"""

    def test_validate_trajectory_service_available(self):
        """RED: ValidateTrajectory service should be available"""
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        assert hasattr(SafetyValidatorNode, "VALIDATE_SERVICE") or hasattr(
            SafetyValidatorNode, "validate_trajectory"
        ), "Validate service should be defined"


class TestVelocityValidation:
    """Test velocity limit validation"""

    def test_rejects_velocity_above_limit(self):
        """RED: Trajectory with velocity > limit → rejected"""
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        node = SafetyValidatorNode()

        # Mock limits
        limits = {"max_linear_velocity": 1.0, "max_angular_velocity": 1.0}

        # Trajectory with velocity exceeding limit
        trajectory = {"velocities": [0.5, 1.5, 2.0], "waypoints": []}  # 1.5 and 2.0 > 1.0 limit

        result = node.validate_trajectory(trajectory, limits)

        assert result["approved"] is False, "Should reject trajectory with excessive velocity"
        assert (
            "velocity" in result["rejection_reason"].lower()
        ), "Rejection reason should mention velocity"

    def test_approves_velocity_within_limit(self):
        """RED: Trajectory with velocity <= limit → approved"""
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        node = SafetyValidatorNode()

        limits = {"max_linear_velocity": 1.0, "max_angular_velocity": 1.0}

        # Safe trajectory
        trajectory = {"velocities": [0.1, 0.5, 0.8], "waypoints": []}  # All < 1.0

        result = node.validate_trajectory(trajectory, limits)

        assert result["approved"] is True, "Should approve safe trajectory"


class TestWorkspaceValidation:
    """Test workspace boundary validation"""

    def test_rejects_workspace_violation(self):
        """RED: Trajectory outside workspace → rejected"""
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        node = SafetyValidatorNode()

        limits = {
            "max_linear_velocity": 1.0,
            "workspace_bounds": [
                {"x": -10.0, "y": -10.0},
                {"x": 10.0, "y": -10.0},
                {"x": 10.0, "y": 10.0},
                {"x": -10.0, "y": 10.0},
            ],
        }

        # Trajectory with waypoint outside workspace
        trajectory = {
            "velocities": [0.5],
            "waypoints": [{"x": 15.0, "y": 0.0}],  # Outside bounds (x > 10)
        }

        result = node.validate_trajectory(trajectory, limits)

        assert result["approved"] is False, "Should reject trajectory outside workspace"
        assert (
            "workspace" in result["rejection_reason"].lower()
        ), "Rejection reason should mention workspace"

    def test_approves_trajectory_within_workspace(self):
        """RED: Trajectory within workspace → approved"""
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        node = SafetyValidatorNode()

        limits = {
            "max_linear_velocity": 1.0,
            "workspace_bounds": [
                {"x": -10.0, "y": -10.0},
                {"x": 10.0, "y": -10.0},
                {"x": 10.0, "y": 10.0},
                {"x": -10.0, "y": 10.0},
            ],
        }

        # Safe trajectory within bounds
        trajectory = {"velocities": [0.5], "waypoints": [{"x": 5.0, "y": 5.0}]}  # Within bounds

        result = node.validate_trajectory(trajectory, limits)

        assert result["approved"] is True, "Should approve trajectory within workspace"


class TestSafetyCertificate:
    """Test safety certificate generation"""

    def test_approves_safe_trajectory_with_certificate(self):
        """RED: Safe trajectory → approved with certificate"""
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        node = SafetyValidatorNode()

        limits = {"max_linear_velocity": 1.0}
        trajectory = {"velocities": [0.5], "waypoints": [{"x": 0.0, "y": 0.0}]}

        result = node.validate_trajectory(trajectory, limits)

        assert result["approved"] is True
        assert "certificate" in result, "Should include certificate"
        assert result["certificate"] is not None

    def test_certificate_has_30s_validity(self):
        """RED: Certificate expires after 30 seconds"""
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        node = SafetyValidatorNode()

        limits = {"max_linear_velocity": 1.0}
        trajectory = {"velocities": [0.5], "waypoints": [{"x": 0.0, "y": 0.0}]}

        result = node.validate_trajectory(trajectory, limits)
        certificate = result["certificate"]

        assert "issued_at" in certificate, "Certificate should have issued_at"
        assert "expires_at" in certificate, "Certificate should have expires_at"

        validity = certificate["expires_at"] - certificate["issued_at"]
        assert validity == 30.0, f"Certificate validity should be 30s, got {validity}s"

    def test_certificate_contains_validation_id(self):
        """RED: Certificate should contain unique validation ID"""
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        node = SafetyValidatorNode()

        limits = {"max_linear_velocity": 1.0}
        trajectory = {"velocities": [0.5], "waypoints": [{"x": 0.0, "y": 0.0}]}

        result = node.validate_trajectory(trajectory, limits)
        certificate = result["certificate"]

        assert "validation_id" in certificate, "Certificate should have validation_id"
        assert len(certificate["validation_id"]) > 0, "Validation ID should not be empty"


class TestValidationLatency:
    """Test validation timing requirements"""

    def test_validation_latency_under_10ms(self):
        """RED: Validation completes in <10ms"""
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        node = SafetyValidatorNode()

        limits = {
            "max_linear_velocity": 1.0,
            "workspace_bounds": [
                {"x": -10.0, "y": -10.0},
                {"x": 10.0, "y": -10.0},
                {"x": 10.0, "y": 10.0},
                {"x": -10.0, "y": 10.0},
            ],
        }

        trajectory = {
            "velocities": [0.1, 0.2, 0.3, 0.4, 0.5],
            "waypoints": [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 1.0}, {"x": 2.0, "y": 2.0}],
        }

        # Warm up
        node.validate_trajectory(trajectory, limits)

        # Measure validation time
        times = []
        for _ in range(100):
            start = time.time()
            node.validate_trajectory(trajectory, limits)
            elapsed = time.time() - start
            times.append(elapsed * 1000)  # Convert to ms

        avg_time = sum(times) / len(times)
        max_time = max(times)

        assert avg_time < 10.0, f"Average validation too slow: {avg_time}ms"
        assert max_time < 10.0, f"Max validation too slow: {max_time}ms"


class TestJointLimitsValidation:
    """Test joint limits for manipulators"""

    def test_rejects_joint_velocity_above_limit(self):
        """RED: Joint velocity > limit → rejected"""
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        node = SafetyValidatorNode()

        limits = {"max_joint_velocity": 1.0}

        trajectory = {"joint_velocities": [0.5, 1.5], "waypoints": []}  # 1.5 > 1.0

        result = node.validate_trajectory(trajectory, limits)

        assert result["approved"] is False
        assert "joint" in result["rejection_reason"].lower()

    def test_rejects_force_above_limit(self):
        """RED: Force > limit → rejected"""
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        node = SafetyValidatorNode()

        limits = {"max_force": 100.0}

        trajectory = {"forces": [50.0, 150.0], "waypoints": []}  # 150 > 100

        result = node.validate_trajectory(trajectory, limits)

        assert result["approved"] is False
        assert "force" in result["rejection_reason"].lower()


class TestRestrictedZones:
    """Test restricted zone validation"""

    def test_rejects_trajectory_through_restricted_zone(self):
        """RED: Trajectory through restricted zone → rejected"""
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        node = SafetyValidatorNode()

        limits = {
            "max_linear_velocity": 1.0,
            "restricted_zones": [
                {
                    "name": "danger_zone",
                    "bounds": [
                        {"x": 2.0, "y": 2.0},
                        {"x": 4.0, "y": 2.0},
                        {"x": 4.0, "y": 4.0},
                        {"x": 2.0, "y": 4.0},
                    ],
                }
            ],
        }

        # Trajectory passing through restricted zone
        trajectory = {
            "velocities": [0.5],
            "waypoints": [{"x": 3.0, "y": 3.0}],  # Inside restricted zone
        }

        result = node.validate_trajectory(trajectory, limits)

        assert result["approved"] is False
        assert "restricted" in result["rejection_reason"].lower()
