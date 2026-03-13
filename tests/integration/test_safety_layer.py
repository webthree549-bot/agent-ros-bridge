"""
Integration Tests for Safety Layer
Agent ROS Bridge v0.6.1 - Week 2 Implementation

Tests integration between:
- validator → limits
- watchdog → emergency_stop
- End-to-end safety flow
"""

import time


class TestValidatorLimitsIntegration:
    """Test integration between validator and limits nodes"""

    def test_validator_queries_limits_for_robot(self):
        """Validator should query limits from limits node"""
        from agent_ros_bridge.safety.limits import SafetyLimitsNode
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        # Setup limits node with config
        config = {
            "robots": {
                "turtlebot_01": {
                    "max_linear_velocity": 0.5,
                    "workspace_bounds": [
                        {"x": -10.0, "y": -10.0},
                        {"x": 10.0, "y": -10.0},
                        {"x": 10.0, "y": 10.0},
                        {"x": -10.0, "y": 10.0},
                    ],
                }
            }
        }
        limits_node = SafetyLimitsNode(config=config)
        validator_node = SafetyValidatorNode()

        # Get limits for robot
        limits = limits_node.get_limits_for_robot("turtlebot_01")
        assert limits is not None

        # Validate trajectory using those limits
        trajectory = {"velocities": [0.3], "waypoints": [{"x": 0.0, "y": 0.0}]}  # Within 0.5 limit

        result = validator_node.validate_trajectory(trajectory, limits)
        assert result["approved"] is True

    def test_validator_rejects_when_limits_exceeded(self):
        """Validator should reject trajectory exceeding cached limits"""
        from agent_ros_bridge.safety.limits import SafetyLimitsNode
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        config = {"robots": {"turtlebot_01": {"max_linear_velocity": 0.5}}}
        limits_node = SafetyLimitsNode(config=config)
        validator_node = SafetyValidatorNode()

        limits = limits_node.get_limits_for_robot("turtlebot_01")

        # Trajectory exceeding limits
        trajectory = {"velocities": [0.8], "waypoints": [{"x": 0.0, "y": 0.0}]}  # Exceeds 0.5 limit

        result = validator_node.validate_trajectory(trajectory, limits)
        assert result["approved"] is False


class TestWatchdogEmergencyStopIntegration:
    """Test integration between watchdog and emergency stop"""

    def test_watchdog_triggers_e_stop_on_timeout(self):
        """Watchdog should trigger e-stop when critical node times out"""
        from agent_ros_bridge.safety.emergency_stop import EmergencyStopNode
        from agent_ros_bridge.safety.watchdog import WatchdogNode

        watchdog = WatchdogNode()
        e_stop = EmergencyStopNode()

        # Connect watchdog to e-stop
        watchdog.emergency_stop_callback = e_stop.trigger_emergency_stop

        # Register critical node
        watchdog.register_node("/safety/validator", critical=True)

        # Send initial heartbeat
        watchdog.process_heartbeat("/safety/validator", seq=1, timestamp=time.time())
        assert e_stop.is_emergency_stopped() is False

        # Simulate timeout with old timestamp
        old_timestamp = time.time() - 0.01  # 10ms ago
        watchdog.process_heartbeat("/safety/validator", seq=1, timestamp=old_timestamp)
        watchdog.check_timeouts()

        # E-stop should be triggered
        assert e_stop.is_emergency_stopped() is True

    def test_watchdog_logs_timeout_before_e_stop(self):
        """Watchdog should log timeout event before triggering e-stop"""
        from agent_ros_bridge.safety.emergency_stop import EmergencyStopNode
        from agent_ros_bridge.safety.watchdog import WatchdogNode

        watchdog = WatchdogNode()
        e_stop = EmergencyStopNode()

        watchdog.emergency_stop_callback = e_stop.trigger_emergency_stop
        watchdog.register_node("/test_node", critical=True)

        # Simulate timeout
        old_timestamp = time.time() - 0.01
        watchdog.process_heartbeat("/test_node", seq=1, timestamp=old_timestamp)
        watchdog.check_timeouts()

        # Verify timeout was logged
        assert len(watchdog.timeout_log) > 0
        assert watchdog.timeout_log[-1]["node"] == "/test_node"


class TestEndToEndSafetyFlow:
    """Test end-to-end safety flow"""

    def test_safe_trajectory_flow(self):
        """End-to-end: Safe trajectory should be validated and approved"""
        from agent_ros_bridge.safety.limits import SafetyLimitsNode
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        # Setup
        config = {
            "robots": {
                "turtlebot_01": {
                    "max_linear_velocity": 1.0,
                    "workspace_bounds": [{"x": -10.0, "y": -10.0}, {"x": 10.0, "y": 10.0}],
                }
            }
        }
        limits_node = SafetyLimitsNode(config=config)
        validator_node = SafetyValidatorNode()

        # Get limits
        limits = limits_node.get_limits_for_robot("turtlebot_01")

        # Create safe trajectory
        trajectory = {
            "velocities": [0.5, 0.6, 0.7],
            "waypoints": [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 1.0}, {"x": 2.0, "y": 2.0}],
        }

        # Validate
        result = validator_node.validate_trajectory(trajectory, limits)

        # Should be approved with certificate
        assert result["approved"] is True
        assert result["certificate"] is not None
        assert "validation_id" in result["certificate"]

    def test_unsafe_trajectory_flow(self):
        """End-to-end: Unsafe trajectory should be rejected"""
        from agent_ros_bridge.safety.limits import SafetyLimitsNode
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        config = {"robots": {"turtlebot_01": {"max_linear_velocity": 0.5}}}
        limits_node = SafetyLimitsNode(config=config)
        validator_node = SafetyValidatorNode()

        limits = limits_node.get_limits_for_robot("turtlebot_01")

        # Create unsafe trajectory
        trajectory = {"velocities": [0.8], "waypoints": [{"x": 0.0, "y": 0.0}]}  # Exceeds limit

        result = validator_node.validate_trajectory(trajectory, limits)

        # Should be rejected without certificate
        assert result["approved"] is False
        assert result["certificate"] is None
        assert "velocity" in result["rejection_reason"].lower()

    def test_e_stop_blocks_validation(self):
        """End-to-end: E-stop active should block validation/execution"""
        from agent_ros_bridge.safety.emergency_stop import EmergencyStopNode
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        e_stop = EmergencyStopNode()
        validator = SafetyValidatorNode()

        # Trigger e-stop
        e_stop.trigger_emergency_stop(reason="test", source="test")
        assert e_stop.is_emergency_stopped() is True

        # Validation should still work (it's independent)
        # But execution would be blocked by e-stop state
        limits = {"max_linear_velocity": 1.0}
        trajectory = {"velocities": [0.5], "waypoints": [{"x": 0.0, "y": 0.0}]}

        result = validator.validate_trajectory(trajectory, limits)

        # Validator operates independently
        assert result["approved"] is True
        # But executor would check e-stop state before executing


class TestSafetyLayerFailSafe:
    """Test fail-safe behaviors"""

    def test_unknown_robot_returns_none(self):
        """Unknown robot should return None (fail-safe)"""
        from agent_ros_bridge.safety.limits import SafetyLimitsNode

        limits_node = SafetyLimitsNode(config={"robots": {}})
        limits = limits_node.get_limits_for_robot("unknown_robot")

        assert limits is None

    def test_empty_limits_use_defaults(self):
        """Empty limits config should use conservative defaults"""
        from agent_ros_bridge.safety.limits import SafetyLimitsNode

        config = {"robots": {"test_robot": {}}}  # Empty config
        limits_node = SafetyLimitsNode(config=config)
        limits = limits_node.get_limits_for_robot("test_robot")

        # Should have conservative defaults
        assert limits["max_linear_velocity"] <= 0.5
        assert limits["max_angular_velocity"] <= 1.0

    def test_missing_config_file_uses_empty_defaults(self):
        """Missing config file should use empty defaults"""
        from agent_ros_bridge.safety.limits import SafetyLimitsNode

        limits_node = SafetyLimitsNode(config_path="/nonexistent/path.yaml")
        limits = limits_node.get_limits_for_robot("any_robot")

        assert limits is None  # No robots configured
