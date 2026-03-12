"""
TDD Tests for /safety/emergency_stop Node
Agent ROS Bridge v0.6.1 - Week 2 Implementation

Following TDD: Red -> Green -> Refactor
"""

import pytest
import time
from unittest.mock import Mock, patch, call


class TestEmergencyStopNodeExists:
    """Test that emergency stop node exists and can be instantiated"""

    def test_emergency_stop_node_module_exists(self):
        """RED: Emergency stop module should exist"""
        try:
            from agent_ros_bridge.safety.emergency_stop import EmergencyStopNode

            assert True
        except ImportError as e:
            pytest.fail(f"EmergencyStopNode module not found: {e}")

    def test_emergency_stop_node_class_exists(self):
        """RED: EmergencyStopNode class should exist"""
        from agent_ros_bridge.safety.emergency_stop import EmergencyStopNode

        assert hasattr(EmergencyStopNode, "__init__")


class TestTriggerEmergencyStopService:
    """Test TriggerEmergencyStop service availability"""

    def test_trigger_emergency_stop_service_available(self):
        """RED: TriggerEmergencyStop service should be available"""
        from agent_ros_bridge.safety.emergency_stop import EmergencyStopNode

        assert hasattr(EmergencyStopNode, "TRIGGER_SERVICE") or hasattr(
            EmergencyStopNode, "trigger_emergency_stop"
        ), "Trigger service should be defined"

    def test_clear_emergency_stop_service_available(self):
        """RED: ClearEmergencyStop service should be available"""
        from agent_ros_bridge.safety.emergency_stop import EmergencyStopNode

        assert hasattr(EmergencyStopNode, "CLEAR_SERVICE") or hasattr(
            EmergencyStopNode, "clear_emergency_stop"
        ), "Clear service should be defined"


class TestEmergencyStopTrigger:
    """Test emergency stop triggering"""

    def test_e_stop_triggers_immediately(self):
        """RED: E-stop triggers in <50ms"""
        from agent_ros_bridge.safety.emergency_stop import EmergencyStopNode

        node = EmergencyStopNode()

        # Measure trigger time
        start = time.time()
        result = node.trigger_emergency_stop(reason="test", source="test")
        elapsed = (time.time() - start) * 1000  # Convert to ms

        assert result is True, "Trigger should succeed"
        assert elapsed < 50.0, f"E-stop too slow: {elapsed}ms"
        assert node.is_emergency_stopped() is True, "E-stop should be active"

    def test_e_stop_cannot_be_overridden_by_software(self):
        """RED: E-stop cannot be overridden by software"""
        from agent_ros_bridge.safety.emergency_stop import EmergencyStopNode

        node = EmergencyStopNode()

        # Trigger e-stop
        node.trigger_emergency_stop(reason="test", source="test")
        assert node.is_emergency_stopped() is True

        # Attempt to override (should fail)
        result = node.attempt_override()
        assert result is False, "Software override should fail"
        assert node.is_emergency_stopped() is True, "E-stop should remain active"


class TestEmergencyStopClear:
    """Test emergency stop clearing"""

    def test_clear_requires_authorization(self):
        """RED: Clearing e-stop requires auth code"""
        from agent_ros_bridge.safety.emergency_stop import EmergencyStopNode

        node = EmergencyStopNode()

        # Trigger e-stop
        node.trigger_emergency_stop(reason="test", source="test")
        assert node.is_emergency_stopped() is True

        # Attempt clear without auth (should fail)
        result = node.clear_emergency_stop(auth_code=None)
        assert result is False, "Clear without auth should fail"
        assert node.is_emergency_stopped() is True

        # Clear with correct auth (should succeed)
        result = node.clear_emergency_stop(auth_code="valid_auth_code")
        assert result is True, "Clear with valid auth should succeed"
        assert node.is_emergency_stopped() is False

    def test_clear_with_invalid_auth_fails(self):
        """RED: Invalid auth code should not clear e-stop"""
        from agent_ros_bridge.safety.emergency_stop import EmergencyStopNode

        node = EmergencyStopNode()

        # Trigger e-stop
        node.trigger_emergency_stop(reason="test", source="test")

        # Attempt clear with invalid auth
        result = node.clear_emergency_stop(auth_code="invalid_code")
        assert result is False, "Clear with invalid auth should fail"
        assert node.is_emergency_stopped() is True


class TestEmergencyStopState:
    """Test emergency stop state management"""

    def test_initial_state_is_not_stopped(self):
        """RED: Initial state should not be emergency stopped"""
        from agent_ros_bridge.safety.emergency_stop import EmergencyStopNode

        node = EmergencyStopNode()
        assert node.is_emergency_stopped() is False

    def test_e_stop_publishes_status(self):
        """RED: E-stop should publish status on state change"""
        from agent_ros_bridge.safety.emergency_stop import EmergencyStopNode

        node = EmergencyStopNode()
        mock_publisher = Mock()
        node.status_publisher = mock_publisher

        # Trigger e-stop
        node.trigger_emergency_stop(reason="test", source="test")

        # Verify status was published
        assert mock_publisher.called, "Status should be published"

    def test_e_stop_logs_all_triggers(self):
        """RED: All e-stop triggers should be logged"""
        from agent_ros_bridge.safety.emergency_stop import EmergencyStopNode

        node = EmergencyStopNode()

        # Trigger e-stop
        node.trigger_emergency_stop(reason="collision_detected", source="validator")

        # Verify trigger was logged
        assert len(node.trigger_history) > 0, "Trigger should be logged"
        last_trigger = node.trigger_history[-1]
        assert last_trigger["reason"] == "collision_detected"
        assert last_trigger["source"] == "validator"


class TestEmergencyStopMultipleSources:
    """Test emergency stop from multiple sources"""

    def test_e_stop_can_be_triggered_from_multiple_sources(self):
        """RED: E-stop can be triggered from validator, watchdog, manual"""
        from agent_ros_bridge.safety.emergency_stop import EmergencyStopNode

        node = EmergencyStopNode()

        sources = ["manual_button", "watchdog", "validator", "hardware"]

        for source in sources:
            node.clear_emergency_stop(auth_code="valid_auth_code")
            assert node.is_emergency_stopped() is False

            result = node.trigger_emergency_stop(reason="test", source=source)
            assert result is True, f"E-stop from {source} should work"
            assert node.is_emergency_stopped() is True


class TestEmergencyStopTiming:
    """Test emergency stop timing requirements"""

    def test_e_stop_activation_latency_under_50ms(self):
        """RED: E-stop activation latency <50ms"""
        from agent_ros_bridge.safety.emergency_stop import EmergencyStopNode

        node = EmergencyStopNode()

        times = []
        for _ in range(10):
            node.clear_emergency_stop(auth_code="valid_auth_code")

            start = time.time()
            node.trigger_emergency_stop(reason="test", source="test")
            elapsed = (time.time() - start) * 1000
            times.append(elapsed)

        avg_time = sum(times) / len(times)
        max_time = max(times)

        assert avg_time < 50.0, f"Average activation too slow: {avg_time}ms"
        assert max_time < 50.0, f"Max activation too slow: {max_time}ms"


class TestEmergencyStopLatching:
    """Test e-stop latching behavior"""

    def test_e_stop_latches_until_cleared(self):
        """RED: E-stop remains active until explicitly cleared"""
        from agent_ros_bridge.safety.emergency_stop import EmergencyStopNode

        node = EmergencyStopNode()

        # Trigger e-stop
        node.trigger_emergency_stop(reason="test", source="test")

        # Wait a bit
        time.sleep(0.1)

        # Should still be stopped
        assert node.is_emergency_stopped() is True, "E-stop should latch"

        # Clear with auth
        node.clear_emergency_stop(auth_code="valid_auth_code")
        assert node.is_emergency_stopped() is False
