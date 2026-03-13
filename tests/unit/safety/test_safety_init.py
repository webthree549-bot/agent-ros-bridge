"""Tests for safety module initialization and exports."""


from agent_ros_bridge import safety


class TestSafetyModuleImports:
    """Test safety module imports and availability flags."""

    def test_ros2_nodes_available_flag_exists(self):
        """_ROS2_NODES_AVAILABLE flag is defined."""
        assert hasattr(safety, "_ROS2_NODES_AVAILABLE")
        assert isinstance(safety._ROS2_NODES_AVAILABLE, bool)

    def test_safety_manager_exported(self):
        """SafetyManager is exported."""
        assert hasattr(safety, "SafetyManager")
        assert safety.SafetyManager is not None

    def test_safety_level_exported(self):
        """SafetyLevel is exported."""
        assert hasattr(safety, "SafetyLevel")
        assert safety.SafetyLevel is not None

    def test_safety_policy_exported(self):
        """SafetyPolicy is exported."""
        assert hasattr(safety, "SafetyPolicy")
        assert safety.SafetyPolicy is not None

    def test_confirmation_request_exported(self):
        """ConfirmationRequest is exported."""
        assert hasattr(safety, "ConfirmationRequest")
        assert safety.ConfirmationRequest is not None

    def test_safety_validator_node_exported(self):
        """SafetyValidatorNode is exported."""
        assert hasattr(safety, "SafetyValidatorNode")
        # Note: SafetyValidatorNode may be None if ROS2 not available
        # The important thing is that the attribute exists

    def test_safety_limits_node_exported(self):
        """SafetyLimitsNode is exported (may be None)."""
        assert hasattr(safety, "SafetyLimitsNode")
        # May be None if ROS2 not available

    def test_emergency_stop_node_exported(self):
        """EmergencyStopNode is exported (may be None)."""
        assert hasattr(safety, "EmergencyStopNode")
        # May be None if ROS2 not available

    def test_watchdog_node_exported(self):
        """WatchdogNode is exported (may be None)."""
        assert hasattr(safety, "WatchdogNode")
        # May be None if ROS2 not available

    def test_safety_validator_ros_node_exported(self):
        """SafetyValidatorROSNode is exported (may be None)."""
        assert hasattr(safety, "SafetyValidatorROSNode")
        # May be None if ROS2 not available


class TestAllExports:
    """Test __all__ exports."""

    def test_all_list_exists(self):
        """__all__ list is defined."""
        assert hasattr(safety, "__all__")
        assert isinstance(safety.__all__, list)
        assert len(safety.__all__) > 0

    def test_all_exports_available(self):
        """All items in __all__ are exported."""
        for name in safety.__all__:
            assert hasattr(safety, name), f"{name} not exported"
