"""Unit tests for Safety and Confirmation."""

import pytest
import asyncio

from agent_ros_bridge.safety import (
    Confirmation, ActionSafety, SafetyLevel,
    ConfirmationStatus, ConfirmationRejected, ConfirmationTimeout
)
from agent_ros_bridge import ROSBridge


@pytest.mark.unit
class TestActionSafety:
    """Test ActionSafety functionality."""
    
    @pytest.fixture
    def safety(self):
        """Create safety manager."""
        bridge = ROSBridge(ros_version=2)
        return ActionSafety(bridge)
    
    def test_default_safety_levels(self, safety):
        """Test default safety level assignments."""
        assert safety.get_safety_level("navigate") == SafetyLevel.DANGEROUS
        assert safety.get_safety_level("get_battery") == SafetyLevel.SAFE
        assert safety.get_safety_level("patrol") == SafetyLevel.MEDIUM
    
    def test_set_safety_level(self, safety):
        """Test setting custom safety level."""
        safety.set_safety_level("custom_action", SafetyLevel.DANGEROUS)
        assert safety.get_safety_level("custom_action") == SafetyLevel.DANGEROUS
    
    def test_needs_confirmation_dangerous(self, safety):
        """Test confirmation requirement for dangerous actions."""
        assert safety.needs_confirmation("navigate", "session1") is True
        assert safety.needs_confirmation("navigate", "session2") is True
    
    def test_needs_confirmation_safe(self, safety):
        """Test no confirmation for safe actions."""
        assert safety.needs_confirmation("get_status", "session1") is False
    
    def test_needs_confirmation_medium_first_time(self, safety):
        """Test confirmation for medium action first time."""
        assert safety.needs_confirmation("patrol", "session1") is True
        safety.mark_confirmed("patrol", "session1")
        assert safety.needs_confirmation("patrol", "session1") is False
    
    def test_emergency_stop(self, safety):
        """Test emergency stop functionality."""
        assert safety.is_emergency_stopped() is False
        safety.emergency_stop()
        assert safety.is_emergency_stopped() is True
        # Everything needs confirmation during emergency
        assert safety.needs_confirmation("get_status", "session1") is True
    
    def test_emergency_clear(self, safety):
        """Test clearing emergency stop."""
        safety.emergency_stop()
        safety.emergency_clear()
        assert safety.is_emergency_stopped() is False
    
    def test_get_safety_report(self, safety):
        """Test safety report generation."""
        report = safety.get_safety_report()
        assert "emergency_stop" in report
        assert "pending_confirmations" in report


@pytest.mark.unit
class TestConfirmation:
    """Test Confirmation functionality."""
    
    @pytest.fixture
    def confirmation(self):
        """Create confirmation instance."""
        bridge = ROSBridge(ros_version=2)
        return Confirmation(bridge, "test_action", "session1")
    
    @pytest.mark.asyncio
    async def test_confirmation_request(self, confirmation):
        """Test confirmation request."""
        # Simulate confirmation
        asyncio.create_task(self._delayed_confirm(confirmation))
        
        result = await confirmation.request("Test message?", timeout=2)
        assert result is True
        assert confirmation.request.status == ConfirmationStatus.CONFIRMED
    
    @pytest.mark.asyncio
    async def test_confirmation_reject(self, confirmation):
        """Test confirmation rejection."""
        asyncio.create_task(self._delayed_reject(confirmation))
        
        with pytest.raises(ConfirmationRejected):
            await confirmation.request("Test message?", timeout=2)
    
    @pytest.mark.asyncio
    async def test_confirmation_timeout(self, confirmation):
        """Test confirmation timeout."""
        with pytest.raises(ConfirmationTimeout):
            await confirmation.request("Test message?", timeout=0.1)
    
    async def _delayed_confirm(self, confirmation, delay: float = 0.05):
        """Helper to confirm after delay."""
        await asyncio.sleep(delay)
        confirmation.confirm("test_user")
    
    async def _delayed_reject(self, confirmation, delay: float = 0.05):
        """Helper to reject after delay."""
        await asyncio.sleep(delay)
        confirmation.reject("test_user", "No thanks")


@pytest.mark.unit
class TestSafetyLevelEnum:
    """Test SafetyLevel enum."""
    
    def test_safety_values(self):
        """Test safety level values."""
        assert SafetyLevel.SAFE.value == "safe"
        assert SafetyLevel.MEDIUM.value == "medium"
        assert SafetyLevel.DANGEROUS.value == "dangerous"
        assert SafetyLevel.EMERGENCY.value == "emergency"
