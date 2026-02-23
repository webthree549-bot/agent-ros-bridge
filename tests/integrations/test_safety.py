"""Tests for SafetyManager."""

import pytest
import asyncio
from agent_ros_bridge.integrations.safety import SafetyManager, SafetyLevel


@pytest.fixture
def safety():
    """Create safety manager."""
    return SafetyManager()


class TestSafetyManager:
    """Test SafetyManager functionality."""
    
    def test_register_policy(self, safety):
        """Test policy registration."""
        safety.register_policy("move_arm", SafetyLevel.DANGEROUS)
        level = safety.get_safety_level("move_arm")
        assert level == SafetyLevel.DANGEROUS
    
    def test_requires_confirmation(self, safety):
        """Test confirmation requirement."""
        safety.register_policy("dangerous_action", SafetyLevel.DANGEROUS)
        safety.register_policy("safe_action", SafetyLevel.SAFE)
        
        assert safety.requires_confirmation("dangerous_action") is True
        assert safety.requires_confirmation("safe_action") is False
    
    @pytest.mark.asyncio
    async def test_confirmation_flow(self, safety):
        """Test confirmation request and approval."""
        request = await safety.request_confirmation(
            "test_action", "Test confirmation?"
        )
        
        assert request.id in safety.pending
        
        # Approve
        await safety.confirm(request.id)
        assert safety.pending[request.id].approved is True
    
    @pytest.mark.asyncio
    async def test_wait_for_confirmation(self, safety):
        """Test waiting for confirmation."""
        request = await safety.request_confirmation(
            "test_action", "Test?", timeout=2
        )
        
        # Approve after short delay
        async def approve_later():
            await asyncio.sleep(0.1)
            await safety.confirm(request.id)
        
        asyncio.create_task(approve_later())
        
        result = await safety.wait_for_confirmation(request.id)
        assert result is True
    
    def test_emergency_stop(self, safety):
        """Test emergency stop."""
        safety.register_policy("action", SafetyLevel.SAFE)
        
        # Before emergency
        assert safety.requires_confirmation("action") is False
        
        # Trigger emergency
        safety.trigger_emergency_stop("Test emergency")
        
        # Everything requires confirmation during emergency
        assert safety.requires_confirmation("action") is True
        
        # Clear emergency
        safety.clear_emergency_stop()
        assert safety.requires_confirmation("action") is False
