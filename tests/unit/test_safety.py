"""
Unit tests for Safety Manager.
Tests action confirmation, emergency stop, and audit logging.
Uses synchronous wrappers for async methods.
"""

import asyncio
import pytest
from datetime import datetime
from unittest import mock

from agent_ros_bridge.integrations.safety import (
    SafetyManager, SafetyLevel, SafetyPolicy, ConfirmationRequest
)


def run_async(coro):
    """Helper to run async function synchronously"""
    return asyncio.run(coro)


class TestSafetyPolicy:
    """Test SafetyPolicy dataclass"""
    
    def test_policy_creation(self):
        """Test basic policy creation"""
        policy = SafetyPolicy(
            action="move_arm",
            level=SafetyLevel.DANGEROUS,
            description="Move robot arm",
            timeout_seconds=30
        )
        
        assert policy.action == "move_arm"
        assert policy.level == SafetyLevel.DANGEROUS
        assert policy.description == "Move robot arm"
        assert policy.timeout_seconds == 30


class TestSafetyManagerBasics:
    """Test SafetyManager basic functionality"""
    
    @pytest.fixture
    def safety(self):
        """Create fresh SafetyManager"""
        return SafetyManager()
    
    def test_initialization(self, safety):
        """Test SafetyManager initializes correctly"""
        assert safety.policies == {}
        assert safety.pending == {}
        assert safety.audit_log == []
        assert safety.emergency_stop is False
        assert safety._callbacks == []
    
    def test_register_policy(self, safety):
        """Test policy registration"""
        safety.register_policy(
            "move_arm",
            SafetyLevel.DANGEROUS,
            "Move robot arm to position",
            timeout=60
        )
        
        assert "move_arm" in safety.policies
        policy = safety.policies["move_arm"]
        assert policy.level == SafetyLevel.DANGEROUS
        assert policy.description == "Move robot arm to position"
        assert policy.timeout_seconds == 60
    
    def test_register_multiple_policies(self, safety):
        """Test registering multiple policies"""
        safety.register_policy("safe_action", SafetyLevel.SAFE)
        safety.register_policy("normal_action", SafetyLevel.NORMAL)
        safety.register_policy("dangerous_action", SafetyLevel.DANGEROUS)
        
        assert len(safety.policies) == 3
        assert safety.policies["safe_action"].level == SafetyLevel.SAFE
        assert safety.policies["normal_action"].level == SafetyLevel.NORMAL
        assert safety.policies["dangerous_action"].level == SafetyLevel.DANGEROUS


class TestSafetyLevelChecks:
    """Test safety level checking"""
    
    @pytest.fixture
    def safety(self):
        """Create SafetyManager with policies"""
        sm = SafetyManager()
        sm.register_policy("safe_action", SafetyLevel.SAFE)
        sm.register_policy("normal_action", SafetyLevel.NORMAL)
        sm.register_policy("dangerous_action", SafetyLevel.DANGEROUS)
        return sm
    
    def test_requires_confirmation_safe(self, safety):
        """Test SAFE level doesn't require confirmation"""
        assert safety.requires_confirmation("safe_action") is False
    
    def test_requires_confirmation_normal(self, safety):
        """Test NORMAL level doesn't require confirmation by default"""
        assert safety.requires_confirmation("normal_action") is False
    
    def test_requires_confirmation_dangerous(self, safety):
        """Test DANGEROUS level requires confirmation"""
        assert safety.requires_confirmation("dangerous_action") is True
    
    def test_requires_confirmation_unknown(self, safety):
        """Test unknown action defaults to no confirmation"""
        assert safety.requires_confirmation("unknown_action") is False
    
    def test_get_safety_level(self, safety):
        """Test getting safety level"""
        assert safety.get_safety_level("safe_action") == SafetyLevel.SAFE
        assert safety.get_safety_level("normal_action") == SafetyLevel.NORMAL
        assert safety.get_safety_level("dangerous_action") == SafetyLevel.DANGEROUS
    
    def test_get_safety_level_unknown(self, safety):
        """Test unknown action defaults to SAFE"""
        assert safety.get_safety_level("unknown") == SafetyLevel.SAFE


class TestEmergencyStop:
    """Test emergency stop functionality"""
    
    @pytest.fixture
    def safety(self):
        """Create SafetyManager with policies"""
        sm = SafetyManager()
        sm.register_policy("safe_action", SafetyLevel.SAFE)
        sm.register_policy("dangerous_action", SafetyLevel.DANGEROUS)
        return sm
    
    def test_trigger_emergency_stop(self, safety):
        """Test emergency stop triggers"""
        safety.trigger_emergency_stop("Test emergency")
        
        assert safety.emergency_stop is True
        assert len(safety.audit_log) == 1
        assert safety.audit_log[0]["type"] == "emergency_stop"
        assert safety.audit_log[0]["reason"] == "Test emergency"
    
    def test_emergency_stop_requires_all_confirmations(self, safety):
        """Test emergency stop makes all actions require confirmation"""
        # Before emergency stop
        assert safety.requires_confirmation("safe_action") is False
        
        # Trigger emergency stop
        safety.trigger_emergency_stop()
        
        # After emergency stop - everything requires confirmation
        assert safety.requires_confirmation("safe_action") is True
        assert safety.requires_confirmation("dangerous_action") is True
        assert safety.requires_confirmation("unknown_action") is True
    
    def test_clear_emergency_stop(self, safety):
        """Test clearing emergency stop"""
        safety.trigger_emergency_stop("Test")
        assert safety.emergency_stop is True
        
        safety.clear_emergency_stop()
        
        assert safety.emergency_stop is False
        assert safety.audit_log[-1]["type"] == "emergency_stop_cleared"
    
    def test_confirmation_requirements_after_clear(self, safety):
        """Test confirmation requirements return to normal after clear"""
        safety.trigger_emergency_stop()
        safety.clear_emergency_stop()
        
        # Should return to normal
        assert safety.requires_confirmation("safe_action") is False
        assert safety.requires_confirmation("dangerous_action") is True


class TestConfirmationRequests:
    """Test confirmation request flow"""
    
    @pytest.fixture
    def safety(self):
        """Create SafetyManager"""
        return SafetyManager()
    
    def test_request_confirmation(self, safety):
        """Test creating confirmation request"""
        request = run_async(safety.request_confirmation(
            "move_arm",
            "Move arm to position X?",
            timeout=30
        ))
        
        assert isinstance(request, ConfirmationRequest)
        assert request.action == "move_arm"
        assert request.message == "Move arm to position X?"
        assert request.timeout == 30
        assert request.resolved is False
        assert request.id in safety.pending
    
    def test_confirm_request(self, safety):
        """Test confirming a request"""
        request = run_async(safety.request_confirmation("move_arm", "Move?"))
        
        result = run_async(safety.confirm(request.id))
        
        assert result is True
        assert safety.pending[request.id].resolved is True
        assert safety.pending[request.id].approved is True
    
    def test_reject_request(self, safety):
        """Test rejecting a request"""
        request = run_async(safety.request_confirmation("move_arm", "Move?"))
        
        result = run_async(safety.reject(request.id))
        
        assert result is True
        assert safety.pending[request.id].resolved is True
        assert safety.pending[request.id].approved is False
    
    def test_confirm_nonexistent_request(self, safety):
        """Test confirming non-existent request"""
        result = run_async(safety.confirm("nonexistent-id"))
        
        assert result is False
    
    def test_reject_nonexistent_request(self, safety):
        """Test rejecting non-existent request"""
        result = run_async(safety.reject("nonexistent-id"))
        
        assert result is False
    
    def test_wait_for_confirmation_approved(self, safety):
        """Test waiting for confirmation - approved case"""
        request = run_async(safety.request_confirmation("move_arm", "Move?"))
        
        # Confirm in background
        async def confirm_later():
            await asyncio.sleep(0.05)
            await safety.confirm(request.id)
        
        async def wait_and_confirm():
            task = asyncio.create_task(confirm_later())
            result = await safety.wait_for_confirmation(request.id, timeout=5)
            await task
            return result
        
        result = run_async(wait_and_confirm())
        
        assert result is True
    
    def test_wait_for_confirmation_rejected(self, safety):
        """Test waiting for confirmation - rejected case"""
        request = run_async(safety.request_confirmation("move_arm", "Move?"))
        
        # Reject in background
        async def reject_later():
            await asyncio.sleep(0.05)
            await safety.reject(request.id)
        
        async def wait_and_reject():
            task = asyncio.create_task(reject_later())
            result = await safety.wait_for_confirmation(request.id, timeout=5)
            await task
            return result
        
        result = run_async(wait_and_reject())
        
        assert result is False
    
    def test_wait_for_confirmation_timeout(self, safety):
        """Test waiting for confirmation - timeout case"""
        request = run_async(safety.request_confirmation("move_arm", "Move?"))
        
        # Don't confirm - should timeout
        result = run_async(safety.wait_for_confirmation(request.id, timeout=0.1))
        
        assert result is False
        assert safety.pending[request.id].resolved is True
        assert safety.pending[request.id].approved is False


class TestAuditLogging:
    """Test audit logging functionality"""
    
    @pytest.fixture
    def safety(self):
        """Create SafetyManager"""
        return SafetyManager()
    
    def test_confirmation_request_logged(self, safety):
        """Test confirmation requests are logged"""
        run_async(safety.request_confirmation("move_arm", "Move?"))
        
        log = safety.get_audit_log()
        assert len(log) == 1
        assert log[0]["type"] == "confirmation_requested"
        assert log[0]["action"] == "move_arm"
        assert "timestamp" in log[0]
    
    def test_confirmation_approved_logged(self, safety):
        """Test approval is logged"""
        request = run_async(safety.request_confirmation("move_arm", "Move?"))
        run_async(safety.confirm(request.id))
        
        log = safety.get_audit_log()
        approval_entries = [e for e in log if e["type"] == "confirmation_approved"]
        assert len(approval_entries) == 1
        assert approval_entries[0]["action"] == "move_arm"
    
    def test_confirmation_rejected_logged(self, safety):
        """Test rejection is logged"""
        request = run_async(safety.request_confirmation("move_arm", "Move?"))
        run_async(safety.reject(request.id))
        
        log = safety.get_audit_log()
        rejection_entries = [e for e in log if e["type"] == "confirmation_rejected"]
        assert len(rejection_entries) == 1
    
    def test_emergency_stop_logged(self, safety):
        """Test emergency stop is logged"""
        safety.trigger_emergency_stop("Test reason")
        
        log = safety.get_audit_log()
        assert len(log) == 1
        assert log[0]["type"] == "emergency_stop"
        assert log[0]["reason"] == "Test reason"
    
    def test_audit_log_is_copy(self, safety):
        """Test get_audit_log returns a copy"""
        safety.trigger_emergency_stop("Test")
        
        log1 = safety.get_audit_log()
        log1.clear()  # Modify the returned copy
        
        log2 = safety.get_audit_log()
        assert len(log2) == 1  # Original should be unchanged


class TestCallbacks:
    """Test callback functionality"""
    
    @pytest.fixture
    def safety(self):
        """Create SafetyManager"""
        return SafetyManager()
    
    def test_callback_registered(self, safety):
        """Test callback registration"""
        callback_called = False
        received_request = None
        
        def callback(request):
            nonlocal callback_called, received_request
            callback_called = True
            received_request = request
        
        safety.on_confirmation_request(callback)
        
        request = run_async(safety.request_confirmation("move_arm", "Move?"))
        
        assert callback_called is True
        assert received_request.id == request.id
    
    def test_multiple_callbacks(self, safety):
        """Test multiple callbacks are called"""
        call_count = 0
        
        def callback1(request):
            nonlocal call_count
            call_count += 1
        
        def callback2(request):
            nonlocal call_count
            call_count += 1
        
        safety.on_confirmation_request(callback1)
        safety.on_confirmation_request(callback2)
        
        run_async(safety.request_confirmation("move_arm", "Move?"))
        
        assert call_count == 2
    
    def test_callback_error_handling(self, safety):
        """Test callback errors don't break other callbacks"""
        good_callback_called = False
        
        def bad_callback(request):
            raise Exception("Callback error")
        
        def good_callback(request):
            nonlocal good_callback_called
            good_callback_called = True
        
        safety.on_confirmation_request(bad_callback)
        safety.on_confirmation_request(good_callback)
        
        # Should not raise
        run_async(safety.request_confirmation("move_arm", "Move?"))
        
        assert good_callback_called is True


class TestEdgeCases:
    """Test edge cases and error handling"""
    
    @pytest.fixture
    def safety(self):
        """Create SafetyManager"""
        return SafetyManager()
    
    def test_wait_for_nonexistent_request(self, safety):
        """Test waiting for non-existent request"""
        result = run_async(safety.wait_for_confirmation("nonexistent", timeout=0.1))
        
        assert result is False
    
    def test_get_audit_log_empty(self, safety):
        """Test getting empty audit log"""
        log = safety.get_audit_log()
        
        assert log == []
    
    def test_multiple_confirmations_same_action(self, safety):
        """Test multiple confirmation requests for same action"""
        request1 = run_async(safety.request_confirmation("move_arm", "Move 1?"))
        request2 = run_async(safety.request_confirmation("move_arm", "Move 2?"))
        
        assert request1.id != request2.id
        assert len(safety.pending) == 2
    
    def test_double_confirm(self, safety):
        """Test confirming already confirmed request"""
        request = run_async(safety.request_confirmation("move_arm", "Move?"))
        
        result1 = run_async(safety.confirm(request.id))
        result2 = run_async(safety.confirm(request.id))
        
        assert result1 is True
        assert result2 is True  # Should still return True
