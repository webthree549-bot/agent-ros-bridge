"""Safety Manager - Action confirmation and emergency stop."""

import asyncio
import logging
from enum import Enum
from typing import Dict, List, Optional, Callable
from dataclasses import dataclass, field
from datetime import datetime

logger = logging.getLogger(__name__)


class SafetyLevel(Enum):
    """Safety classification for actions."""
    SAFE = "safe"           # No confirmation needed
    NORMAL = "normal"       # Optional confirmation
    DANGEROUS = "dangerous" # Always confirm


@dataclass
class SafetyPolicy:
    """Policy for an action."""
    action: str
    level: SafetyLevel
    description: str
    timeout_seconds: int = 30


@dataclass
class ConfirmationRequest:
    """Pending confirmation."""
    id: str
    action: str
    message: str
    timestamp: datetime
    timeout: int
    resolved: bool = False
    approved: bool = False


class SafetyManager:
    """Manages action safety and confirmations.
    
    Example:
        safety = SafetyManager()
        safety.register_policy("move_arm", SafetyLevel.DANGEROUS)
        
        if safety.requires_confirmation("move_arm"):
            request = await safety.request_confirmation(
                "move_arm", "Move arm to position X?"
            )
            approved = await safety.wait_for_confirmation(request.id)
    """
    
    def __init__(self):
        self.policies: Dict[str, SafetyPolicy] = {}
        self.pending: Dict[str, ConfirmationRequest] = {}
        self.audit_log: List[Dict] = []
        self.emergency_stop = False
        self._callbacks: List[Callable] = []
        logger.info("SafetyManager initialized")
    
    def register_policy(self, action: str, level: SafetyLevel, 
                       description: str = "", timeout: int = 30):
        """Register safety policy for an action."""
        self.policies[action] = SafetyPolicy(
            action=action,
            level=level,
            description=description,
            timeout_seconds=timeout
        )
        logger.debug(f"Registered policy: {action} -> {level.value}")
    
    def requires_confirmation(self, action: str) -> bool:
        """Check if action requires confirmation."""
        if self.emergency_stop:
            return True  # Everything requires confirmation during emergency
        
        policy = self.policies.get(action)
        if policy is None:
            return False  # Default: no confirmation
        
        return policy.level == SafetyLevel.DANGEROUS
    
    def get_safety_level(self, action: str) -> SafetyLevel:
        """Get safety level for an action."""
        policy = self.policies.get(action)
        if policy is None:
            return SafetyLevel.SAFE
        return policy.level
    
    async def request_confirmation(self, action: str, message: str,
                                   timeout: int = 30) -> ConfirmationRequest:
        """Request confirmation for an action."""
        import uuid
        
        request_id = str(uuid.uuid4())
        request = ConfirmationRequest(
            id=request_id,
            action=action,
            message=message,
            timestamp=datetime.now(),
            timeout=timeout
        )
        
        self.pending[request_id] = request
        
        # Log the request
        self.audit_log.append({
            "type": "confirmation_requested",
            "action": action,
            "message": message,
            "timestamp": datetime.now().isoformat()
        })
        
        logger.info(f"Confirmation requested for {action}: {request_id}")
        
        # Notify callbacks
        for callback in self._callbacks:
            try:
                callback(request)
            except Exception as e:
                logger.error(f"Callback error: {e}")
        
        return request
    
    async def confirm(self, request_id: str) -> bool:
        """Approve a confirmation request."""
        request = self.pending.get(request_id)
        if request is None:
            logger.warning(f"Confirmation request not found: {request_id}")
            return False
        
        request.resolved = True
        request.approved = True
        
        self.audit_log.append({
            "type": "confirmation_approved",
            "action": request.action,
            "request_id": request_id,
            "timestamp": datetime.now().isoformat()
        })
        
        logger.info(f"Confirmation approved: {request_id}")
        return True
    
    async def reject(self, request_id: str) -> bool:
        """Reject a confirmation request."""
        request = self.pending.get(request_id)
        if request is None:
            return False
        
        request.resolved = True
        request.approved = False
        
        self.audit_log.append({
            "type": "confirmation_rejected",
            "action": request.action,
            "request_id": request_id,
            "timestamp": datetime.now().isoformat()
        })
        
        logger.info(f"Confirmation rejected: {request_id}")
        return True
    
    async def wait_for_confirmation(self, request_id: str, 
                                   timeout: Optional[int] = None) -> bool:
        """Wait for confirmation with timeout."""
        request = self.pending.get(request_id)
        if request is None:
            return False
        
        wait_time = timeout or request.timeout
        
        for _ in range(wait_time * 10):  # Check every 100ms
            if request.resolved:
                return request.approved
            await asyncio.sleep(0.1)
        
        # Timeout - auto-reject
        await self.reject(request_id)
        return False
    
    def trigger_emergency_stop(self, reason: str = ""):
        """Trigger emergency stop."""
        self.emergency_stop = True
        self.audit_log.append({
            "type": "emergency_stop",
            "reason": reason,
            "timestamp": datetime.now().isoformat()
        })
        logger.critical(f"EMERGENCY STOP triggered: {reason}")
    
    def clear_emergency_stop(self):
        """Clear emergency stop after manual review."""
        self.emergency_stop = False
        self.audit_log.append({
            "type": "emergency_stop_cleared",
            "timestamp": datetime.now().isoformat()
        })
        logger.info("Emergency stop cleared")
    
    def get_audit_log(self) -> List[Dict]:
        """Get full audit log."""
        return self.audit_log.copy()
    
    def on_confirmation_request(self, callback: Callable):
        """Register callback for confirmation requests."""
        self._callbacks.append(callback)
