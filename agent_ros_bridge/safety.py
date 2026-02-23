"""Action confirmation system for safety-critical robot operations.

Provides human-in-the-loop confirmation for dangerous actions.

Example:
    from agent_ros_bridge.safety import Confirmation, ActionSafety
       
    @bridge.action("move_arm", safety_level="dangerous")
    async def move_arm(position: str, confirm: Confirmation):
        # Will prompt for confirmation before executing
        await confirm.request(
            message=f"Move arm to {position}?",
            timeout=30
        )
        
        # Proceed with action
        ...
"""

import asyncio
import logging
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Set
from dataclasses import dataclass, field
from datetime import datetime

from agent_ros_bridge import ROSBridge

logger = logging.getLogger(__name__)


class ConfirmationStatus(Enum):
    """Status of a confirmation request."""
    PENDING = "pending"
    CONFIRMED = "confirmed"
    REJECTED = "rejected"
    TIMEOUT = "timeout"
    CANCELLED = "cancelled"


class SafetyLevel(Enum):
    """Safety classification for actions."""
    SAFE = "safe"           # No confirmation needed
    MEDIUM = "medium"       # Confirmation for first use in session
    DANGEROUS = "dangerous" # Always confirm
    EMERGENCY = "emergency" # Require explicit override


@dataclass
class ConfirmationRequest:
    """A request for human confirmation."""
    id: str
    action: str
    message: str
    status: ConfirmationStatus
    created_at: datetime
    timeout: int
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    # Response tracking
    responded_at: Optional[datetime] = None
    responded_by: Optional[str] = None
    response_message: Optional[str] = None


class Confirmation:
    """Handle confirmation for a single action invocation.
    
    Usage:
        @bridge.action("dangerous_action", safety_level="dangerous")
        async def dangerous_action(param: str, confirm: Confirmation):
            await confirm.request(f"Execute with {param}?")
            # If we get here, user confirmed
    """
    
    def __init__(self, bridge: ROSBridge, action_name: str, session_id: str):
        self.bridge = bridge
        self.action_name = action_name
        self.session_id = session_id
        self.request: Optional[ConfirmationRequest] = None
        self._response_event = asyncio.Event()
    
    async def request(
        self,
        message: str,
        timeout: int = 30,
        metadata: Dict[str, Any] = None
    ) -> bool:
        """Request confirmation from user.
        
        Args:
            message: Human-readable confirmation message
            timeout: Seconds to wait for response
            metadata: Additional context
            
        Returns:
            True if confirmed, False if rejected/timeout
            
        Raises:
            ConfirmationRejected: If user rejects
            ConfirmationTimeout: If timeout
        """
        import uuid
        
        self.request = ConfirmationRequest(
            id=str(uuid.uuid4()),
            action=self.action_name,
            message=message,
            status=ConfirmationStatus.PENDING,
            created_at=datetime.utcnow(),
            timeout=timeout,
            metadata=metadata or {}
        )
        
        # Register with bridge's safety manager
        if hasattr(self.bridge, '_safety_manager'):
            self.bridge._safety_manager.register_request(self.request)
        
        logger.info(f"Confirmation requested: {message}")
        
        # Notify connected agents (dashboard, OpenClaw, etc.)
        await self._notify_agents()
        
        # Wait for response
        try:
            await asyncio.wait_for(
                self._response_event.wait(),
                timeout=timeout
            )
        except asyncio.TimeoutError:
            self.request.status = ConfirmationStatus.TIMEOUT
            raise ConfirmationTimeout(f"Confirmation timeout after {timeout}s")
        
        # Check result
        if self.request.status == ConfirmationStatus.CONFIRMED:
            return True
        elif self.request.status == ConfirmationStatus.REJECTED:
            raise ConfirmationRejected("Action rejected by user")
        else:
            raise ConfirmationCancelled("Action cancelled")
    
    async def _notify_agents(self):
        """Notify connected agents of confirmation request."""
        # This would send WebSocket messages, etc.
        # For now, just log
        logger.info(f"Notify: Confirmation request {self.request.id}")
    
    def confirm(self, user_id: str = "unknown", message: str = ""):
        """Mark request as confirmed."""
        if self.request:
            self.request.status = ConfirmationStatus.CONFIRMED
            self.request.responded_at = datetime.utcnow()
            self.request.responded_by = user_id
            self.request.response_message = message
            self._response_event.set()
            logger.info(f"Confirmed by {user_id}")
    
    def reject(self, user_id: str = "unknown", message: str = ""):
        """Mark request as rejected."""
        if self.request:
            self.request.status = ConfirmationStatus.REJECTED
            self.request.responded_at = datetime.utcnow()
            self.request.responded_by = user_id
            self.request.response_message = message
            self._response_event.set()
            logger.info(f"Rejected by {user_id}: {message}")
    
    def cancel(self):
        """Cancel the confirmation request."""
        if self.request:
            self.request.status = ConfirmationStatus.CANCELLED
            self._response_event.set()


class ConfirmationRejected(Exception):
    """Raised when user rejects confirmation."""
    pass


class ConfirmationTimeout(Exception):
    """Raised when confirmation times out."""
    pass


class ConfirmationCancelled(Exception):
    """Raised when confirmation is cancelled."""
    pass


class ActionSafety:
    """Manage action safety levels and confirmations.
    
    Provides:
    - Safety level classification
    - Confirmation management
    - Audit logging
    - Emergency stop
    """
    
    def __init__(self, bridge: ROSBridge):
        self.bridge = bridge
        self._action_safety: Dict[str, SafetyLevel] = {}
        self._confirmed_this_session: Set[str] = set()
        self._pending_confirmations: Dict[str, ConfirmationRequest] = {}
        self._emergency_stop = False
        
        # Default safety levels
        self._default_levels = {
            # Dangerous - always confirm
            "navigate": SafetyLevel.DANGEROUS,
            "move_arm": SafetyLevel.DANGEROUS,
            "grasp": SafetyLevel.DANGEROUS,
            "emergency_stop": SafetyLevel.EMERGENCY,
            
            # Medium - confirm first time
            "patrol": SafetyLevel.MEDIUM,
            "rotate": SafetyLevel.MEDIUM,
            
            # Safe - no confirmation
            "get_status": SafetyLevel.SAFE,
            "get_battery": SafetyLevel.SAFE,
            "get_position": SafetyLevel.SAFE,
        }
    
    def set_safety_level(self, action: str, level: SafetyLevel):
        """Set safety level for an action.
        
        Args:
            action: Action name
            level: Safety level
        """
        self._action_safety[action] = level
        logger.info(f"Set safety level for {action}: {level.value}")
    
    def get_safety_level(self, action: str) -> SafetyLevel:
        """Get safety level for an action.
        
        Args:
            action: Action name
            
        Returns:
            Safety level
        """
        # Check explicit setting
        if action in self._action_safety:
            return self._action_safety[action]
        
        # Check defaults
        if action in self._default_levels:
            return self._default_levels[action]
        
        # Check by name patterns
        dangerous_keywords = ["move", "nav", "grip", "grasp", "pick", "place", "arm"]
        for kw in dangerous_keywords:
            if kw in action.lower():
                return SafetyLevel.DANGEROUS
        
        return SafetyLevel.SAFE
    
    def needs_confirmation(self, action: str, session_id: str) -> bool:
        """Check if action needs confirmation.
        
        Args:
            action: Action name
            session_id: Agent session ID
            
        Returns:
            True if confirmation required
        """
        if self._emergency_stop:
            return True  # Everything needs confirmation during emergency
        
        level = self.get_safety_level(action)
        
        if level == SafetyLevel.SAFE:
            return False
        
        if level == SafetyLevel.DANGEROUS:
            return True
        
        if level == SafetyLevel.MEDIUM:
            # Confirm once per session
            session_action = f"{session_id}:{action}"
            return session_action not in self._confirmed_this_session
        
        return True
    
    def mark_confirmed(self, action: str, session_id: str):
        """Mark action as confirmed for this session.
        
        Args:
            action: Action name
            session_id: Agent session ID
        """
        session_action = f"{session_id}:{action}"
        self._confirmed_this_session.add(session_action)
    
    def register_request(self, request: ConfirmationRequest):
        """Register a confirmation request."""
        self._pending_confirmations[request.id] = request
    
    def get_pending_confirmations(self) -> List[ConfirmationRequest]:
        """Get all pending confirmation requests."""
        return [
            req for req in self._pending_confirmations.values()
            if req.status == ConfirmationStatus.PENDING
        ]
    
    def confirm(self, request_id: str, user_id: str = "unknown"):
        """Confirm a pending request."""
        if request_id in self._pending_confirmations:
            req = self._pending_confirmations[request_id]
            req.status = ConfirmationStatus.CONFIRMED
            req.responded_at = datetime.utcnow()
            req.responded_by = user_id
            logger.info(f"Confirmed {request_id} by {user_id}")
    
    def reject(self, request_id: str, user_id: str = "unknown", reason: str = ""):
        """Reject a pending request."""
        if request_id in self._pending_confirmations:
            req = self._pending_confirmations[request_id]
            req.status = ConfirmationStatus.REJECTED
            req.responded_at = datetime.utcnow()
            req.responded_by = user_id
            req.response_message = reason
            logger.info(f"Rejected {request_id} by {user_id}: {reason}")
    
    def emergency_stop(self):
        """Trigger emergency stop."""
        self._emergency_stop = True
        logger.critical("EMERGENCY STOP ACTIVATED")
        
        # Cancel all pending actions
        for req in self._pending_confirmations.values():
            if req.status == ConfirmationStatus.PENDING:
                req.status = ConfirmationStatus.CANCELLED
    
    def emergency_clear(self):
        """Clear emergency stop."""
        self._emergency_stop = False
        logger.info("Emergency stop cleared")
    
    def is_emergency_stopped(self) -> bool:
        """Check if emergency stop is active."""
        return self._emergency_stop
    
    def get_safety_report(self) -> Dict[str, Any]:
        """Generate safety status report."""
        return {
            "emergency_stop": self._emergency_stop,
            "pending_confirmations": len(self.get_pending_confirmations()),
            "confirmed_this_session": len(self._confirmed_this_session),
            "action_safety_levels": {
                action: level.value
                for action, level in self._action_safety.items()
            }
        }


# Decorator for action safety
def safety_level(level: SafetyLevel):
    """Decorator to set safety level for an action.
    
    Example:
        @bridge.action("move_arm")
        @safety_level(SafetyLevel.DANGEROUS)
        async def move_arm(position: str, confirm: Confirmation):
            await confirm.request(f"Move arm to {position}?")
            # ...
    """
    def decorator(func: Callable):
        func._safety_level = level
        return func
    return decorator


def confirm_dangerous(message: Optional[str] = None, timeout: int = 30):
    """Decorator to require confirmation for dangerous actions.
    
    Example:
        @bridge.action("navigate")
        @confirm_dangerous("Navigate to new position?")
        async def navigate(x: float, y: float, confirm: Confirmation):
            # Will prompt for confirmation
            pass
    """
    def decorator(func: Callable):
        async def wrapper(*args, **kwargs):
            # Extract confirmation from kwargs
            confirm = kwargs.get('confirm')
            if confirm is None:
                raise ValueError("Action decorated with @confirm_dangerous must accept 'confirm' parameter")
            
            # Request confirmation
            await confirm.request(
                message or f"Execute {func.__name__}?",
                timeout=timeout
            )
            
            # Proceed with action
            return await func(*args, **kwargs)
        
        wrapper._safety_level = SafetyLevel.DANGEROUS
        return wrapper
    return decorator


__all__ = [
    "Confirmation",
    "ConfirmationRequest",
    "ConfirmationStatus",
    "ActionSafety",
    "SafetyLevel",
    "ConfirmationRejected",
    "ConfirmationTimeout",
    "ConfirmationCancelled",
    "safety_level",
    "confirm_dangerous"
]
