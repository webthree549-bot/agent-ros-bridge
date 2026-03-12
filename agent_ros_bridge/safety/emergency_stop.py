"""
/safety/emergency_stop Node
Agent ROS Bridge v0.6.1 - Week 2 Implementation

Central emergency stop coordination.
Target: <50ms response time from trigger to motor cutoff
"""

import time
import threading
from typing import Dict, List, Optional, Any, Callable


class EmergencyStopNode:
    """
    Emergency Stop Node

    Monitors emergency stop triggers from multiple sources.
    Executes immediate stop commands.
    Manages emergency stop state machine.

    Timing Requirements:
    - E-stop activation latency: <50ms from trigger to motor cutoff
    """

    # Service names
    TRIGGER_SERVICE = "/safety/trigger_emergency_stop"
    CLEAR_SERVICE = "/safety/clear_emergency_stop"

    # Status topic
    STATUS_TOPIC = "/safety/emergency_stop_status"

    def __init__(self, auth_code: str = "valid_auth_code"):
        """
        Initialize Emergency Stop Node

        Args:
            auth_code: Authorization code for clearing e-stop
        """
        self._emergency_stopped = False
        self._auth_code = auth_code
        self._trigger_history: List[Dict[str, Any]] = []
        self._lock = threading.Lock()

        # Callbacks
        self.status_publisher: Optional[Callable] = None
        self.motor_cutoff_callback: Optional[Callable] = None
        self.hardware_estop_callback: Optional[Callable] = None

    def trigger_emergency_stop(self, reason: str, source: str) -> bool:
        """
        Trigger emergency stop

        Args:
            reason: Reason for e-stop
            source: Source of trigger (e.g., 'manual', 'watchdog', 'validator')

        Returns:
            True if trigger was successful

        Timing: Must complete in <50ms
        """
        start_time = time.time()

        with self._lock:
            # Set e-stop state
            self._emergency_stopped = True

            # Log trigger
            trigger_event = {
                "timestamp": start_time,
                "reason": reason,
                "source": source,
                "latency_ms": 0,  # Will be updated
            }
            self._trigger_history.append(trigger_event)

            # Execute motor cutoff
            if self.motor_cutoff_callback:
                self.motor_cutoff_callback()

            # Trigger hardware e-stop if available
            if self.hardware_estop_callback:
                self.hardware_estop_callback()

            # Publish status
            if self.status_publisher:
                self.status_publisher(self._get_status())

            # Update latency
            elapsed_ms = (time.time() - start_time) * 1000
            trigger_event["latency_ms"] = elapsed_ms

        return True

    def clear_emergency_stop(self, auth_code: Optional[str] = None) -> bool:
        """
        Clear emergency stop (requires authorization)

        Args:
            auth_code: Authorization code

        Returns:
            True if cleared successfully, False otherwise
        """
        with self._lock:
            # Check authorization
            if auth_code != self._auth_code:
                return False

            # Clear e-stop state
            self._emergency_stopped = False

            # Publish status
            if self.status_publisher:
                self.status_publisher(self._get_status())

            return True

    def attempt_override(self) -> bool:
        """
        Attempt to override e-stop via software

        Returns:
            False - software override is never allowed
        """
        # Software override is never allowed
        return False

    def is_emergency_stopped(self) -> bool:
        """
        Check if emergency stop is active

        Returns:
            True if e-stop is active
        """
        with self._lock:
            return self._emergency_stopped

    def _get_status(self) -> Dict[str, Any]:
        """Get current e-stop status (assumes lock is held by caller)"""
        return {
            "emergency_stopped": self._emergency_stopped,
            "timestamp": time.time(),
            "trigger_count": len(self._trigger_history),
        }

    @property
    def trigger_history(self) -> List[Dict[str, Any]]:
        """Get history of e-stop triggers"""
        with self._lock:
            return self._trigger_history.copy()

    def get_last_trigger(self) -> Optional[Dict[str, Any]]:
        """Get the last trigger event"""
        with self._lock:
            if self._trigger_history:
                return self._trigger_history[-1]
            return None

    def get_trigger_stats(self) -> Dict[str, Any]:
        """Get trigger statistics"""
        with self._lock:
            if not self._trigger_history:
                return {"total_triggers": 0, "average_latency_ms": 0, "max_latency_ms": 0}

            latencies = [t["latency_ms"] for t in self._trigger_history]
            return {
                "total_triggers": len(self._trigger_history),
                "average_latency_ms": sum(latencies) / len(latencies),
                "max_latency_ms": max(latencies),
            }
