"""
/safety/watchdog Node
Agent ROS Bridge v0.6.1 - Week 2 Implementation

Monitors system health and triggers failsafe actions.
1kHz heartbeat monitoring with 1ms timeout.
"""

import time
import threading
from typing import Dict, List, Optional, Any, Callable, Set


class WatchdogNode:
    """
    Watchdog Node

    Monitors critical nodes via heartbeat messages.
    Auto-triggers emergency stop on timeout.

    Timing Requirements:
    - Heartbeat frequency: 1kHz (1ms period)
    - Timeout threshold: 1ms (1 missed heartbeat)
    """

    # Constants
    MONITORING_FREQUENCY_HZ = 1000  # 1kHz
    MONITORING_PERIOD_MS = 1.0  # 1ms
    TIMEOUT_THRESHOLD_MS = 1.0  # 1ms timeout

    # Topics
    HEARTBEAT_TOPIC = "/safety/watchdog_heartbeat"
    STATUS_TOPIC = "/safety/watchdog_status"

    def __init__(self):
        """Initialize Watchdog Node"""
        self._monitored_nodes: Dict[str, Dict[str, Any]] = {}
        self._heartbeats: Dict[str, Dict[str, Any]] = {}
        self._timeout_log: List[Dict[str, Any]] = []
        self._lock = threading.Lock()
        self._seq = 0

        # Callbacks
        self.heartbeat_publisher: Optional[Callable] = None
        self.status_publisher: Optional[Callable] = None
        self.emergency_stop_callback: Optional[Callable] = None

    def register_node(self, node_id: str, critical: bool = True) -> None:
        """
        Register a node for monitoring

        Args:
            node_id: Unique node identifier
            critical: Whether node is critical (triggers e-stop on timeout)
        """
        with self._lock:
            self._monitored_nodes[node_id] = {"critical": critical, "registered_at": time.time()}
            self._heartbeats[node_id] = {"seq": 0, "timestamp": 0, "status": 0}

    def unregister_node(self, node_id: str) -> None:
        """
        Unregister a node from monitoring

        Args:
            node_id: Node to unregister
        """
        with self._lock:
            self._monitored_nodes.pop(node_id, None)
            self._heartbeats.pop(node_id, None)

    def process_heartbeat(self, node_id: str, seq: int, timestamp: float, status: int = 0) -> None:
        """
        Process a heartbeat from a monitored node

        Args:
            node_id: Node that sent heartbeat
            seq: Sequence number
            timestamp: Heartbeat timestamp
            status: Node status (0=OK, 1=WARNING, 2=ERROR)
        """
        with self._lock:
            if node_id not in self._monitored_nodes:
                return

            self._heartbeats[node_id] = {
                "seq": seq,
                "timestamp": timestamp,
                "status": status,
                "received_at": time.time(),
            }

    def check_timeouts(self) -> List[str]:
        """
        Check for timed out nodes

        Returns:
            List of node IDs that have timed out
        """
        timed_out = []
        current_time = time.time()

        with self._lock:
            for node_id, node_info in self._monitored_nodes.items():
                heartbeat = self._heartbeats.get(node_id, {})
                last_timestamp = heartbeat.get("timestamp", 0)

                # Calculate time since last heartbeat
                elapsed_ms = (current_time - last_timestamp) * 1000

                if elapsed_ms > self.TIMEOUT_THRESHOLD_MS:
                    timed_out.append(node_id)

                    # Log timeout
                    timeout_event = {
                        "timestamp": current_time,
                        "node": node_id,
                        "elapsed_ms": elapsed_ms,
                        "critical": node_info.get("critical", False),
                    }
                    self._timeout_log.append(timeout_event)

                    # Trigger e-stop for critical nodes
                    if node_info.get("critical", False) and self.emergency_stop_callback:
                        self.emergency_stop_callback(
                            reason=f"Watchdog timeout: {node_id}", source="watchdog"
                        )

        return timed_out

    def create_heartbeat(self) -> Dict[str, Any]:
        """
        Create a watchdog heartbeat message

        Returns:
            Heartbeat message dictionary
        """
        self._seq += 1
        return {
            "seq": self._seq,
            "timestamp": time.time(),
            "source": "/safety/watchdog",
            "status": 0,  # OK
        }

    def publish_heartbeat(self) -> None:
        """Publish watchdog heartbeat"""
        heartbeat = self.create_heartbeat()
        if self.heartbeat_publisher:
            self.heartbeat_publisher(heartbeat)

    def get_status(self) -> Dict[str, Any]:
        """
        Get watchdog status

        Returns:
            Status dictionary
        """
        with self._lock:
            return {
                "monitored_nodes": list(self._monitored_nodes.keys()),
                "node_count": len(self._monitored_nodes),
                "timestamp": time.time(),
            }

    def publish_status(self) -> None:
        """Publish watchdog status"""
        status = self.get_status()
        if self.status_publisher:
            self.status_publisher(status)

    @property
    def monitored_nodes(self) -> Set[str]:
        """Get set of monitored node IDs"""
        with self._lock:
            return set(self._monitored_nodes.keys())

    @property
    def heartbeats(self) -> Dict[str, Dict[str, Any]]:
        """Get heartbeat data (copy)"""
        with self._lock:
            return self._heartbeats.copy()

    @property
    def timeout_log(self) -> List[Dict[str, Any]]:
        """Get timeout log (copy)"""
        with self._lock:
            return self._timeout_log.copy()

    def get_node_health(self, node_id: str) -> Optional[Dict[str, Any]]:
        """
        Get health status for a specific node

        Args:
            node_id: Node to check

        Returns:
            Health status dictionary or None
        """
        with self._lock:
            if node_id not in self._monitored_nodes:
                return None

            heartbeat = self._heartbeats.get(node_id, {})
            last_timestamp = heartbeat.get("timestamp", 0)
            elapsed_ms = (time.time() - last_timestamp) * 1000

            return {
                "node_id": node_id,
                "last_heartbeat": last_timestamp,
                "elapsed_ms": elapsed_ms,
                "timed_out": elapsed_ms > self.TIMEOUT_THRESHOLD_MS,
                "status": heartbeat.get("status", 0),
            }
