"""Dashboard API for shadow mode metrics.

Provides HTTP endpoints and WebSocket for real-time metrics.
"""

from datetime import UTC, datetime
from typing import Any, Callable

from .comparator import DecisionComparator
from .decision_logger import DecisionLogger


class DashboardAPI:
    """Dashboard API for shadow mode metrics.

    Usage:
        dashboard = DashboardAPI()
        metrics = dashboard.get_metrics()
        decisions = dashboard.get_recent_decisions(limit=10)
    """

    def __init__(
        self,
        logger: DecisionLogger | None = None,
        comparator: DecisionComparator | None = None,
    ):
        """Initialize dashboard API.

        Args:
            logger: DecisionLogger instance (creates default if None)
            comparator: DecisionComparator instance (creates default if None)
        """
        self._logger = logger or DecisionLogger()
        self._comparator = comparator or DecisionComparator()
        self._subscribers: list[Callable[[dict], None]] = []

    def get_metrics(self) -> dict[str, Any]:
        """Get current metrics.

        Returns:
            Dictionary with agreement_rate, total_decisions, pending_count
        """
        # Get logger stats
        stats = self._logger.get_stats()

        # Calculate agreement rate from completed decisions
        # For now, return placeholder metrics
        return {
            "agreement_rate": 0.0,
            "total_decisions": stats.get("total_logged", 0),
            "pending_count": stats.get("pending_count", 0),
            "timestamp": datetime.now(UTC).isoformat(),
        }

    def get_recent_decisions(self, limit: int = 10) -> list[dict[str, Any]]:
        """Get recent decisions.

        Args:
            limit: Maximum number of decisions to return

        Returns:
            List of decision records as dictionaries
        """
        # Placeholder - would query from database
        return []

    def get_robot_metrics(self, robot_id: str) -> dict[str, Any] | None:
        """Get metrics for a specific robot.

        Args:
            robot_id: Robot identifier

        Returns:
            Robot metrics or None if robot not found
        """
        # Check if robot has pending decisions
        pending = self._logger.get_pending(robot_id)

        if not pending:
            # Robot not found
            return None

        return {
            "robot_id": robot_id,
            "decision_count": len(pending),
            "pending_count": len(pending),
            "timestamp": datetime.now(UTC).isoformat(),
        }

    def subscribe_to_updates(
        self,
        callback: Callable[[dict], None] | None = None,
    ) -> Callable[[dict], None]:
        """Subscribe to real-time updates.

        Args:
            callback: Function to call when updates occur

        Returns:
            The callback function (or a default if None)
        """
        if callback is None:

            def default_callback(data: dict) -> None:
                pass

            callback = default_callback

        self._subscribers.append(callback)
        return callback

    def broadcast_update(self, data: dict[str, Any]) -> None:
        """Broadcast update to all subscribers.

        Args:
            data: Update data to broadcast
        """
        for callback in self._subscribers:
            callback(data)


class MetricsEndpoint:
    """HTTP endpoint handler for metrics requests."""

    def __init__(self, dashboard: DashboardAPI | None = None):
        """Initialize endpoint.

        Args:
            dashboard: DashboardAPI instance
        """
        self._dashboard = dashboard or DashboardAPI()

    def handle_get(self) -> dict[str, Any]:
        """Handle GET request for metrics.

        Returns:
            HTTP response dictionary
        """
        metrics = self._dashboard.get_metrics()

        return {
            "status": 200,
            "timestamp": datetime.now(UTC).isoformat(),
            "data": metrics,
        }
