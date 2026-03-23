"""TDD tests for shadow mode dashboard API - RED PHASE.

These tests define the expected behavior before implementation exists.
Run these first - they should all FAIL.
"""

import pytest

# These imports will fail until we implement the modules
from agent_ros_bridge.shadow.dashboard import DashboardAPI, MetricsEndpoint


class TestDashboardAPITDD:
    """TDD tests for DashboardAPI - write these first, watch them fail."""

    @pytest.fixture
    def dashboard(self):
        """Create dashboard API instance."""
        return DashboardAPI()

    def test_dashboard_initializes(self):
        """RED: DashboardAPI should initialize without errors."""
        dashboard = DashboardAPI()
        assert dashboard is not None

    def test_get_metrics_returns_dict(self, dashboard):
        """RED: get_metrics() should return metrics dictionary."""
        metrics = dashboard.get_metrics()
        assert isinstance(metrics, dict)
        assert "agreement_rate" in metrics
        assert "total_decisions" in metrics
        assert "pending_count" in metrics

    def test_get_metrics_agreement_rate_is_float(self, dashboard):
        """RED: agreement_rate should be a float between 0 and 1."""
        metrics = dashboard.get_metrics()
        assert isinstance(metrics["agreement_rate"], float)
        assert 0.0 <= metrics["agreement_rate"] <= 1.0

    def test_get_recent_decisions_returns_list(self, dashboard):
        """RED: get_recent_decisions() should return list of decisions."""
        decisions = dashboard.get_recent_decisions(limit=10)
        assert isinstance(decisions, list)
        assert len(decisions) <= 10

    def test_get_recent_decisions_respects_limit(self, dashboard):
        """RED: get_recent_decisions() should respect limit parameter."""
        decisions = dashboard.get_recent_decisions(limit=5)
        assert len(decisions) <= 5

    def test_get_robot_metrics_returns_dict_for_active_robot(self, dashboard):
        """RED: get_robot_metrics() should return per-robot metrics for active robots."""
        # First log a decision for the robot
        from agent_ros_bridge.shadow.models import AIProposal

        dashboard._logger.log_ai_proposal(
            "bot1", AIProposal(intent_type="NAVIGATE", confidence=0.95)
        )

        robot_metrics = dashboard.get_robot_metrics("bot1")
        assert isinstance(robot_metrics, dict)
        assert "robot_id" in robot_metrics
        assert "decision_count" in robot_metrics

    def test_get_robot_metrics_returns_none_for_unknown(self, dashboard):
        """RED: get_robot_metrics() should return None for unknown robot."""
        robot_metrics = dashboard.get_robot_metrics("nonexistent")
        assert robot_metrics is None

    def test_subscribe_to_updates_returns_callback(self, dashboard):
        """RED: subscribe_to_updates() should return callback function."""
        callback = dashboard.subscribe_to_updates()
        assert callable(callback)

    def test_broadcast_update_triggers_callbacks(self, dashboard):
        """RED: broadcast_update() should trigger subscribed callbacks."""
        received = []

        def callback(data):
            received.append(data)

        dashboard.subscribe_to_updates(callback)
        dashboard.broadcast_update({"test": "data"})

        assert len(received) == 1
        assert received[0]["test"] == "data"


class TestMetricsEndpointTDD:
    """TDD tests for MetricsEndpoint HTTP handler."""

    @pytest.fixture
    def endpoint(self):
        """Create metrics endpoint."""
        return MetricsEndpoint()

    def test_metrics_endpoint_handles_get(self, endpoint):
        """RED: MetricsEndpoint should handle GET request."""
        response = endpoint.handle_get()
        assert isinstance(response, dict)
        assert "status" in response

    def test_metrics_endpoint_returns_200_status(self, endpoint):
        """RED: MetricsEndpoint should return 200 status."""
        response = endpoint.handle_get()
        assert response.get("status") == 200

    def test_metrics_endpoint_includes_timestamp(self, endpoint):
        """RED: MetricsEndpoint should include timestamp."""
        response = endpoint.handle_get()
        assert "timestamp" in response
        assert isinstance(response["timestamp"], str)
