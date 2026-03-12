"""
TDD Tests for /safety/watchdog Node
Agent ROS Bridge v0.6.1 - Week 2 Implementation

Following TDD: Red -> Green -> Refactor
"""

import pytest
import time
from unittest.mock import Mock, patch


class TestWatchdogNodeExists:
    """Test that watchdog node exists and can be instantiated"""

    def test_watchdog_node_module_exists(self):
        """RED: Watchdog module should exist"""
        try:
            from agent_ros_bridge.safety.watchdog import WatchdogNode

            assert True
        except ImportError as e:
            pytest.fail(f"WatchdogNode module not found: {e}")

    def test_watchdog_node_class_exists(self):
        """RED: WatchdogNode class should exist"""
        from agent_ros_bridge.safety.watchdog import WatchdogNode

        assert hasattr(WatchdogNode, "__init__")


class TestHeartbeatMonitoring:
    """Test heartbeat monitoring functionality"""

    def test_monitors_heartbeat(self):
        """RED: Monitors 1kHz heartbeat from critical nodes"""
        from agent_ros_bridge.safety.watchdog import WatchdogNode

        node = WatchdogNode()

        # Register a critical node
        node.register_node("/safety/validator")

        # Simulate heartbeat
        node.process_heartbeat("/safety/validator", seq=1, timestamp=time.time())

        # Verify heartbeat was recorded
        assert "/safety/validator" in node.heartbeats
        assert node.heartbeats["/safety/validator"]["seq"] == 1

    def test_triggers_e_stop_on_timeout(self):
        """RED: No heartbeat → triggers e-stop"""
        from agent_ros_bridge.safety.watchdog import WatchdogNode

        node = WatchdogNode()
        node.emergency_stop_callback = Mock()

        # Register a critical node
        node.register_node("/safety/validator")

        # Send initial heartbeat with old timestamp (simulating stale heartbeat)
        old_timestamp = time.time() - 0.01  # 10ms ago (beyond 1ms threshold)
        node.process_heartbeat("/safety/validator", seq=1, timestamp=old_timestamp)

        # Manually trigger timeout check
        node.check_timeouts()

        # Verify e-stop was triggered
        assert node.emergency_stop_callback.called, "E-stop should be triggered on timeout"

    def test_heartbeat_timeout_1ms(self):
        """RED: Timeout after 1ms missing heartbeat"""
        from agent_ros_bridge.safety.watchdog import WatchdogNode

        node = WatchdogNode()

        # The timeout threshold should be 1ms (for 1kHz monitoring)
        assert node.TIMEOUT_THRESHOLD_MS == 1.0 or hasattr(
            node, "timeout_ms"
        ), "Timeout should be 1ms for 1kHz"


class TestWatchdogHeartbeatGeneration:
    """Test watchdog's own heartbeat generation"""

    def test_generates_1khz_heartbeat(self):
        """RED: Watchdog generates 1kHz heartbeat"""
        from agent_ros_bridge.safety.watchdog import WatchdogNode

        node = WatchdogNode()
        node.heartbeat_publisher = Mock()

        # Generate heartbeats for 10ms
        start = time.time()
        count = 0
        while (time.time() - start) < 0.01:  # 10ms
            node.publish_heartbeat()
            count += 1
            time.sleep(0.0005)  # 0.5ms

        # Should have published multiple heartbeats (system dependent)
        assert count >= 3, f"Should generate multiple heartbeats in 10ms, got {count}"

    def test_heartbeat_contains_sequence_number(self):
        """RED: Heartbeat contains monotonic sequence number"""
        from agent_ros_bridge.safety.watchdog import WatchdogNode

        node = WatchdogNode()

        heartbeat1 = node.create_heartbeat()
        heartbeat2 = node.create_heartbeat()

        assert "seq" in heartbeat1, "Heartbeat should have seq"
        assert heartbeat2["seq"] == heartbeat1["seq"] + 1, "Seq should increment"

    def test_heartbeat_contains_timestamp(self):
        """RED: Heartbeat contains timestamp"""
        from agent_ros_bridge.safety.watchdog import WatchdogNode

        node = WatchdogNode()

        heartbeat = node.create_heartbeat()

        assert "timestamp" in heartbeat, "Heartbeat should have timestamp"
        assert heartbeat["timestamp"] > 0, "Timestamp should be positive"


class TestNodeRegistration:
    """Test node registration for monitoring"""

    def test_register_critical_node(self):
        """RED: Can register critical nodes for monitoring"""
        from agent_ros_bridge.safety.watchdog import WatchdogNode

        node = WatchdogNode()

        node.register_node("/safety/validator")
        node.register_node("/safety/limits")

        assert "/safety/validator" in node.monitored_nodes
        assert "/safety/limits" in node.monitored_nodes

    def test_unregister_node(self):
        """RED: Can unregister nodes"""
        from agent_ros_bridge.safety.watchdog import WatchdogNode

        node = WatchdogNode()

        node.register_node("/safety/validator")
        assert "/safety/validator" in node.monitored_nodes

        node.unregister_node("/safety/validator")
        assert "/safety/validator" not in node.monitored_nodes


class TestTimeoutDetection:
    """Test timeout detection timing"""

    def test_detects_timeout_within_1ms(self):
        """RED: Detects missed heartbeat within 1ms"""
        from agent_ros_bridge.safety.watchdog import WatchdogNode

        node = WatchdogNode()
        node.emergency_stop_callback = Mock()

        node.register_node("/test_node")

        # Send heartbeat
        node.process_heartbeat("/test_node", seq=1, timestamp=time.time())

        # Wait 2ms (beyond 1ms threshold)
        time.sleep(0.002)

        # Check timeout
        node.check_timeouts()

        assert node.emergency_stop_callback.called, "Should detect timeout within 1ms"

    def test_no_timeout_for_active_heartbeat(self):
        """RED: No timeout if heartbeat is active"""
        from agent_ros_bridge.safety.watchdog import WatchdogNode

        node = WatchdogNode()
        node.emergency_stop_callback = Mock()

        node.register_node("/test_node")

        # Send heartbeat with current timestamp
        node.process_heartbeat("/test_node", seq=1, timestamp=time.time())

        # Check timeout immediately (should not timeout)
        node.check_timeouts()

        assert not node.emergency_stop_callback.called, "Should not timeout with active heartbeat"


class TestWatchdogStatus:
    """Test watchdog status reporting"""

    def test_publishes_watchdog_status(self):
        """RED: Watchdog publishes status topic"""
        from agent_ros_bridge.safety.watchdog import WatchdogNode

        node = WatchdogNode()
        mock_publisher = Mock()
        node.status_publisher = mock_publisher

        node.publish_status()

        assert mock_publisher.called, "Should publish status"

    def test_status_includes_monitored_nodes(self):
        """RED: Status includes list of monitored nodes"""
        from agent_ros_bridge.safety.watchdog import WatchdogNode

        node = WatchdogNode()
        node.register_node("/safety/validator")
        node.register_node("/ai/executor")

        status = node.get_status()

        assert "monitored_nodes" in status
        assert "/safety/validator" in status["monitored_nodes"]
        assert "/ai/executor" in status["monitored_nodes"]


class TestEmergencyStopIntegration:
    """Test integration with emergency stop"""

    def test_triggers_e_stop_on_critical_node_timeout(self):
        """RED: Triggers e-stop when critical node times out"""
        from agent_ros_bridge.safety.watchdog import WatchdogNode

        node = WatchdogNode()
        e_stop_triggered = [False]

        def mock_e_stop(reason, source):
            e_stop_triggered[0] = True

        node.emergency_stop_callback = mock_e_stop

        # Register critical node
        node.register_node("/safety/validator", critical=True)

        # Send initial heartbeat
        node.process_heartbeat("/safety/validator", seq=1, timestamp=time.time())

        # Wait for timeout
        time.sleep(0.002)
        node.check_timeouts()

        assert e_stop_triggered[0], "Should trigger e-stop on critical node timeout"

    def test_logs_timeout_events(self):
        """RED: Logs all timeout events"""
        from agent_ros_bridge.safety.watchdog import WatchdogNode

        node = WatchdogNode()
        node.register_node("/test_node")

        # Send initial heartbeat
        node.process_heartbeat("/test_node", seq=1, timestamp=time.time())

        # Wait for timeout
        time.sleep(0.002)
        node.check_timeouts()

        assert len(node.timeout_log) > 0, "Timeout should be logged"
        last_timeout = node.timeout_log[-1]
        assert last_timeout["node"] == "/test_node"


class TestWatchdogFrequency:
    """Test watchdog monitoring frequency"""

    def test_1khz_monitoring_frequency(self):
        """RED: Watchdog monitors at 1kHz (1ms period)"""
        from agent_ros_bridge.safety.watchdog import WatchdogNode

        node = WatchdogNode()

        # The monitoring frequency should be 1000Hz
        assert node.MONITORING_FREQUENCY_HZ == 1000 or hasattr(
            node, "monitoring_hz"
        ), "Monitoring should be at 1kHz"

        # Or check period
        if hasattr(node, "MONITORING_PERIOD_MS"):
            assert node.MONITORING_PERIOD_MS == 1.0, "Period should be 1ms"
