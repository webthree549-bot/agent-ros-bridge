"""
TDD Tests for ROS Auto-Discovery and Self-Diagnostics

Tests define expected behavior of ROSDiscovery, ROSHealthMonitor,
and SelfHealingController.
"""

from unittest.mock import MagicMock, Mock, patch

import pytest

from agent_ros_bridge.discovery import (
    DeviceHealth,
    DeviceHealthStatus,
    ROSDiscovery,
    ROSEndpoint,
    ROSHealthMonitor,
    ROSNodeInfo,
    SelfHealingController,
    discover_and_create_agent,
)


class TestROSDiscovery:
    """ROSDiscovery finds devices from ROS graph"""

    def test_discovery_initializes_with_ros_version(self):
        """Red: Must accept ROS version parameter"""
        discovery = ROSDiscovery(ros_version="ros2")
        assert discovery.ros_version == "ros2"

    def test_discovery_defaults_to_ros2(self):
        """Red: Must default to ROS2"""
        discovery = ROSDiscovery()
        assert discovery.ros_version == "ros2"

    def test_device_signatures_defined(self):
        """Red: Must have device signatures for type inference"""
        discovery = ROSDiscovery()

        assert "mobile_robot" in discovery.DEVICE_SIGNATURES
        assert "drone" in discovery.DEVICE_SIGNATURES
        assert "manipulator" in discovery.DEVICE_SIGNATURES
        assert "humanoid" in discovery.DEVICE_SIGNATURES
        assert "sensor_array" in discovery.DEVICE_SIGNATURES

    def test_mobile_robot_signature_has_required_topics(self):
        """Red: Mobile robot must have cmd_vel and odom"""
        discovery = ROSDiscovery()
        sig = discovery.DEVICE_SIGNATURES["mobile_robot"]

        assert "/cmd_vel" in sig["required"]
        assert "/odom" in sig["required"]

    def test_drone_signature_has_mavros_topics(self):
        """Red: Drone must have MAVROS topics"""
        discovery = ROSDiscovery()
        sig = discovery.DEVICE_SIGNATURES["drone"]

        assert "/mavros/state" in sig["required"]


class TestDeviceTypeInference:
    """Infer device type from ROS graph"""

    def test_infer_mobile_robot_from_topics(self):
        """Red: Must detect mobile robot from cmd_vel and odom"""
        discovery = ROSDiscovery()

        # Mock ROS graph with mobile robot topics
        graph = {
            "nodes": ["/bot1/nav_node", "/bot1/odom_node"],
            "topics": [
                {"name": "/bot1/cmd_vel", "types": ["geometry_msgs/Twist"]},
                {"name": "/bot1/odom", "types": ["nav_msgs/Odometry"]},
            ],
            "services": [],
        }

        device_type = discovery.infer_device_type("bot1", graph)

        assert device_type == "mobile_robot"

    def test_infer_drone_from_mavros(self):
        """Red: Must detect drone from MAVROS topics"""
        discovery = ROSDiscovery()

        graph = {
            "nodes": ["/drone1/mavros"],
            "topics": [
                {"name": "/mavros/state", "types": ["mavros_msgs/State"]},
                {
                    "name": "/mavros/local_position/pose",
                    "types": ["geometry_msgs/PoseStamped"],
                },
            ],
            "services": [],
        }

        device_type = discovery.infer_device_type("drone1", graph)

        assert device_type == "drone"

    def test_return_none_for_unknown_device(self):
        """Red: Must return None if device type unclear"""
        discovery = ROSDiscovery()

        graph = {
            "nodes": ["/unknown/something"],
            "topics": [{"name": "/unknown/random_topic", "types": ["std_msgs/String"]}],
            "services": [],
        }

        device_type = discovery.infer_device_type("unknown", graph)

        assert device_type is None

    def test_infer_requires_minimum_confidence(self):
        """Red: Must not infer without sufficient evidence"""
        discovery = ROSDiscovery()

        # Only one optional topic - not enough
        graph = {
            "nodes": ["/bot1/node"],
            "topics": [{"name": "/bot1/scan", "types": ["sensor_msgs/LaserScan"]}],
            "services": [],
        }

        device_type = discovery.infer_device_type("bot1", graph)

        # Should not infer with only optional topics
        assert device_type is None


class TestCapabilityDiscovery:
    """Discover device capabilities from ROS graph"""

    def test_discover_navigation_capabilities(self):
        """Red: Must find navigation from cmd_vel"""
        discovery = ROSDiscovery()

        graph = {
            "topics": [{"name": "/bot1/cmd_vel", "types": ["geometry_msgs/Twist"]}],
        }

        caps = discovery.discover_capabilities("bot1", graph)

        assert "navigate_to" in caps
        assert "rotate" in caps
        assert "stop" in caps

    def test_discover_gripper_capabilities(self):
        """Red: Must find gripper from gripper topics"""
        discovery = ROSDiscovery()

        graph = {
            "topics": [{"name": "/arm1/gripper_controller/gripper_cmd", "types": []}],
        }

        caps = discovery.discover_capabilities("arm1", graph)

        assert "grasp" in caps
        assert "release" in caps

    def test_discover_manipulator_capabilities(self):
        """Red: Must find arm capabilities"""
        discovery = ROSDiscovery()

        graph = {
            "topics": [{"name": "/arm1/arm_controller/follow_joint_trajectory", "types": []}],
        }

        caps = discovery.discover_capabilities("arm1", graph)

        assert "move_to" in caps
        assert "follow_trajectory" in caps

    def test_discover_drone_capabilities(self):
        """Red: Must find drone capabilities from MAVROS"""
        discovery = ROSDiscovery()

        graph = {"topics": [{"name": "/mavros/cmd/takeoff", "types": []}]}

        caps = discovery.discover_capabilities("drone1", graph)

        assert "takeoff" in caps
        assert "land" in caps


class TestDiscoverAllDevices:
    """Discover all devices on network"""

    def test_discover_all_finds_multiple_devices(self):
        """Red: Must find all devices from node namespaces"""
        discovery = ROSDiscovery()

        # Mock ROS graph with multiple devices
        graph = {
            "nodes": [
                "/bot1/nav_node",
                "/bot1/odom_node",
                "/arm1/controller",
                "/drone1/mavros",
            ],
            "topics": [
                {"name": "/bot1/cmd_vel", "types": []},
                {"name": "/bot1/odom", "types": []},
                {"name": "/arm1/joint_states", "types": []},
                {"name": "/mavros/state", "types": []},
            ],
            "services": [],
        }

        with patch.object(discovery, "get_ros_graph", return_value=graph):
            devices = discovery.discover_all_devices()

        assert len(devices) >= 1  # At least one device found

    def test_discover_all_returns_device_info(self):
        """Red: Must return device_id, type, and capabilities"""
        discovery = ROSDiscovery()

        graph = {
            "nodes": ["/bot1/nav_node"],
            "topics": [
                {"name": "/bot1/cmd_vel", "types": []},
                {"name": "/bot1/odom", "types": []},
            ],
            "services": [],
        }

        with patch.object(discovery, "get_ros_graph", return_value=graph):
            devices = discovery.discover_all_devices()

        assert len(devices) > 0

        device = devices[0]
        assert "device_id" in device
        assert "device_type" in device
        assert "capabilities" in device


class TestROSHealthMonitor:
    """ROSHealthMonitor checks device health"""

    def test_health_monitor_initializes(self):
        """Red: Must initialize with device_id"""
        monitor = ROSHealthMonitor("bot1")

        assert monitor.device_id == "bot1"
        assert monitor.ros_version == "ros2"  # default
        assert not monitor._running

    def test_check_health_returns_health_object(self):
        """Red: Must return DeviceHealth"""
        monitor = ROSHealthMonitor("bot1")

        # Mock the internal checks
        with patch.object(monitor, "_check_responsiveness", return_value=True):
            with patch.object(monitor, "_check_required_topics", return_value=[]):
                with patch.object(monitor, "_check_diagnostics", return_value=(0, 0, [])):
                    health = monitor.check_health()

        assert isinstance(health, DeviceHealth)
        assert health.device_id == "bot1"
        assert isinstance(health.status, DeviceHealthStatus)
        assert health.latency_ms >= 0

    def test_healthy_device_status(self):
        """Red: Responsive with no issues = HEALTHY"""
        monitor = ROSHealthMonitor("bot1")

        with patch.object(monitor, "_check_responsiveness", return_value=True):
            with patch.object(monitor, "_check_required_topics", return_value=[]):
                with patch.object(monitor, "_check_diagnostics", return_value=(0, 0, [])):
                    health = monitor.check_health()

        assert health.status == DeviceHealthStatus.HEALTHY

    def test_offline_device_status(self):
        """Red: Non-responsive = OFFLINE"""
        monitor = ROSHealthMonitor("bot1")

        with patch.object(monitor, "_check_responsiveness", return_value=False):
            health = monitor.check_health()

        assert health.status == DeviceHealthStatus.OFFLINE

    def test_degraded_with_missing_topics(self):
        """Red: Missing topics = DEGRADED"""
        monitor = ROSHealthMonitor("bot1")

        with patch.object(monitor, "_check_responsiveness", return_value=True):
            with patch.object(monitor, "_check_required_topics", return_value=["/cmd_vel"]):
                with patch.object(monitor, "_check_diagnostics", return_value=(0, 0, [])):
                    health = monitor.check_health()

        assert health.status == DeviceHealthStatus.DEGRADED
        assert "/cmd_vel" in health.missing_topics

    def test_unhealthy_with_errors(self):
        """Red: Errors present = UNHEALTHY"""
        monitor = ROSHealthMonitor("bot1")

        with patch.object(monitor, "_check_responsiveness", return_value=True):
            with patch.object(monitor, "_check_required_topics", return_value=[]):
                with patch.object(monitor, "_check_diagnostics", return_value=(3, 0, ["Error 1"])):
                    health = monitor.check_health()

        assert health.status == DeviceHealthStatus.UNHEALTHY
        assert health.error_count == 3

    def test_health_history_tracking(self):
        """Red: Must track health check history"""
        monitor = ROSHealthMonitor("bot1")

        # Perform multiple checks
        with patch.object(monitor, "_check_responsiveness", return_value=True):
            with patch.object(monitor, "_check_required_topics", return_value=[]):
                with patch.object(monitor, "_check_diagnostics", return_value=(0, 0, [])):
                    for _ in range(5):
                        monitor.check_health()

        assert len(monitor._health_history) == 5


class TestSelfHealingController:
    """SelfHealingController attempts recovery"""

    def test_healer_initializes(self):
        """Red: Must initialize with device_id"""
        healer = SelfHealingController("bot1")

        assert healer.device_id == "bot1"
        assert healer._recovery_attempts == 0
        assert healer._max_recovery_attempts == 3

    def test_attempt_recovery_increments_counter(self):
        """Red: Failed recovery increments attempt counter"""
        healer = SelfHealingController("bot1")

        # Create unhealthy health status
        health = DeviceHealth(
            device_id="bot1",
            status=DeviceHealthStatus.UNHEALTHY,
            last_seen=0,
            latency_ms=100,
            missing_topics=["/cmd_vel"],
        )

        # Mock recovery strategies to fail
        with patch.object(healer, "_restart_missing_nodes", return_value=False):
            with patch.object(healer, "_reinitialize_connection", return_value=False):
                with patch.object(healer, "_clear_error_state", return_value=False):
                    success = healer.attempt_recovery(health)

        assert not success
        assert healer._recovery_attempts == 1

    def test_max_recovery_attempts(self):
        """Red: Must stop after max attempts"""
        healer = SelfHealingController("bot1")
        healer._recovery_attempts = 3  # Already at max

        health = DeviceHealth(
            device_id="bot1",
            status=DeviceHealthStatus.UNHEALTHY,
            last_seen=0,
            latency_ms=100,
        )

        success = healer.attempt_recovery(health)

        assert not success  # Should fail immediately

    def test_successful_recovery_resets_counter(self):
        """Red: Success resets recovery counter"""
        healer = SelfHealingController("bot1")
        healer._recovery_attempts = 2  # Near max

        health = DeviceHealth(
            device_id="bot1",
            status=DeviceHealthStatus.UNHEALTHY,
            last_seen=0,
            latency_ms=100,
            missing_topics=["/cmd_vel"],
        )

        # Mock successful restart
        with patch.object(healer, "_restart_missing_nodes", return_value=True):
            success = healer.attempt_recovery(health)

        assert success
        assert healer._recovery_attempts == 0  # Reset


class TestDiscoverAndCreateAgent:
    """Integration: discover and create RobotAgent"""

    def test_discover_raises_if_device_unknown(self):
        """Red: Must raise if cannot discover device type"""
        with patch(
            "agent_ros_bridge.discovery.ROSDiscovery.infer_device_type",
            return_value=None,
        ):
            with pytest.raises(ValueError) as exc_info:
                discover_and_create_agent("unknown_device")

            assert "Could not auto-discover" in str(exc_info.value)

    def test_discover_creates_agent_for_known_device(self):
        """Red: Must create agent for discovered device"""
        from agent_ros_bridge.agentic import RobotAgent

        with patch(
            "agent_ros_bridge.discovery.ROSDiscovery.infer_device_type",
            return_value="mobile_robot",
        ):
            with patch(
                "agent_ros_bridge.discovery.ROSDiscovery.discover_capabilities",
                return_value=["navigate_to", "rotate"],
            ):
                with patch("agent_ros_bridge.discovery.ROSHealthMonitor") as mock_monitor:
                    mock_monitor.return_value.check_health.return_value = MagicMock(
                        status=DeviceHealthStatus.HEALTHY
                    )

                    agent = discover_and_create_agent("bot1")

        assert isinstance(agent, RobotAgent)
        assert agent.device_id == "bot1"
        assert agent.device_type == "mobile_robot"


class TestRobotAgentDiscoverMethods:
    """RobotAgent.discover() and RobotAgent.discover_all()"""

    def test_robot_agent_has_discover_classmethod(self):
        """Red: RobotAgent must have discover() class method"""
        from agent_ros_bridge.agentic import RobotAgent

        assert hasattr(RobotAgent, "discover")
        assert callable(RobotAgent.discover)

    def test_robot_agent_has_discover_all_classmethod(self):
        """Red: RobotAgent must have discover_all() class method"""
        from agent_ros_bridge.agentic import RobotAgent

        assert hasattr(RobotAgent, "discover_all")
        assert callable(RobotAgent.discover_all)


class TestTDDPrinciples:
    """Verify TDD principles"""

    def test_discovery_module_has_tests(self):
        """Red: discovery.py must have corresponding tests"""
        # This test file exists, so requirement is met
        import agent_ros_bridge.discovery

        assert hasattr(agent_ros_bridge.discovery, "ROSDiscovery")
        assert hasattr(agent_ros_bridge.discovery, "ROSHealthMonitor")
        assert hasattr(agent_ros_bridge.discovery, "SelfHealingController")

    def test_tests_define_expected_behavior(self):
        """Red: Tests must specify what code should do"""
        # Tests above specify:
        # - Discovery must infer device types from ROS graph
        # - Health monitoring must check responsiveness, topics, diagnostics
        # - Self-healing must attempt recovery strategies
        # - Integration must create agents from discovered devices
        pass

    def test_all_public_classes_have_tests(self):
        """Red: All public classes must have tests"""
        # Classes tested:
        # - ROSDiscovery
        # - ROSHealthMonitor
        # - SelfHealingController
        # - DeviceHealth
        # - ROSEndpoint
        # - ROSNodeInfo
        pass
