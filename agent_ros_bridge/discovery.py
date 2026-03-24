"""
ROS Auto-Discovery and Self-Diagnostics

Provides automatic device detection, health monitoring, and self-healing
for Agent ROS Bridge.

Usage:
    # Auto-discover device type
    agent = RobotAgent.discover('bot1')

    # Or discover all devices on network
    robots = RobotAgent.discover_all()

    # With health monitoring
    agent = RobotAgent.discover('bot1', enable_health_monitor=True)
"""

import asyncio
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class DeviceHealthStatus(Enum):
    """Health status of a ROS device"""

    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNHEALTHY = "unhealthy"
    OFFLINE = "offline"
    UNKNOWN = "unknown"


@dataclass
class ROSEndpoint:
    """Represents a ROS topic, service, or action"""

    name: str
    endpoint_type: str  # 'topic', 'service', 'action'
    msg_type: str | None = None
    publishers: list[str] = field(default_factory=list)
    subscribers: list[str] = field(default_factory=list)


@dataclass
class ROSNodeInfo:
    """Information about a ROS node"""

    name: str
    namespace: str
    uri: str
    pid: int | None = None
    endpoints: list[ROSEndpoint] = field(default_factory=list)


@dataclass
class DeviceHealth:
    """Health status of a device"""

    device_id: str
    status: DeviceHealthStatus
    last_seen: float
    latency_ms: float
    missing_topics: list[str] = field(default_factory=list)
    error_count: int = 0
    warning_count: int = 0
    diagnostic_messages: list[str] = field(default_factory=list)


class ROSDiscovery:
    """
    Discovers ROS devices and their capabilities by introspecting the ROS graph.
    """

    # Topic patterns that indicate device types
    DEVICE_SIGNATURES = {
        "mobile_robot": {
            "required": ["/cmd_vel", "/odom"],
            "optional": ["/scan", "/amcl_pose", "/move_base/status"],
            "indicators": {
                "/cmd_vel": "Twist",
                "/odom": "Odometry",
            },
        },
        "drone": {
            "required": ["/mavros/state", "/mavros/local_position/pose"],
            "optional": ["/mavros/battery", "/mavros/cmd/takeoff"],
            "indicators": {
                "/mavros/state": "State",
                "/mavros/local_position/pose": "PoseStamped",
            },
        },
        "manipulator": {
            "required": ["/joint_states", "/arm_controller/follow_joint_trajectory"],
            "optional": ["/gripper_controller/gripper_cmd", "/robot_description"],
            "indicators": {
                "/joint_states": "JointState",
            },
        },
        "humanoid": {
            "required": ["/joint_states"],
            "optional": ["/walking_controller/status", "/balance_controller/status"],
            "indicators": {
                "/joint_states": "JointState",
                "/walking_controller/status": "String",
            },
        },
        "sensor_array": {
            "required": ["/camera/image_raw"],
            "optional": ["/scan", "/imu/data", "/thermal/image"],
            "indicators": {
                "/camera/image_raw": "Image",
            },
        },
    }

    def __init__(self, ros_version: str = "ros2"):
        """
        Initialize ROS discovery.

        Args:
            ros_version: 'ros1' or 'ros2'
        """
        self.ros_version = ros_version
        self._discovered_devices: dict[str, dict[str, Any]] = {}

    def get_ros_graph(self) -> dict[str, Any]:
        """
        Get current ROS graph state.

        Returns:
            Dictionary with nodes, topics, services
        """
        if self.ros_version == "ros2":
            return self._get_ros2_graph()
        else:
            return self._get_ros1_graph()

    def _get_ros2_graph(self) -> dict[str, Any]:
        """Get ROS2 graph using rclpy"""
        try:
            import rclpy
            from rclpy.node import Node

            # Initialize if needed
            if not rclpy.ok():
                rclpy.init()

            # Create temporary node for discovery
            node = Node("discovery_node")

            # Get node names
            node_names = node.get_node_names()

            # Get topic names and types
            topic_names_and_types = node.get_topic_names_and_types()

            # Get service names and types
            service_names_and_types = node.get_service_names_and_types()

            node.destroy_node()

            return {
                "nodes": node_names,
                "topics": [{"name": name, "types": types} for name, types in topic_names_and_types],
                "services": [
                    {"name": name, "types": types} for name, types in service_names_and_types
                ],
            }

        except ImportError:
            # Fallback: return empty graph
            return {"nodes": [], "topics": [], "services": []}

    def _get_ros1_graph(self) -> dict[str, Any]:
        """Get ROS1 graph using rosgraph"""
        try:
            import rosgraph

            master = rosgraph.Master("/discovery_node")

            # Get system state
            state = master.getSystemState()

            # Get topic types
            topic_types = master.getTopicTypes()

            return {
                "nodes": list(
                    set(
                        [n for _, nodes in state[0] for n in nodes]
                        + [n for _, nodes in state[1] for n in nodes]
                    )
                ),
                "topics": [{"name": name, "types": [msg_type]} for name, msg_type in topic_types],
                "services": [{"name": name} for name, _ in master.getSystemState()[2]],
            }

        except ImportError:
            return {"nodes": [], "topics": [], "services": []}

    def infer_device_type(self, device_id: str, graph: dict[str, Any] = None) -> str | None:
        """
        Infer device type from ROS graph.

        Args:
            device_id: Device identifier (node namespace prefix)
            graph: ROS graph (optional, will query if not provided)

        Returns:
            Device type string or None if unknown
        """
        if graph is None:
            graph = self.get_ros_graph()

        # Get topics/services for this device
        device_topics = [
            t
            for t in graph.get("topics", [])
            if device_id in t["name"] or t["name"].startswith(f"/{device_id}")
        ]

        topic_names = {t["name"] for t in device_topics}

        # Score each device type
        scores = {}
        for device_type, signature in self.DEVICE_SIGNATURES.items():
            score = 0

            # Check required topics
            for req in signature["required"]:
                if any(req in t for t in topic_names):
                    score += 10

            # Check optional topics
            for opt in signature["optional"]:
                if any(opt in t for t in topic_names):
                    score += 3

            # Check topic type indicators
            for topic_name, msg_type in signature["indicators"].items():
                for t in device_topics:
                    if topic_name in t["name"] and msg_type in t.get("types", []):
                        score += 5

            scores[device_type] = score

        # Return highest scoring type (if above threshold)
        if scores:
            best_type = max(scores, key=scores.get)
            if scores[best_type] >= 10:  # Minimum confidence threshold
                return best_type

        return None

    def discover_capabilities(self, device_id: str, graph: dict[str, Any] = None) -> list[str]:
        """
        Discover device capabilities from ROS graph.

        Args:
            device_id: Device identifier
            graph: ROS graph (optional)

        Returns:
            List of capability names
        """
        if graph is None:
            graph = self.get_ros_graph()

        capabilities = []
        topic_names = {t["name"] for t in graph.get("topics", [])}

        # Infer capabilities from topics
        if any("cmd_vel" in t for t in topic_names):
            capabilities.extend(["navigate_to", "rotate", "stop"])

        if any("gripper" in t for t in topic_names):
            capabilities.extend(["grasp", "release"])

        if any("arm_controller" in t for t in topic_names):
            capabilities.extend(["move_to", "follow_trajectory"])

        if any("mavros" in t for t in topic_names):
            capabilities.extend(["takeoff", "land", "fly_to", "hover"])

        if any("walking" in t for t in topic_names):
            capabilities.extend(["walk", "balance", "climb"])

        return list(set(capabilities))

    def discover_all_devices(self) -> list[dict[str, Any]]:
        """
        Discover all devices on the ROS network.

        Returns:
            List of device info dictionaries
        """
        graph = self.get_ros_graph()
        devices = []

        # Find potential device namespaces from node names
        namespaces = set()
        for node in graph.get("nodes", []):
            # Extract namespace (e.g., /bot1/nav_node → bot1)
            parts = node.strip("/").split("/")
            if parts:
                namespaces.add(parts[0])

        # Try to identify device type for each namespace
        for ns in namespaces:
            device_type = self.infer_device_type(ns, graph)
            if device_type:
                capabilities = self.discover_capabilities(ns, graph)
                devices.append(
                    {
                        "device_id": ns,
                        "device_type": device_type,
                        "capabilities": capabilities,
                        "confidence": "high" if device_type else "low",
                    }
                )

        return devices


class ROSHealthMonitor:
    """
    Monitors ROS device health and provides self-diagnostics.
    """

    def __init__(self, device_id: str, ros_version: str = "ros2"):
        """
        Initialize health monitor.

        Args:
            device_id: Device to monitor
            ros_version: 'ros1' or 'ros2'
        """
        self.device_id = device_id
        self.ros_version = ros_version
        self._running = False
        self._health_history: list[DeviceHealth] = []
        self._max_history = 100

    def check_health(self) -> DeviceHealth:
        """
        Perform health check on device.

        Returns:
            DeviceHealth with current status
        """
        start_time = time.time()

        # Check if device is responsive
        is_responsive = self._check_responsiveness()

        # Check required topics
        missing_topics = self._check_required_topics()

        # Check for errors/warnings
        errors, warnings, messages = self._check_diagnostics()

        # Calculate latency
        latency_ms = (time.time() - start_time) * 1000

        # Determine status
        if not is_responsive:
            status = DeviceHealthStatus.OFFLINE
        elif errors > 0:
            status = DeviceHealthStatus.UNHEALTHY
        elif missing_topics or warnings > 0:
            status = DeviceHealthStatus.DEGRADED
        else:
            status = DeviceHealthStatus.HEALTHY

        health = DeviceHealth(
            device_id=self.device_id,
            status=status,
            last_seen=time.time(),
            latency_ms=latency_ms,
            missing_topics=missing_topics,
            error_count=errors,
            warning_count=warnings,
            diagnostic_messages=messages,
        )

        # Add to history
        self._health_history.append(health)
        if len(self._health_history) > self._max_history:
            self._health_history.pop(0)

        return health

    def _check_responsiveness(self) -> bool:
        """Check if device is responsive"""
        # Try to ping device via ROS
        try:
            if self.ros_version == "ros2":
                # ROS2: Check if node is in node list
                import rclpy
                from rclpy.node import Node

                if not rclpy.ok():
                    return False

                node = Node("health_check")
                nodes = node.get_node_names()
                node.destroy_node()

                return any(self.device_id in n for n in nodes)
            else:
                # ROS1: Check roscore
                import rosgraph

                try:
                    master = rosgraph.Master("/health_check")
                    master.getPid()
                    return True
                except Exception:
                    return False
        except Exception:
            return False

    def _check_required_topics(self) -> list[str]:
        """Check which required topics are missing"""
        discovery = ROSDiscovery(self.ros_version)
        graph = discovery.get_ros_graph()

        # Get expected topics for this device type
        device_type = discovery.infer_device_type(self.device_id, graph)

        if not device_type:
            return []

        signature = ROSDiscovery.DEVICE_SIGNATURES.get(device_type, {})
        required = signature.get("required", [])

        # Check which are missing
        topic_names = {t["name"] for t in graph.get("topics", [])}
        missing = []

        for req in required:
            if not any(req in t for t in topic_names):
                missing.append(req)

        return missing

    def _check_diagnostics(self) -> tuple[int, int, list[str]]:
        """Check diagnostic messages"""
        errors = 0
        warnings = 0
        messages = []

        try:
            # Try to read diagnostics topic
            if self.ros_version == "ros2":
                # ROS2: /diagnostics topic
                pass  # Would subscribe and check
            else:
                # ROS1: /diagnostics
                pass
        except Exception:
            pass

        return errors, warnings, messages

    async def start_monitoring(self, interval_sec: float = 5.0):
        """
        Start continuous health monitoring.

        Args:
            interval_sec: Check interval in seconds
        """
        self._running = True

        while self._running:
            health = self.check_health()

            # Log issues
            if health.status != DeviceHealthStatus.HEALTHY:
                print(f"⚠️  {self.device_id} health: {health.status.value}")
                for msg in health.diagnostic_messages:
                    print(f"   - {msg}")

            await asyncio.sleep(interval_sec)

    def stop_monitoring(self):
        """Stop health monitoring"""
        self._running = False

    def get_health_trend(self) -> dict[str, Any]:
        """
        Get health trend over time.

        Returns:
            Dictionary with trend analysis
        """
        if not self._health_history:
            return {"trend": "unknown"}

        # Count status occurrences
        status_counts = {}
        for h in self._health_history:
            status_counts[h.status.value] = status_counts.get(h.status.value, 0) + 1

        # Check trend
        recent = self._health_history[-10:]
        recent_statuses = [h.status for h in recent]

        if all(s == DeviceHealthStatus.HEALTHY for s in recent_statuses):
            trend = "stable_healthy"
        elif any(s == DeviceHealthStatus.UNHEALTHY for s in recent_statuses):
            trend = "degrading"
        else:
            trend = "fluctuating"

        return {
            "trend": trend,
            "status_distribution": status_counts,
            "average_latency_ms": sum(h.latency_ms for h in self._health_history)
            / len(self._health_history),
            "error_rate": sum(h.error_count for h in self._health_history)
            / len(self._health_history),
        }


class SelfHealingController:
    """
    Provides self-healing capabilities for ROS systems.
    """

    def __init__(self, device_id: str, ros_version: str = "ros2"):
        self.device_id = device_id
        self.ros_version = ros_version
        self._recovery_attempts = 0
        self._max_recovery_attempts = 3

    def attempt_recovery(self, health: DeviceHealth) -> bool:
        """
        Attempt to recover from unhealthy state.

        Args:
            health: Current health status

        Returns:
            True if recovery successful
        """
        if self._recovery_attempts >= self._max_recovery_attempts:
            print(f"❌ Max recovery attempts reached for {self.device_id}")
            return False

        print(f"🔧 Attempting recovery for {self.device_id}...")

        success = False

        # Strategy 1: Restart missing nodes
        if health.missing_topics:
            success = self._restart_missing_nodes(health.missing_topics)

        # Strategy 2: Reinitialize connection
        if not success and health.status == DeviceHealthStatus.OFFLINE:
            success = self._reinitialize_connection()

        # Strategy 3: Clear error state
        if not success and health.error_count > 0:
            success = self._clear_error_state()

        if success:
            print(f"✅ Recovery successful for {self.device_id}")
            self._recovery_attempts = 0
        else:
            print(f"⚠️  Recovery failed for {self.device_id}")
            self._recovery_attempts += 1

        return success

    def _restart_missing_nodes(self, missing_topics: list[str]) -> bool:
        """Restart nodes providing missing topics"""
        # This would use roslaunch or ros2 launch
        print(f"   Restarting nodes for topics: {missing_topics}")
        return False  # Placeholder

    def _reinitialize_connection(self) -> bool:
        """Reinitialize ROS connection"""
        print("   Reinitializing ROS connection...")
        return False  # Placeholder

    def _clear_error_state(self) -> bool:
        """Clear error state"""
        print("   Clearing error state...")
        return False  # Placeholder


# Integration with RobotAgent


def discover_and_create_agent(
    device_id: str, enable_health_monitor: bool = True, enable_self_healing: bool = True
) -> Any:
    """
    Auto-discover device and create RobotAgent.

    Args:
        device_id: Device identifier
        enable_health_monitor: Enable health monitoring
        enable_self_healing: Enable self-healing

    Returns:
        Configured RobotAgent
    """
    from agent_ros_bridge.agentic import RobotAgent

    # Discover device type
    discovery = ROSDiscovery()
    device_type = discovery.infer_device_type(device_id)

    if not device_type:
        raise ValueError(
            f"Could not auto-discover device type for '{device_id}'. "
            f"Please specify device_type explicitly."
        )

    print(f"🔍 Discovered {device_id} as {device_type}")

    # Discover capabilities
    capabilities = discovery.discover_capabilities(device_id)
    print(f"   Capabilities: {capabilities}")

    # Create agent
    agent = RobotAgent(
        device_id=device_id,
        device_type=device_type,
    )

    # Optionally enable health monitoring
    if enable_health_monitor:
        health_monitor = ROSHealthMonitor(device_id)
        health = health_monitor.check_health()
        print(f"   Health: {health.status.value}")

        # Attempt recovery if unhealthy
        if enable_self_healing and health.status != DeviceHealthStatus.HEALTHY:
            healer = SelfHealingController(device_id)
            healer.attempt_recovery(health)

    return agent


# Example usage
if __name__ == "__main__":
    print("=" * 70)
    print("🔍 ROS Auto-Discovery and Self-Diagnostics Demo")
    print("=" * 70)
    print()

    # Demo 1: Discover all devices
    print("1. Discovering all ROS devices...")
    discovery = ROSDiscovery()

    try:
        graph = discovery.get_ros_graph()
        print(f"   Found {len(graph.get('nodes', []))} nodes")
        print(f"   Found {len(graph.get('topics', []))} topics")
        print()

        devices = discovery.discover_all_devices()
        print(f"   Identified {len(devices)} devices:")
        for dev in devices:
            print(f"     - {dev['device_id']}: {dev['device_type']}")
            print(f"       Capabilities: {dev['capabilities']}")
    except Exception as e:
        print(f"   Note: ROS not running ({e})")

    print()
    print("2. Health monitoring example...")
    try:
        monitor = ROSHealthMonitor("bot1")
        health = monitor.check_health()
        print(f"   Status: {health.status.value}")
        print(f"   Latency: {health.latency_ms:.1f}ms")
        if health.missing_topics:
            print(f"   Missing topics: {health.missing_topics}")
    except Exception as e:
        print(f"   Note: Health check requires ROS ({e})")

    print()
    print("=" * 70)
    print("Usage:")
    print("  agent = RobotAgent.discover('bot1')")
    print("  # or")
    print("  robots = RobotAgent.discover_all()")
    print("=" * 70)
