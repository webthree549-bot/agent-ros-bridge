"""Optimized pytest configuration for Agent ROS Bridge.

This conftest provides:
- Fast test fixtures using simulation
- Automatic test isolation
- Parallel test support
- Performance optimizations
- Async test support
"""

import asyncio
import os
import sys
import time
import warnings
from unittest import mock

import pytest

# Configure pytest-asyncio
try:
    import pytest_asyncio
    pytest_plugins = ('pytest_asyncio',)
except ImportError:
    pass

# Suppress deprecation warnings for asyncio.get_event_loop_policy (Python 3.14+)
warnings.filterwarnings("ignore", category=DeprecationWarning, message=".*get_event_loop_policy.*")

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Set test environment variables before any imports
os.environ["JWT_SECRET"] = "test-secret-key-for-ci-only"
os.environ["MOCK_MODE"] = "true"
os.environ["SKIP_INTEGRATION_TESTS"] = "1"  # Skip integration tests by default in unit tests

# Import simulation framework
from tests.simulation import SimulatedRobot, SimulatedROS2Node, SimulatedROSEnvironment

# =============================================================================
# Pytest Configuration Hooks
# =============================================================================


def pytest_configure(config):
    """Configure pytest with custom markers."""
    config.addinivalue_line("markers", "fast: Fast unit tests (run by default)")
    config.addinivalue_line("markers", "slow: Slow tests (skip with --fast)")
    config.addinivalue_line("markers", "ros: Tests requiring ROS2")
    config.addinivalue_line("markers", "simulation: Tests using simulation framework")


def pytest_addoption(parser):
    """Add custom command line options."""
    parser.addoption(
        "--fast",
        action="store_true",
        default=False,
        help="Run only fast tests (skip slow ones)",
    )
    parser.addoption(
        "--with-ros",
        action="store_true",
        default=False,
        help="Run tests that require ROS2",
    )
    parser.addoption(
        "--with-integration",
        action="store_true",
        default=False,
        help="Run integration tests",
    )


def pytest_collection_modifyitems(config, items):
    """Modify test collection based on options."""
    # Skip slow tests in fast mode
    if config.getoption("--fast"):
        skip_slow = pytest.mark.skip(reason="--fast mode: skipping slow tests")
        for item in items:
            if "slow" in item.keywords:
                item.add_marker(skip_slow)

    # Skip ROS tests unless --with-ros
    if not config.getoption("--with-ros"):
        skip_ros = pytest.mark.skip(reason="--with-ros not specified")
        for item in items:
            if "ros" in item.keywords:
                item.add_marker(skip_ros)

    # Skip integration tests unless --with-integration
    if not config.getoption("--with-integration"):
        skip_integration = pytest.mark.skip(reason="--with-integration not specified")
        for item in items:
            if "integration" in item.keywords:
                item.add_marker(skip_integration)


@pytest.fixture(scope="session", autouse=True)
def event_loop_policy():
    """Set event loop policy for the test session."""
    # Use the default asyncio policy but ensure proper cleanup
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        original_policy = asyncio.get_event_loop_policy()
    yield original_policy
    # Cleanup any remaining tasks
    try:
        loop = asyncio.get_event_loop()
        if loop.is_running():
            pending = asyncio.all_tasks(loop)
            for task in pending:
                task.cancel()
    except RuntimeError:
        pass


# =============================================================================
# Fast Test Fixtures
# =============================================================================


@pytest.fixture
def event_loop():
    """Create a fresh event loop for each test."""
    loop = asyncio.new_event_loop()
    yield loop
    # Clean up
    try:
        pending = asyncio.all_tasks(loop)
        for task in pending:
            task.cancel()
        if pending:
            loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()
    except Exception:
        pass


@pytest.fixture(scope="function")
async def bridge():
    """Create a fresh bridge instance for each test.

    This is faster than creating a new bridge for every test,
    but still provides isolation.
    """
    from agent_ros_bridge import Bridge

    bridge = Bridge()
    yield bridge
    # Cleanup
    if bridge.running:
        await bridge.stop()


@pytest.fixture(scope="function")
async def bridge_with_transport():
    """Create a bridge with WebSocket transport."""
    from agent_ros_bridge import Bridge
    from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

    bridge = Bridge()
    transport = WebSocketTransport({"port": 0})  # Random port
    bridge.transport_manager.register(transport)

    yield bridge

    if bridge.running:
        await bridge.stop()


# =============================================================================
# Simulation Fixtures
# =============================================================================


@pytest.fixture(scope="function")
async def sim_node() -> SimulatedROS2Node:
    """Create a simulated ROS2 node."""
    node = SimulatedROS2Node("test_node")
    yield node
    # No cleanup needed for simulation


@pytest.fixture(scope="function")
async def sim_robot() -> SimulatedRobot:
    """Create a simulated robot."""
    robot = SimulatedRobot("test_robot", "differential_drive")
    await robot.start()
    yield robot
    await robot.stop()


@pytest.fixture(scope="function")
async def sim_env() -> SimulatedROSEnvironment:
    """Create a simulated ROS environment."""
    env = SimulatedROSEnvironment()
    yield env
    await env.cleanup()


@pytest.fixture(scope="function")
async def sim_turtlebot():
    """Create a simulated TurtleBot-style robot."""
    robot = SimulatedRobot("turtle1", "differential_drive")
    await robot.start()
    yield robot
    await robot.stop()


# =============================================================================
# Mock Fixtures
# =============================================================================


@pytest.fixture
def mock_ros2_modules():
    """Mock ROS2 modules for testing without ROS installation."""
    mock_modules = {
        "rclpy": mock.MagicMock(),
        "rclpy.node": mock.MagicMock(),
        "rclpy.qos": mock.MagicMock(),
        "std_msgs": mock.MagicMock(),
        "std_msgs.msg": mock.MagicMock(),
        "geometry_msgs": mock.MagicMock(),
        "geometry_msgs.msg": mock.MagicMock(),
        "sensor_msgs": mock.MagicMock(),
        "sensor_msgs.msg": mock.MagicMock(),
        "nav_msgs": mock.MagicMock(),
        "nav_msgs.msg": mock.MagicMock(),
    }

    with mock.patch.dict("sys.modules", mock_modules):
        yield mock_modules


@pytest.fixture
def mock_ros2_connector(mock_ros2_modules):
    """Create a mock ROS2 connector."""
    from agent_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector

    connector = ROS2Connector(domain_id=0)
    connector.node = SimulatedROS2Node("mock_ros2_node")
    return connector


@pytest.fixture
def mock_ros2_robot():
    """Create a mock ROS2 robot with pre-configured topics."""
    from agent_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Robot

    node = SimulatedROS2Node("mock_robot_node")

    # Add standard topics
    node.add_topic("/cmd_vel", "geometry_msgs/Twist")
    node.add_topic("/odom", "nav_msgs/Odometry")
    node.add_topic("/scan", "sensor_msgs/LaserScan")

    robot = ROS2Robot("test_0_", "TestRobot", node)
    robot.connected = True

    return robot


# =============================================================================
# Auth Fixtures
# =============================================================================


@pytest.fixture
def auth_manager():
    """Create an auth manager for testing."""
    from agent_ros_bridge.gateway_v2.auth import JWTAuthManager

    return JWTAuthManager(secret_key="test-secret")


@pytest.fixture
def test_token(auth_manager):
    """Generate a test JWT token."""
    return auth_manager.generate_token("test_user", roles=["operator"])


# =============================================================================
# Performance Monitoring
# =============================================================================


@pytest.fixture(autouse=True)
def test_timer(request):
    """Time each test and report slow ones."""
    start = time.time()
    yield
    elapsed = time.time() - start

    # Report slow tests (> 1 second)
    if elapsed > 1.0:
        print(f"\n⚠️  SLOW TEST: {request.node.name} took {elapsed:.2f}s")


# =============================================================================
# Test Data Fixtures
# =============================================================================


@pytest.fixture
def sample_command():
    """Return a sample command for testing."""
    from agent_ros_bridge.gateway_v2.core import Command

    return Command(
        action="move",
        parameters={"direction": "forward", "distance": 1.0},
        timeout_ms=5000,
        priority=5,
    )


@pytest.fixture
def sample_message():
    """Return a sample message for testing."""
    from agent_ros_bridge.gateway_v2.core import Command, Header, Identity, Message

    return Message(
        header=Header(),
        command=Command(action="test", parameters={}),
        identity=Identity(id="test", name="Test", roles=["operator"]),
    )


@pytest.fixture
def sample_telemetry():
    """Return sample telemetry for testing."""
    from agent_ros_bridge.gateway_v2.core import Telemetry

    return Telemetry(
        topic="/odom",
        data={"position": {"x": 1.0, "y": 2.0}, "orientation": {"z": 0.0}},
        quality=1.0,
    )


# =============================================================================
# Integration Test Fixtures (only used with --with-integration)
# =============================================================================


@pytest.fixture(scope="session")
def docker_compose_file(pytestconfig):
    """Return the path to the docker-compose file for integration tests."""
    return os.path.join(str(pytestconfig.rootdir), "examples", "quickstart", "docker-compose.yml")


# =============================================================================
# Utility Functions for Tests
# =============================================================================


async def wait_for_condition(condition_fn, timeout: float = 1.0, interval: float = 0.01):
    """Wait for a condition to become true.

    Args:
        condition_fn: Function that returns True when condition is met
        timeout: Maximum time to wait in seconds
        interval: Polling interval in seconds

    Raises:
        TimeoutError: If condition is not met within timeout
    """
    start = time.time()
    while time.time() - start < timeout:
        if condition_fn():
            return
        await asyncio.sleep(interval)
    raise TimeoutError(f"Condition not met within {timeout}s")


class AsyncContextManagerMock:
    """Helper for mocking async context managers."""

    def __init__(self, return_value=None):
        self.return_value = return_value

    async def __aenter__(self):
        return self.return_value

    async def __aexit__(self, *args):
        pass
