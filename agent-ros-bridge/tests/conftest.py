"""Test fixtures for Agent ROS Bridge."""

import asyncio
import pytest
from typing import Generator

from agent_ros_bridge import ROSBridge


@pytest.fixture
def event_loop():
    """Create an instance of the default event loop for each test case."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


@pytest.fixture
def bridge() -> ROSBridge:
    """Create a ROSBridge instance for testing."""
    return ROSBridge(ros_version=2)


@pytest.fixture
def mock_action():
    """Create a mock action handler."""
    async def handler(x: float = 0, y: float = 0):
        return {"status": "success", "position": {"x": x, "y": y}}
    return handler


@pytest.fixture
def ws_config():
    """WebSocket transport configuration for testing."""
    return {
        "host": "localhost",
        "port": 8765,
        "auth": {"enabled": False}
    }


@pytest.fixture
def grpc_config():
    """gRPC transport configuration for testing."""
    return {
        "host": "localhost",
        "port": 50051,
        "auth": {"enabled": False}
    }
