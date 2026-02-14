"""pytest configuration for integration tests"""

import pytest
import asyncio


@pytest.fixture(scope="session")
def event_loop():
    """Create an instance of the default event loop for the test session"""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


@pytest.fixture(scope="session")
def bridge_url():
    """Get bridge WebSocket URL from environment"""
    import os
    return os.environ.get("BRIDGE_URL", "ws://localhost:8766")


@pytest.fixture(scope="session")  
def mqtt_broker():
    """Get MQTT broker from environment"""
    import os
    return os.environ.get("MQTT_BROKER", "localhost")


# Custom markers
def pytest_configure(config):
    """Configure custom pytest markers"""
    config.addinivalue_line("markers", "slow: marks tests as slow (deselect with '-m \"not slow\"')")
    config.addinivalue_line("markers", "docker: marks tests as requiring Docker")
    config.addinivalue_line("markers", "mqtt: marks tests as requiring MQTT broker")
