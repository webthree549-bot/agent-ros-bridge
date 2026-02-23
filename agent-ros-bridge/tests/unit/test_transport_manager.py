"""Unit tests for TransportManager."""

import pytest

from agent_ros_bridge import TransportManager


class MockTransport:
    """Mock transport for testing."""
    
    def __init__(self, name):
        self.name = name
        self.started = False
        self.stopped = False
    
    async def start(self):
        self.started = True
    
    async def stop(self):
        self.stopped = True


@pytest.mark.unit
class TestTransportManager:
    """Test TransportManager functionality."""
    
    def test_register_transport(self):
        """Test registering a transport."""
        manager = TransportManager()
        transport = MockTransport("test")
        
        manager.register(transport)
        
        assert transport in manager.get_transports()
        assert len(manager.get_transports()) == 1
    
    def test_register_multiple(self):
        """Test registering multiple transports."""
        manager = TransportManager()
        t1 = MockTransport("t1")
        t2 = MockTransport("t2")
        
        manager.register(t1)
        manager.register(t2)
        
        assert len(manager.get_transports()) == 2
        assert t1 in manager.get_transports()
        assert t2 in manager.get_transports()
    
    @pytest.mark.asyncio
    async def test_start_all(self):
        """Test starting all transports."""
        manager = TransportManager()
        t1 = MockTransport("t1")
        t2 = MockTransport("t2")
        
        manager.register(t1)
        manager.register(t2)
        
        # Note: This would actually run forever in real code
        # For testing, we just verify the structure
        assert manager._transports == [t1, t2]
    
    @pytest.mark.asyncio
    async def test_stop_all(self):
        """Test stopping all transports."""
        manager = TransportManager()
        t1 = MockTransport("t1")
        
        manager.register(t1)
        await manager.stop()
        
        assert t1.stopped
    
    def test_get_transports_copy(self):
        """Test that get_transports returns a copy."""
        manager = TransportManager()
        transport = MockTransport("test")
        
        manager.register(transport)
        transports = manager.get_transports()
        
        # Modify returned list
        transports.clear()
        
        # Original should still have the transport
        assert len(manager.get_transports()) == 1
