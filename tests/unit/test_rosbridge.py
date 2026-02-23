"""Unit tests for ROSBridge core."""

import pytest
import asyncio

from agent_ros_bridge import ROSBridge


@pytest.mark.unit
class TestROSBridge:
    """Test ROSBridge functionality."""
    
    def test_initialization(self):
        """Test ROSBridge initialization."""
        bridge = ROSBridge(ros_version=2)
        
        assert bridge.ros_version == 2
        assert bridge.action_registry is not None
        assert bridge.transport_manager is not None
        assert bridge.connector_manager is not None
        assert bridge.topic_manager is not None
    
    def test_ros_version_1(self):
        """Test ROS1 bridge initialization."""
        bridge = ROSBridge(ros_version=1)
        
        assert bridge.ros_version == 1
    
    def test_action_registration(self):
        """Test action registration via decorator."""
        bridge = ROSBridge(ros_version=2)
        
        @bridge.action("test_action")
        async def handler(x: float = 0):
            return {"x": x}
        
        assert "test_action" in bridge.get_registered_actions()
    
    @pytest.mark.asyncio
    async def test_call_action(self):
        """Test calling a registered action."""
        bridge = ROSBridge(ros_version=2)
        
        @bridge.action("add")
        async def add(x: float, y: float):
            return {"result": x + y}
        
        result = await bridge.call_action("add", x=2, y=3)
        
        assert result["success"] is True
        assert result["action"] == "add"
        assert result["result"] == 5
    
    @pytest.mark.asyncio
    async def test_call_nonexistent_action(self):
        """Test calling an action that doesn't exist."""
        bridge = ROSBridge(ros_version=2)
        
        with pytest.raises(ValueError, match="Action 'missing' not found"):
            await bridge.call_action("missing")
    
    @pytest.mark.asyncio
    async def test_call_action_with_error(self):
        """Test calling an action that raises an exception."""
        bridge = ROSBridge(ros_version=2)
        
        @bridge.action("fail")
        async def fail():
            raise RuntimeError("Intentional failure")
        
        result = await bridge.call_action("fail")
        
        assert result["success"] is False
        assert "error" in result
        assert "Intentional failure" in result["error"]
    
    def test_get_registered_actions_empty(self):
        """Test getting actions when none registered."""
        bridge = ROSBridge(ros_version=2)
        
        assert bridge.get_registered_actions() == []
    
    def test_session_management(self):
        """Test session creation and retrieval."""
        bridge = ROSBridge(ros_version=2)
        
        session = bridge.create_session("agent_1", {"read", "execute"})
        
        assert session.agent_id == "agent_1"
        assert session.permissions == {"read", "execute"}
        assert bridge.get_session(session.session_id) == session
    
    def test_close_session(self):
        """Test closing a session."""
        bridge = ROSBridge(ros_version=2)
        
        session = bridge.create_session("agent_1")
        session_id = session.session_id
        
        bridge.close_session(session_id)
        
        assert bridge.get_session(session_id) is None
    
    def test_get_available_topics_empty(self):
        """Test getting topics when no connectors."""
        bridge = ROSBridge(ros_version=2)
        
        topics = bridge.get_available_topics()
        
        assert topics == []
