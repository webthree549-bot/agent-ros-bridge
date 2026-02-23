"""Unit tests for ActionRegistry."""

import pytest

from agent_ros_bridge import ActionRegistry


@pytest.mark.unit
class TestActionRegistry:
    """Test ActionRegistry functionality."""
    
    def test_register_action(self):
        """Test registering an action."""
        registry = ActionRegistry()
        
        def handler():
            return {"result": "ok"}
        
        registry.register("test_action", handler)
        
        assert "test_action" in registry.list_actions()
        assert registry.get("test_action") == handler
    
    def test_register_with_schema(self):
        """Test registering an action with JSON schema."""
        registry = ActionRegistry()
        
        schema = {
            "type": "object",
            "properties": {
                "x": {"type": "number"},
                "y": {"type": "number"}
            }
        }
        
        registry.register("navigate", lambda: None, schema)
        
        assert registry.get_schema("navigate") == schema
    
    def test_get_nonexistent_action(self):
        """Test getting an action that doesn't exist."""
        registry = ActionRegistry()
        
        assert registry.get("nonexistent") is None
        assert registry.get_schema("nonexistent") == {"type": "object"}
    
    def test_action_decorator(self):
        """Test the action decorator."""
        registry = ActionRegistry()
        
        @registry.action("decorated_action")
        def handler():
            return {"decorated": True}
        
        assert "decorated_action" in registry.list_actions()
        assert registry.get("decorated_action") == handler
    
    def test_list_actions_empty(self):
        """Test listing actions when empty."""
        registry = ActionRegistry()
        
        assert registry.list_actions() == []
    
    def test_default_schema(self):
        """Test default schema for actions without schema."""
        registry = ActionRegistry()
        
        registry.register("no_schema", lambda: None)
        
        default = registry.get_schema("no_schema")
        assert default == {"type": "object", "properties": {}}
