"""
Unit tests for Tool Discovery.
Tests ROS tool discovery and export to AI formats (MCP, OpenAI).
"""

import json
import pytest
from unittest import mock

from agent_ros_bridge.integrations.discovery import (
    ToolDiscovery, ROSAction
)


class TestROSAction:
    """Test ROSAction dataclass"""
    
    def test_action_creation(self):
        """Test basic ROS action creation"""
        action = ROSAction(
            name="/robot/move",
            action_type="topic",
            ros_type="geometry_msgs/Twist",
            description="Move the robot",
            parameters={"linear": {"type": "object"}, "angular": {"type": "object"}},
            dangerous=False
        )
        
        assert action.name == "/robot/move"
        assert action.action_type == "topic"
        assert action.ros_type == "geometry_msgs/Twist"
        assert action.dangerous is False
    
    def test_action_default_dangerous(self):
        """Test dangerous defaults to False"""
        action = ROSAction(
            name="/robot/status",
            action_type="topic",
            ros_type="std_msgs/String",
            description="Robot status",
            parameters={}
        )
        
        assert action.dangerous is False


class TestToolDiscoveryBasics:
    """Test ToolDiscovery basic functionality"""
    
    @pytest.fixture
    def discovery(self):
        """Create ToolDiscovery without bridge"""
        return ToolDiscovery(bridge=None)
    
    @pytest.fixture
    def sample_tools(self):
        """Create sample tools"""
        return [
            ROSAction(
                name="/robot/move",
                action_type="topic",
                ros_type="geometry_msgs/Twist",
                description="Move the robot base",
                parameters={
                    "linear": {"type": "object", "description": "Linear velocity"},
                    "angular": {"type": "object", "description": "Angular velocity"}
                },
                dangerous=True
            ),
            ROSAction(
                name="/robot/arm/move",
                action_type="topic",
                ros_type="trajectory_msgs/JointTrajectory",
                description="Move the robot arm",
                parameters={
                    "joint_names": {"type": "array"},
                    "points": {"type": "array"}
                },
                dangerous=True
            ),
            ROSAction(
                name="/robot/status",
                action_type="topic",
                ros_type="std_msgs/String",
                description="Get robot status",
                parameters={},
                dangerous=False
            ),
            ROSAction(
                name="/robot/capture_image",
                action_type="service",
                ros_type="sensor_msgs/Image",
                description="Capture camera image",
                parameters={"width": {"type": "integer"}, "height": {"type": "integer"}},
                dangerous=False
            )
        ]
    
    def test_initialization_no_bridge(self, discovery):
        """Test initialization without bridge"""
        assert discovery.bridge is None
        assert discovery._cache == {}
    
    def test_initialization_with_bridge(self):
        """Test initialization with bridge"""
        mock_bridge = mock.MagicMock()
        discovery = ToolDiscovery(bridge=mock_bridge)
        
        assert discovery.bridge == mock_bridge
    
    def test_discover_all_no_bridge_empty(self, discovery):
        """Test discover_all returns empty without bridge and cache"""
        tools = discovery.discover_all()
        
        assert tools == []
    
    def test_discover_all_returns_cached(self, discovery, sample_tools):
        """Test discover_all returns cached tools when no bridge"""
        # Pre-populate cache
        discovery._cache = {tool.name: tool for tool in sample_tools}
        
        tools = discovery.discover_all()
        
        assert len(tools) == 4
        assert all(isinstance(t, ROSAction) for t in tools)


class TestToolDiscoveryWithBridge:
    """Test ToolDiscovery with mocked bridge"""
    
    @pytest.fixture
    def mock_bridge(self):
        """Create mock bridge"""
        return mock.MagicMock()
    
    @pytest.fixture
    def discovery_with_bridge(self, mock_bridge):
        """Create ToolDiscovery with mocked bridge"""
        return ToolDiscovery(bridge=mock_bridge)
    
    def test_discover_topics_called(self, discovery_with_bridge, mock_bridge):
        """Test _discover_topics is called during discover_all"""
        with mock.patch.object(discovery_with_bridge, '_discover_topics') as mock_discover:
            mock_discover.return_value = []
            discovery_with_bridge.discover_all()
            
            mock_discover.assert_called_once()
    
    def test_discover_services_called(self, discovery_with_bridge, mock_bridge):
        """Test _discover_services is called during discover_all"""
        with mock.patch.object(discovery_with_bridge, '_discover_services') as mock_discover:
            mock_discover.return_value = []
            discovery_with_bridge.discover_all()
            
            mock_discover.assert_called_once()
    
    def test_discover_actions_called(self, discovery_with_bridge, mock_bridge):
        """Test _discover_actions is called during discover_all"""
        with mock.patch.object(discovery_with_bridge, '_discover_actions') as mock_discover:
            mock_discover.return_value = []
            discovery_with_bridge.discover_all()
            
            mock_discover.assert_called_once()
    
    def test_cache_updated_after_discover(self, discovery_with_bridge):
        """Test cache is updated after discovery"""
        tool = ROSAction(
            name="/test/topic",
            action_type="topic",
            ros_type="std_msgs/String",
            description="Test",
            parameters={}
        )
        
        with mock.patch.object(discovery_with_bridge, '_discover_topics') as mock_discover:
            mock_discover.return_value = [tool]
            with mock.patch.object(discovery_with_bridge, '_discover_services') as mock_svc:
                mock_svc.return_value = []
                with mock.patch.object(discovery_with_bridge, '_discover_actions') as mock_act:
                    mock_act.return_value = []
                    discovery_with_bridge.discover_all()
        
        assert "/test/topic" in discovery_with_bridge._cache
        assert discovery_with_bridge._cache["/test/topic"].name == "/test/topic"


class TestMCPToolExport:
    """Test MCP tool format export"""
    
    @pytest.fixture
    def discovery(self):
        """Create ToolDiscovery"""
        return ToolDiscovery(bridge=None)
    
    @pytest.fixture
    def sample_tools(self):
        """Create sample tools"""
        return [
            ROSAction(
                name="robot_move",
                action_type="topic",
                ros_type="geometry_msgs/Twist",
                description="Move the robot",
                parameters={
                    "linear": {"type": "object", "description": "Linear velocity"},
                    "angular": {"type": "object", "description": "Angular velocity"}
                }
            ),
            ROSAction(
                name="capture_image",
                action_type="service",
                ros_type="sensor_msgs/Image",
                description="Capture image",
                parameters={
                    "width": {"type": "integer"},
                    "height": {"type": "integer"}
                }
            )
        ]
    
    def test_to_mcp_tools_format(self, discovery, sample_tools):
        """Test MCP tool format structure"""
        mcp_tools = discovery.to_mcp_tools(sample_tools)
        
        assert len(mcp_tools) == 2
        
        # Check structure of first tool
        tool = mcp_tools[0]
        assert "name" in tool
        assert "description" in tool
        assert "inputSchema" in tool
        assert tool["inputSchema"]["type"] == "object"
        assert "properties" in tool["inputSchema"]
        assert "required" in tool["inputSchema"]
    
    def test_to_mcp_tools_uses_all_parameters(self, discovery):
        """Test all parameters become required"""
        tool = ROSAction(
            name="test_action",
            action_type="topic",
            ros_type="std_msgs/String",
            description="Test",
            parameters={"param1": {"type": "string"}, "param2": {"type": "integer"}}
        )
        
        mcp_tools = discovery.to_mcp_tools([tool])
        
        assert set(mcp_tools[0]["inputSchema"]["required"]) == {"param1", "param2"}
    
    def test_to_mcp_tools_no_params(self, discovery):
        """Test tool with no parameters"""
        tool = ROSAction(
            name="status",
            action_type="topic",
            ros_type="std_msgs/String",
            description="Get status",
            parameters={}
        )
        
        mcp_tools = discovery.to_mcp_tools([tool])
        
        assert mcp_tools[0]["inputSchema"]["properties"] == {}
        assert mcp_tools[0]["inputSchema"]["required"] == []
    
    def test_to_mcp_tools_uses_cache_if_none_provided(self, discovery):
        """Test to_mcp_tools uses cached tools if none provided"""
        tool = ROSAction(
            name="cached_tool",
            action_type="topic",
            ros_type="std_msgs/String",
            description="Cached",
            parameters={}
        )
        discovery._cache[tool.name] = tool
        
        mcp_tools = discovery.to_mcp_tools()  # No tools provided
        
        assert len(mcp_tools) == 1
        assert mcp_tools[0]["name"] == "cached_tool"


class TestOpenAIExport:
    """Test OpenAI function format export"""
    
    @pytest.fixture
    def discovery(self):
        """Create ToolDiscovery"""
        return ToolDiscovery(bridge=None)
    
    def test_to_openai_functions_format(self, discovery):
        """Test OpenAI function format structure"""
        tool = ROSAction(
            name="robot_move",
            action_type="topic",
            ros_type="geometry_msgs/Twist",
            description="Move the robot",
            parameters={"linear": {"type": "object"}}
        )
        
        functions = discovery.to_openai_functions([tool])
        
        assert len(functions) == 1
        
        func = functions[0]
        assert func["type"] == "function"
        assert "function" in func
        assert func["function"]["name"] == "robot_move"
        assert func["function"]["description"] == "Move the robot"
        assert "parameters" in func["function"]
    
    def test_to_openai_functions_parameters(self, discovery):
        """Test OpenAI function parameters"""
        tool = ROSAction(
            name="complex_action",
            action_type="service",
            ros_type="std_msgs/String",
            description="Complex",
            parameters={
                "x": {"type": "number"},
                "y": {"type": "number"},
                "label": {"type": "string"}
            }
        )
        
        functions = discovery.to_openai_functions([tool])
        params = functions[0]["function"]["parameters"]
        
        assert params["type"] == "object"
        assert "x" in params["properties"]
        assert "y" in params["properties"]
        assert "label" in params["properties"]
        assert set(params["required"]) == {"x", "y", "label"}


class TestDangerousTools:
    """Test dangerous tool filtering"""
    
    @pytest.fixture
    def discovery(self):
        """Create ToolDiscovery"""
        return ToolDiscovery(bridge=None)
    
    @pytest.fixture
    def mixed_tools(self):
        """Create mix of dangerous and safe tools"""
        return [
            ROSAction(
                name="safe_status",
                action_type="topic",
                ros_type="std_msgs/String",
                description="Safe",
                parameters={},
                dangerous=False
            ),
            ROSAction(
                name="dangerous_move",
                action_type="topic",
                ros_type="geometry_msgs/Twist",
                description="Move robot",
                parameters={},
                dangerous=True
            ),
            ROSAction(
                name="dangerous_arm",
                action_type="topic",
                ros_type="trajectory_msgs/JointTrajectory",
                description="Move arm",
                parameters={},
                dangerous=True
            )
        ]
    
    def test_get_dangerous_tools(self, discovery, mixed_tools):
        """Test filtering dangerous tools"""
        # Populate cache
        discovery._cache = {tool.name: tool for tool in mixed_tools}
        
        dangerous = discovery.get_dangerous_tools()
        
        assert len(dangerous) == 2
        assert all(t.dangerous for t in dangerous)
        assert set(t.name for t in dangerous) == {"dangerous_move", "dangerous_arm"}
    
    def test_get_dangerous_tools_empty(self, discovery):
        """Test no dangerous tools returns empty"""
        safe_tools = [
            ROSAction(
                name="safe1",
                action_type="topic",
                ros_type="std_msgs/String",
                description="Safe",
                parameters={},
                dangerous=False
            )
        ]
        discovery._cache = {tool.name: tool for tool in safe_tools}
        
        dangerous = discovery.get_dangerous_tools()
        
        assert dangerous == []


class TestCacheManagement:
    """Test cache operations"""
    
    @pytest.fixture
    def discovery(self):
        """Create ToolDiscovery"""
        return ToolDiscovery(bridge=None)
    
    def test_get_tool_cached(self, discovery):
        """Test getting cached tool by name"""
        tool = ROSAction(
            name="cached_tool",
            action_type="topic",
            ros_type="std_msgs/String",
            description="Cached",
            parameters={}
        )
        discovery._cache["cached_tool"] = tool
        
        result = discovery.get_tool("cached_tool")
        
        assert result == tool
    
    def test_get_tool_not_found(self, discovery):
        """Test getting non-existent tool returns None"""
        result = discovery.get_tool("nonexistent")
        
        assert result is None
    
    def test_invalidate_cache(self, discovery):
        """Test cache invalidation"""
        tool = ROSAction(
            name="tool1",
            action_type="topic",
            ros_type="std_msgs/String",
            description="Test",
            parameters={}
        )
        discovery._cache["tool1"] = tool
        
        discovery.invalidate_cache()
        
        assert discovery._cache == {}
    
    def test_cache_persists_across_calls(self, discovery):
        """Test cache persists across discover_all calls"""
        tool1 = ROSAction(
            name="tool1",
            action_type="topic",
            ros_type="std_msgs/String",
            description="Test1",
            parameters={}
        )
        tool2 = ROSAction(
            name="tool2",
            action_type="topic",
            ros_type="std_msgs/String",
            description="Test2",
            parameters={}
        )
        
        discovery._cache["tool1"] = tool1
        
        # First call
        tools = discovery.discover_all()
        assert len(tools) == 1
        
        # Add another tool to cache
        discovery._cache["tool2"] = tool2
        
        # Second call should see both
        tools = discovery.discover_all()
        assert len(tools) == 2


class TestEdgeCases:
    """Test edge cases"""
    
    @pytest.fixture
    def discovery(self):
        """Create ToolDiscovery"""
        return ToolDiscovery(bridge=None)
    
    def test_empty_tools_to_mcp(self, discovery):
        """Test converting empty tools to MCP"""
        mcp_tools = discovery.to_mcp_tools([])
        
        assert mcp_tools == []
    
    def test_empty_tools_to_openai(self, discovery):
        """Test converting empty tools to OpenAI"""
        functions = discovery.to_openai_functions([])
        
        assert functions == []
    
    def test_tool_with_special_chars_in_name(self, discovery):
        """Test tool names with special characters"""
        tool = ROSAction(
            name="/robot/arm/joint_1/set_position",
            action_type="topic",
            ros_type="std_msgs/Float64",
            description="Set position",
            parameters={"position": {"type": "number"}}
        )
        
        mcp_tools = discovery.to_mcp_tools([tool])
        
        assert mcp_tools[0]["name"] == "/robot/arm/joint_1/set_position"
