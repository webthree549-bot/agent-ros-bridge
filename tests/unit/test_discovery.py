"""Unit tests for ToolDiscovery."""

import pytest

from agent_ros_bridge.discovery import ToolDiscovery, DiscoveredTool, SafetyLevel, discover_tools
from agent_ros_bridge import ROSBridge


@pytest.mark.unit
class TestToolDiscovery:
    """Test ToolDiscovery functionality."""
    
    @pytest.fixture
    def discovery(self):
        """Create discovery with mock bridge."""
        bridge = ROSBridge(ros_version=2)
        return ToolDiscovery(bridge)
    
    def test_classify_safety_dangerous(self, discovery):
        """Test safety classification for dangerous actions."""
        assert discovery._classify_safety("cmd_vel") == SafetyLevel.DANGEROUS
        assert discovery._classify_safety("move_arm") == SafetyLevel.DANGEROUS
        assert discovery._classify_safety("grasp") == SafetyLevel.DANGEROUS
    
    def test_classify_safety_medium(self, discovery):
        """Test safety classification for medium risk."""
        assert discovery._classify_safety("navigate") == SafetyLevel.MEDIUM
        assert discovery._classify_safety("patrol") == SafetyLevel.MEDIUM
    
    def test_classify_safety_safe(self, discovery):
        """Test safety classification for safe actions."""
        assert discovery._classify_safety("battery_status") == SafetyLevel.SAFE
        assert discovery._classify_safety("camera") == SafetyLevel.SAFE
    
    def test_generate_description_known(self, discovery):
        """Test description generation for known actions."""
        desc = discovery._generate_description("navigate", "")
        assert "navigate" in desc.lower()
    
    def test_generate_description_unknown(self, discovery):
        """Test description generation for unknown actions."""
        desc = discovery._generate_description("custom_action", "")
        assert "custom_action" in desc or "Execute" in desc
    
    def test_infer_parameters_navigate(self, discovery):
        """Test parameter inference for navigation."""
        params = discovery._infer_parameters("navigate", "")
        assert "x" in params
        assert "y" in params
        assert params["x"]["type"] == "number"
    
    def test_infer_parameters_gripper(self, discovery):
        """Test parameter inference for gripper."""
        params = discovery._infer_parameters("grasp", "")
        assert "state" in params
        assert "enum" in params["state"]
    
    def test_discovered_tool_to_mcp(self, discovery):
        """Test conversion to MCP tool format."""
        tool = DiscoveredTool(
            name="test_action",
            type="action",
            description="Test action",
            ros_name="/test",
            message_type="std_msgs/String",
            safety_level=SafetyLevel.SAFE,
            parameters={"param1": {"type": "string"}}
        )
        
        mcp = tool.to_mcp_tool()
        assert mcp["name"] == "ros_test_action"
        assert mcp["description"] == "Test action"
        assert "param1" in mcp["inputSchema"]["properties"]


@pytest.mark.unit
class TestSafetyLevel:
    """Test SafetyLevel enum."""
    
    def test_safety_levels(self):
        """Test safety level values."""
        assert SafetyLevel.SAFE.value == "safe"
        assert SafetyLevel.MEDIUM.value == "medium"
        assert SafetyLevel.DANGEROUS.value == "dangerous"
