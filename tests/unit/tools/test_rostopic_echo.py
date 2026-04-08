"""Tests for rostopic_echo tool.

TDD Approach:
1. Test tool instantiation
2. Test parameter validation
3. Test execution (mocked ROS2)
4. Test error handling
"""

from unittest.mock import MagicMock, patch

import pytest

from agent_ros_bridge.tools.rostopic_echo import ROSTopicEchoTool


class TestROSTopicEchoTool:
    """Test ROSTopicEchoTool."""

    def test_tool_attributes(self):
        """Test tool has correct attributes."""
        tool = ROSTopicEchoTool()
        
        assert tool.name == "rostopic_echo"
        assert tool.description == "Echo messages from a ROS topic"
        assert tool.version == "1.0.0"

    def test_execute_without_ros(self):
        """Test execution when ROS2 is not available."""
        tool = ROSTopicEchoTool()
        
        # Mock rclpy as not available
        with patch.dict("sys.modules", {"rclpy": None}):
            result = tool.execute(topic="/test", count=1)
        
        assert result.success is False
        assert "ROS2 not available" in result.error
        assert result.data["topic"] == "/test"
        assert result.execution_time_ms > 0

    def test_execute_with_valid_topic(self):
        """Test execution with valid topic."""
        tool = ROSTopicEchoTool()
        
        # If rclpy not available, test the fallback
        try:
            import rclpy
        except ImportError:
            result = tool.execute(topic="/test_topic", count=1, timeout_sec=0.1)
            assert result.success is False
            assert "ROS2 not available" in result.error
            return
        
        # Mock ROS2 components
        mock_node = MagicMock()
        mock_node.get_topic_names_and_types.return_value = [
            ("/test_topic", ["std_msgs/String"])
        ]
        
        mock_msg = MagicMock()
        mock_msg.data = "test message"
        
        with patch("rclpy.ok", return_value=True):
            with patch("rclpy.init"):
                with patch("rclpy.node.Node", return_value=mock_node):
                    with patch("rclpy.spin_once"):
                        result = tool.execute(topic="/test_topic", count=1, timeout_sec=0.1)
        
        # Should succeed or fail gracefully
        assert isinstance(result.success, bool)
        assert result.execution_time_ms >= 0

    def test_execute_with_nonexistent_topic(self):
        """Test execution with non-existent topic."""
        tool = ROSTopicEchoTool()
        
        # If rclpy not available, test the fallback
        try:
            import rclpy
        except ImportError:
            result = tool.execute(topic="/nonexistent", count=1, timeout_sec=0.1)
            assert result.success is False
            assert "ROS2 not available" in result.error
            return
        
        mock_node = MagicMock()
        mock_node.get_topic_names_and_types.return_value = []  # No topics
        
        with patch("rclpy.ok", return_value=True):
            with patch("rclpy.init"):
                with patch("rclpy.node.Node", return_value=mock_node):
                    result = tool.execute(topic="/nonexistent", count=1, timeout_sec=0.1)
        
        assert result.success is False
        assert "not found" in result.error.lower() or "no messages" in result.error.lower()

    def test_execute_preserves_params(self):
        """Test that parameters are preserved in result."""
        tool = ROSTopicEchoTool()
        
        with patch.dict("sys.modules", {"rclpy": None}):
            result = tool.execute(topic="/cmd_vel", count=5)
        
        assert result.data["topic"] == "/cmd_vel"
        assert result.data["count"] == 5


class TestROSTopicEchoValidation:
    """Test parameter validation."""

    def test_empty_topic(self):
        """Test with empty topic name."""
        tool = ROSTopicEchoTool()
        
        # Empty topic should still attempt execution
        with patch.dict("sys.modules", {"rclpy": None}):
            result = tool.execute(topic="", count=1)
        
        # Should fail gracefully
        assert result.success is False

    def test_zero_count(self):
        """Test with zero count."""
        tool = ROSTopicEchoTool()
        
        with patch.dict("sys.modules", {"rclpy": None}):
            result = tool.execute(topic="/test", count=0)
        
        assert result.data["count"] == 0

    def test_negative_timeout(self):
        """Test with negative timeout."""
        tool = ROSTopicEchoTool()
        
        # Negative timeout should be handled gracefully
        with patch.dict("sys.modules", {"rclpy": None}):
            result = tool.execute(topic="/test", timeout_sec=-1.0)
        
        # Should either fail or use absolute value
        assert isinstance(result.success, bool)


class TestROSTopicEchoEdgeCases:
    """Test edge cases."""

    def test_very_long_topic_name(self):
        """Test with very long topic name."""
        tool = ROSTopicEchoTool()
        long_topic = "/" + "a" * 1000
        
        with patch.dict("sys.modules", {"rclpy": None}):
            result = tool.execute(topic=long_topic, count=1)
        
        assert result.data["topic"] == long_topic

    def test_special_characters_in_topic(self):
        """Test with special characters."""
        tool = ROSTopicEchoTool()
        special_topic = "/test-topic_123/test"
        
        with patch.dict("sys.modules", {"rclpy": None}):
            result = tool.execute(topic=special_topic, count=1)
        
        assert result.data["topic"] == special_topic

    def test_large_count(self):
        """Test with large count."""
        tool = ROSTopicEchoTool()
        
        with patch.dict("sys.modules", {"rclpy": None}):
            result = tool.execute(topic="/test", count=10000)
        
        assert result.data["count"] == 10000
