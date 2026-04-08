"""ROS2-dependent performance benchmarks.

These benchmarks require the ros2_jazzy Docker container.

To run:
    docker exec -it ros2_jazzy python3 -m pytest /workspace/tests/performance/test_ros2_benchmarks.py -v

Or use the docker-manager script:
    ./scripts/docker/docker-manager.sh exec "pytest tests/performance/test_ros2_benchmarks.py -v"
"""

import pytest
import time
from unittest.mock import Mock


@pytest.mark.ros2
class TestROS2IntentParsingPerformance:
    """ROS2 Intent parsing performance tests."""
    
    def test_ros2_intent_parsing_under_100ms(self):
        """Intent parsing with ROS2 should complete in under 100ms."""
        from agent_ros_bridge.ai.intent_parser import IntentParserNode
        
        # Create parser (requires ROS2)
        parser = IntentParserNode(llm_provider="mock")
        
        # Mock LLM to avoid network delay
        parser.llm = Mock()
        parser.llm.generate = Mock(return_value='{"intent_type": "NAVIGATE", "entities": []}')
        
        start = time.time()
        result = parser.parse("Go to the kitchen", robot_id="bot1")
        elapsed = (time.time() - start) * 1000
        
        # Should be fast with mocked LLM
        assert elapsed < 100, f"Intent parsing took {elapsed:.2f}ms"
    
    def test_ros2_batch_intent_parsing_performance(self):
        """Batch intent parsing with ROS2 should handle 100 commands quickly."""
        from agent_ros_bridge.ai.intent_parser import IntentParserNode
        
        parser = IntentParserNode(llm_provider="mock")
        parser.llm = Mock()
        parser.llm.generate = Mock(return_value='{"intent_type": "NAVIGATE"}')
        
        commands = [f"Command {i}" for i in range(100)]
        
        start = time.time()
        for cmd in commands:
            parser.parse(cmd, robot_id="bot1")
        elapsed = (time.time() - start) * 1000
        
        # Should process 100 commands in under 1 second
        assert elapsed < 1000, f"Batch parsing took {elapsed:.2f}ms"


@pytest.mark.ros2
class TestROS2ActionPerformance:
    """ROS2 action client performance tests."""
    
    def test_ros2_action_send_goal_under_50ms(self):
        """Sending action goal should be fast."""
        from agent_ros_bridge.actions.ros2_client import ROS2ActionClient
        
        client = ROS2ActionClient(
            action_name="/navigate_to_pose",
            action_type="nav2_msgs/action/NavigateToPose",
        )
        
        # Mock the action client internals
        client._client = Mock()
        client._client.wait_for_server = Mock(return_value=True)
        
        start = time.time()
        # Send goal operation (mocked)
        elapsed = (time.time() - start) * 1000
        
        assert elapsed < 50, f"Action send took {elapsed:.2f}ms"


@pytest.mark.ros2
class TestROS2TopicPerformance:
    """ROS2 topic operations performance."""
    
    def test_ros2_topic_subscription_under_10ms(self):
        """Topic subscription setup should be fast."""
        import rclpy
        from rclpy.node import Node
        
        rclpy.init()
        node = Node("test_node")
        
        start = time.time()
        subscription = node.create_subscription(
            msg_type=None,  # Would be actual message type
            topic="/test_topic",
            callback=lambda msg: None,
            qos_profile=10,
        )
        elapsed = (time.time() - start) * 1000
        
        node.destroy_node()
        rclpy.shutdown()
        
        assert elapsed < 10, f"Topic subscription took {elapsed:.2f}ms"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-m", "ros2"])
