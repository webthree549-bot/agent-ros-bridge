"""Test references content for Agent ROS Bridge ClawHub skill.

TDD for references/ directory content.
"""

import re
import unittest
from pathlib import Path

SKILL_PATH = Path(__file__).parent.parent.parent / "skills" / "agent-ros-bridge"
REFERENCES_PATH = SKILL_PATH / "references"
SKILL_MD_PATH = SKILL_PATH / "SKILL.md"


class TestROS1Guide(unittest.TestCase):
    """Test ROS1 guide reference."""

    def setUp(self):
        self.ros1_path = REFERENCES_PATH / "ros1-guide.md"
        self.skill_content = SKILL_MD_PATH.read_text()

    def test_ros1_guide_exists_or_referenced(self):
        """Test that ROS1 guide exists or is properly referenced."""
        # Either file exists or skill doesn't claim ROS1 support
        if not self.ros1_path.exists():
            # Check if skill mentions ROS1
            if "ROS1" in self.skill_content or "ros1" in self.skill_content:
                self.fail(
                    "Skill mentions ROS1 but ros1-guide.md not found. "
                    "Create references/ros1-guide.md"
                )
            else:
                self.skipTest("Skill doesn't mention ROS1")

    def test_ros1_guide_has_common_topics(self):
        """Test that ROS1 guide covers common topics."""
        if not self.ros1_path.exists():
            self.skipTest("ROS1 guide not found")

        content = self.ros1_path.read_text()

        # Check for key ROS1 topics
        required_topics = [
            "roscore",
            "rospy",
            "Noetic",
        ]

        for topic in required_topics:
            self.assertIn(topic.lower(), content.lower(), f"ROS1 guide should mention {topic}")

    def test_ros1_guide_has_examples(self):
        """Test that ROS1 guide has code examples."""
        if not self.ros1_path.exists():
            self.skipTest("ROS1 guide not found")

        content = self.ros1_path.read_text()

        # Should have code blocks
        self.assertIn("```", content, "ROS1 guide should have code examples")


class TestROS2Guide(unittest.TestCase):
    """Test ROS2 guide reference."""

    def setUp(self):
        self.ros2_path = REFERENCES_PATH / "ros2-guide.md"
        self.skill_content = SKILL_MD_PATH.read_text()

    def test_ros2_guide_exists_or_referenced(self):
        """Test that ROS2 guide exists or is properly referenced."""
        if not self.ros2_path.exists():
            if "ROS2" in self.skill_content or "ros2" in self.skill_content:
                self.fail(
                    "Skill mentions ROS2 but ros2-guide.md not found. "
                    "Create references/ros2-guide.md"
                )
            else:
                self.skipTest("Skill doesn't mention ROS2")

    def test_ros2_guide_has_common_topics(self):
        """Test that ROS2 guide covers common topics."""
        if not self.ros2_path.exists():
            self.skipTest("ROS2 guide not found")

        content = self.ros2_path.read_text()

        # Check for key ROS2 topics
        required_topics = [
            "rclpy",
            "Jazzy",
            "Humble",
            "colcon",
        ]

        found_topics = [topic for topic in required_topics if topic.lower() in content.lower()]

        self.assertTrue(
            len(found_topics) >= 2, f"ROS2 guide should cover key topics. Found: {found_topics}"
        )

    def test_ros2_guide_has_examples(self):
        """Test that ROS2 guide has code examples."""
        if not self.ros2_path.exists():
            self.skipTest("ROS2 guide not found")

        content = self.ros2_path.read_text()

        self.assertIn("```", content, "ROS2 guide should have code examples")


class TestWebSocketAPIReference(unittest.TestCase):
    """Test WebSocket API reference."""

    def setUp(self):
        self.ws_path = REFERENCES_PATH / "websocket-api.md"

    def test_websocket_api_exists(self):
        """Test that WebSocket API reference exists."""
        # Optional but recommended
        if not self.ws_path.exists():
            self.skipTest("WebSocket API reference is optional")

    def test_websocket_api_has_message_format(self):
        """Test that WebSocket API documents message format."""
        if not self.ws_path.exists():
            self.skipTest("WebSocket API reference not found")

        content = self.ws_path.read_text()

        # Should document message structure
        self.assertIn("command", content.lower(), "WebSocket API should document command structure")

    def test_websocket_api_has_authentication(self):
        """Test that WebSocket API documents authentication."""
        if not self.ws_path.exists():
            self.skipTest("WebSocket API reference not found")

        content = self.ws_path.read_text()

        self.assertIn(
            "token", content.lower(), "WebSocket API should document token authentication"
        )


class TestExamples(unittest.TestCase):
    """Test examples in references/examples/."""

    def setUp(self):
        self.examples_path = REFERENCES_PATH / "examples"

    def test_basic_movement_example(self):
        """Test basic movement example exists."""
        if not self.examples_path.exists():
            self.skipTest("Examples directory not found")

        movement_file = self.examples_path / "basic-movement.md"

        # Optional but recommended
        if not movement_file.exists():
            self.skipTest("Basic movement example is optional")

    def test_sensor_reading_example(self):
        """Test sensor reading example exists."""
        if not self.examples_path.exists():
            self.skipTest("Examples directory not found")

        sensor_file = self.examples_path / "sensor-reading.md"

        if not sensor_file.exists():
            self.skipTest("Sensor reading example is optional")

    def test_fleet_management_example(self):
        """Test fleet management example exists."""
        if not self.examples_path.exists():
            self.skipTest("Examples directory not found")

        fleet_file = self.examples_path / "fleet-management.md"

        if not fleet_file.exists():
            self.skipTest("Fleet management example is optional")


if __name__ == "__main__":
    unittest.main()
