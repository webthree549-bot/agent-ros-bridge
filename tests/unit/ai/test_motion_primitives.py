"""Tests for motion primitives."""

from unittest.mock import MagicMock, Mock, patch

import pytest

# These tests require ROS2 environment
# Run with: docker exec ros2_humble bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace && python3 -m pytest tests/unit/ai/test_motion_primitives.py -v"

# Try to import rclpy, skip all tests in this module if not available
try:
    import rclpy

    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False

if RCLPY_AVAILABLE:
    from agent_ros_bridge.ai.motion_primitives import (
        GripperControlPrimitive,
        MotionPrimitive,
        MotionPrimitiveFactory,
        NavigateToPosePrimitive,
        RotateInPlacePrimitive,
    )

pytestmark = pytest.mark.skipif(not RCLPY_AVAILABLE, reason="ROS2 rclpy not available")


class TestMotionPrimitiveBase:
    """Test base motion primitive."""

    def test_to_dict(self):
        """Test converting to dictionary."""
        primitive = MotionPrimitive(
            type="TEST",
            primitive_id="test_primitive",
            parameters={"key": "value"},
            expected_duration=5.0,
        )

        data = primitive.to_dict()

        assert data["type"] == "TEST"
        assert data["primitive_id"] == "test_primitive"
        assert data["parameters"]["key"] == "value"
        assert data["expected_duration"] == 5.0


class TestNavigateToPosePrimitive:
    """Test navigate to pose primitive."""

    def test_creation(self):
        """Test creating navigate primitive."""
        primitive = NavigateToPosePrimitive(
            target_pose=None,
            max_velocity=0.5,
        )
        assert primitive.type == "NAVIGATE"
        assert primitive.primitive_id == "navigate_to_pose"
        assert primitive.parameters["max_velocity"] == 0.5

    def test_validate_with_pose(self):
        """Test validation with valid pose."""
        mock_pose = Mock()
        primitive = NavigateToPosePrimitive(target_pose=mock_pose)
        assert primitive.validate() is True

    def test_validate_without_pose(self):
        """Test validation without pose."""
        primitive = NavigateToPosePrimitive(target_pose=None)
        assert primitive.validate() is False


class TestGripperControlPrimitive:
    """Test gripper control primitive."""

    def test_creation_open(self):
        """Test creating open gripper primitive."""
        primitive = GripperControlPrimitive(action="open")
        assert primitive.type == "GRIPPER"
        assert primitive.parameters["action"] == "open"

    def test_creation_close(self):
        """Test creating close gripper primitive."""
        primitive = GripperControlPrimitive(action="close")
        assert primitive.parameters["action"] == "close"

    def test_validate_open(self):
        """Test validation for open action."""
        primitive = GripperControlPrimitive(action="open")
        assert primitive.validate() is True

    def test_validate_close(self):
        """Test validation for close action."""
        primitive = GripperControlPrimitive(action="close")
        assert primitive.validate() is True

    def test_validate_invalid(self):
        """Test validation with invalid action."""
        primitive = GripperControlPrimitive(action="invalid")
        assert primitive.validate() is False


class TestRotateInPlacePrimitive:
    """Test rotate in place primitive."""

    def test_creation(self):
        """Test creating rotate primitive."""
        primitive = RotateInPlacePrimitive(angle=90.0)
        # Rotate is a type of NAVIGATE
        assert primitive.type == "NAVIGATE"
        assert primitive.parameters["angle"] == 90.0

    def test_validate(self):
        """Test validation."""
        primitive = RotateInPlacePrimitive(angle=90.0)
        assert primitive.validate() is True


class TestMotionPrimitiveFactory:
    """Test motion primitive factory."""

    def test_create_navigate_to_pose(self):
        """Test creating navigate primitive."""
        factory = MotionPrimitiveFactory()
        mock_pose = Mock()
        primitive = factory.create_navigate_to_pose(target_pose=mock_pose)

        assert isinstance(primitive, NavigateToPosePrimitive)

    def test_create_gripper_control(self):
        """Test creating gripper primitive."""
        factory = MotionPrimitiveFactory()
        primitive = factory.create_gripper_control(action="open")

        assert isinstance(primitive, GripperControlPrimitive)

    def test_create_rotate_in_place(self):
        """Test creating rotate primitive."""
        factory = MotionPrimitiveFactory()
        primitive = factory.create_rotate_in_place(angle=90.0)

        assert isinstance(primitive, RotateInPlacePrimitive)

    def test_create_from_dict(self):
        """Test creating from dictionary."""
        factory = MotionPrimitiveFactory()
        data = {
            "type": "GRIPPER",
            "primitive_id": "gripper_control",
            "parameters": {"action": "open"},
        }
        primitive = factory.create_from_dict(data)

        assert primitive is not None
        assert primitive.type == "GRIPPER"
