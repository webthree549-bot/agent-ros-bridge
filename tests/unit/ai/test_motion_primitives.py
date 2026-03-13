"""Unit tests for motion primitives library.

TDD Approach: Write failing tests first, then implement to pass.
"""

# Mock ROS2 modules before importing our code
import sys
from unittest.mock import MagicMock

# Create mock for geometry_msgs
mock_geometry_msgs = MagicMock()
mock_pose = MagicMock()
mock_pose.pose.position.x = 1.0
mock_pose.pose.position.y = 2.0
mock_pose.pose.position.z = 0.0
mock_pose.pose.orientation.w = 1.0
mock_geometry_msgs.msg.PoseStamped = MagicMock(return_value=mock_pose)

sys.modules["geometry_msgs"] = mock_geometry_msgs
sys.modules["geometry_msgs.msg"] = mock_geometry_msgs.msg


class TestMotionPrimitive:
    """Tests for base MotionPrimitive class."""

    def test_motion_primitive_base_class_exists(self):
        """RED: MotionPrimitive base class should exist."""
        from agent_ros_bridge.ai.motion_primitives import MotionPrimitive

        assert MotionPrimitive is not None

    def test_motion_primitive_has_required_fields(self):
        """RED: MotionPrimitive should have type, primitive_id, target_pose, parameters, expected_duration."""
        # Check dataclass fields exist
        import inspect

        from agent_ros_bridge.ai.motion_primitives import MotionPrimitive

        sig = inspect.signature(MotionPrimitive.__init__)
        params = list(sig.parameters.keys())

        assert "type" in params
        assert "primitive_id" in params
        assert "target_pose" in params
        assert "parameters" in params
        assert "expected_duration" in params


class TestNavigateToPosePrimitive:
    """Tests for NavigateToPosePrimitive."""

    def test_navigate_to_pose_primitive_exists(self):
        """RED: NavigateToPosePrimitive should exist."""
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        assert NavigateToPosePrimitive is not None

    def test_navigate_to_pose_creates_navigate_primitive(self):
        """RED: Creates NAVIGATE primitive with target pose."""
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        target_pose = MagicMock()
        target_pose.pose.position.x = 1.0
        target_pose.pose.position.y = 2.0

        primitive = NavigateToPosePrimitive(target_pose=target_pose)

        assert primitive.type == "NAVIGATE"
        assert primitive.primitive_id == "navigate_to_pose"
        assert primitive.target_pose == target_pose

    def test_navigate_to_pose_has_default_max_velocity(self):
        """RED: Should have default max_velocity of 0.5."""
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose)

        assert primitive.parameters.get("max_velocity") == 0.5

    def test_navigate_to_pose_accepts_custom_max_velocity(self):
        """RED: Should accept custom max_velocity parameter."""
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose, max_velocity=1.0)

        assert primitive.parameters.get("max_velocity") == 1.0

    def test_navigate_to_pose_validates_with_valid_pose(self):
        """RED: Should validate successfully with valid pose."""
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose)

        assert primitive.validate() is True

    def test_navigate_to_pose_rejects_none_pose(self):
        """RED: Should reject None target_pose."""
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        primitive = NavigateToPosePrimitive(target_pose=None)

        assert primitive.validate() is False

    def test_navigate_to_pose_rejects_invalid_velocity(self):
        """RED: Should reject max_velocity <= 0."""
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose, max_velocity=0.0)

        assert primitive.validate() is False

    def test_navigate_to_pose_rejects_negative_velocity(self):
        """RED: Should reject negative max_velocity."""
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose, max_velocity=-1.0)

        assert primitive.validate() is False

    def test_navigate_to_pose_has_expected_duration(self):
        """RED: Should have expected_duration > 0."""
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose)

        assert primitive.expected_duration > 0


class TestPickObjectPrimitive:
    """Tests for PickObjectPrimitive."""

    def test_pick_object_primitive_exists(self):
        """RED: PickObjectPrimitive should exist."""
        from agent_ros_bridge.ai.motion_primitives import PickObjectPrimitive

        assert PickObjectPrimitive is not None

    def test_pick_object_creates_manipulate_primitive(self):
        """RED: Creates MANIPULATE primitive with object_id."""
        from agent_ros_bridge.ai.motion_primitives import PickObjectPrimitive

        grasp_pose = MagicMock()
        primitive = PickObjectPrimitive(object_id="obj_123", grasp_pose=grasp_pose)

        assert primitive.type == "MANIPULATE"
        assert primitive.primitive_id == "pick_object"
        assert primitive.parameters.get("object_id") == "obj_123"

    def test_pick_object_has_default_gripper_force(self):
        """RED: Should have default gripper_force of 10.0."""
        from agent_ros_bridge.ai.motion_primitives import PickObjectPrimitive

        grasp_pose = MagicMock()
        primitive = PickObjectPrimitive(object_id="obj_123", grasp_pose=grasp_pose)

        assert primitive.parameters.get("gripper_force") == 10.0

    def test_pick_object_accepts_custom_gripper_force(self):
        """RED: Should accept custom gripper_force parameter."""
        from agent_ros_bridge.ai.motion_primitives import PickObjectPrimitive

        grasp_pose = MagicMock()
        primitive = PickObjectPrimitive(
            object_id="obj_123", grasp_pose=grasp_pose, gripper_force=20.0
        )

        assert primitive.parameters.get("gripper_force") == 20.0

    def test_pick_object_validates_with_valid_params(self):
        """RED: Should validate successfully with valid parameters."""
        from agent_ros_bridge.ai.motion_primitives import PickObjectPrimitive

        grasp_pose = MagicMock()
        primitive = PickObjectPrimitive(object_id="obj_123", grasp_pose=grasp_pose)

        assert primitive.validate() is True

    def test_pick_object_rejects_empty_object_id(self):
        """RED: Should reject empty object_id."""
        from agent_ros_bridge.ai.motion_primitives import PickObjectPrimitive

        grasp_pose = MagicMock()
        primitive = PickObjectPrimitive(object_id="", grasp_pose=grasp_pose)

        assert primitive.validate() is False

    def test_pick_object_rejects_none_object_id(self):
        """RED: Should reject None object_id."""
        from agent_ros_bridge.ai.motion_primitives import PickObjectPrimitive

        grasp_pose = MagicMock()
        primitive = PickObjectPrimitive(object_id=None, grasp_pose=grasp_pose)

        assert primitive.validate() is False

    def test_pick_object_rejects_invalid_gripper_force(self):
        """RED: Should reject gripper_force <= 0."""
        from agent_ros_bridge.ai.motion_primitives import PickObjectPrimitive

        grasp_pose = MagicMock()
        primitive = PickObjectPrimitive(
            object_id="obj_123", grasp_pose=grasp_pose, gripper_force=0.0
        )

        assert primitive.validate() is False


class TestPlaceObjectPrimitive:
    """Tests for PlaceObjectPrimitive."""

    def test_place_object_primitive_exists(self):
        """RED: PlaceObjectPrimitive should exist."""
        from agent_ros_bridge.ai.motion_primitives import PlaceObjectPrimitive

        assert PlaceObjectPrimitive is not None

    def test_place_object_creates_manipulate_primitive(self):
        """RED: Creates MANIPULATE primitive for placing."""
        from agent_ros_bridge.ai.motion_primitives import PlaceObjectPrimitive

        place_pose = MagicMock()
        primitive = PlaceObjectPrimitive(object_id="obj_123", place_pose=place_pose)

        assert primitive.type == "MANIPULATE"
        assert primitive.primitive_id == "place_object"
        assert primitive.parameters.get("object_id") == "obj_123"


class TestGripperControlPrimitive:
    """Tests for GripperControlPrimitive."""

    def test_gripper_control_primitive_exists(self):
        """RED: GripperControlPrimitive should exist."""
        from agent_ros_bridge.ai.motion_primitives import GripperControlPrimitive

        assert GripperControlPrimitive is not None

    def test_gripper_open_creates_gripper_primitive(self):
        """RED: Creates GRIPPER primitive for open action."""
        from agent_ros_bridge.ai.motion_primitives import GripperControlPrimitive

        primitive = GripperControlPrimitive(action="open")

        assert primitive.type == "GRIPPER"
        assert primitive.primitive_id == "gripper_control"
        assert primitive.parameters.get("action") == "open"

    def test_gripper_close_creates_gripper_primitive(self):
        """RED: Creates GRIPPER primitive for close action."""
        from agent_ros_bridge.ai.motion_primitives import GripperControlPrimitive

        primitive = GripperControlPrimitive(action="close", force=5.0)

        assert primitive.type == "GRIPPER"
        assert primitive.parameters.get("action") == "close"
        assert primitive.parameters.get("force") == 5.0

    def test_gripper_validates_open_action(self):
        """RED: Should validate open action."""
        from agent_ros_bridge.ai.motion_primitives import GripperControlPrimitive

        primitive = GripperControlPrimitive(action="open")

        assert primitive.validate() is True

    def test_gripper_validates_close_action(self):
        """RED: Should validate close action."""
        from agent_ros_bridge.ai.motion_primitives import GripperControlPrimitive

        primitive = GripperControlPrimitive(action="close")

        assert primitive.validate() is True

    def test_gripper_rejects_invalid_action(self):
        """RED: Should reject invalid action."""
        from agent_ros_bridge.ai.motion_primitives import GripperControlPrimitive

        primitive = GripperControlPrimitive(action="invalid")

        assert primitive.validate() is False


class TestRotateInPlacePrimitive:
    """Tests for RotateInPlacePrimitive."""

    def test_rotate_in_place_primitive_exists(self):
        """RED: RotateInPlacePrimitive should exist."""
        from agent_ros_bridge.ai.motion_primitives import RotateInPlacePrimitive

        assert RotateInPlacePrimitive is not None

    def test_rotate_creates_navigate_primitive(self):
        """RED: Creates NAVIGATE primitive for rotation."""
        from agent_ros_bridge.ai.motion_primitives import RotateInPlacePrimitive

        primitive = RotateInPlacePrimitive(angle=1.57)

        assert primitive.type == "NAVIGATE"
        assert primitive.primitive_id == "rotate_in_place"
        assert primitive.parameters.get("angle") == 1.57

    def test_rotate_has_default_angular_velocity(self):
        """RED: Should have default angular_velocity."""
        from agent_ros_bridge.ai.motion_primitives import RotateInPlacePrimitive

        primitive = RotateInPlacePrimitive(angle=1.57)

        assert primitive.parameters.get("angular_velocity") == 0.5

    def test_rotate_validates_positive_angle(self):
        """RED: Should validate positive angle."""
        from agent_ros_bridge.ai.motion_primitives import RotateInPlacePrimitive

        primitive = RotateInPlacePrimitive(angle=1.57)

        assert primitive.validate() is True

    def test_rotate_validates_negative_angle(self):
        """RED: Should validate negative angle (rotate other direction)."""
        from agent_ros_bridge.ai.motion_primitives import RotateInPlacePrimitive

        primitive = RotateInPlacePrimitive(angle=-1.57)

        assert primitive.validate() is True


class TestPrimitiveParameterValidation:
    """Tests for primitive parameter validation."""

    def test_primitive_parameters_is_dict(self):
        """RED: parameters should be stored as dict."""
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose, max_velocity=0.8)

        assert isinstance(primitive.parameters, dict)
        assert "max_velocity" in primitive.parameters

    def test_primitive_expected_duration_is_float(self):
        """RED: expected_duration should be a float."""
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose)

        assert isinstance(primitive.expected_duration, float)

    def test_primitive_type_is_string(self):
        """RED: type should be a string."""
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose)

        assert isinstance(primitive.type, str)
        assert primitive.type == "NAVIGATE"


class TestMotionPrimitiveFactory:
    """Tests for MotionPrimitiveFactory."""

    def test_primitive_factory_exists(self):
        """RED: MotionPrimitiveFactory should exist."""
        from agent_ros_bridge.ai.motion_primitives import MotionPrimitiveFactory

        assert MotionPrimitiveFactory is not None

    def test_factory_creates_navigate_primitive(self):
        """RED: Factory should create NavigateToPosePrimitive."""
        from agent_ros_bridge.ai.motion_primitives import (
            MotionPrimitiveFactory,
            NavigateToPosePrimitive,
        )

        factory = MotionPrimitiveFactory()
        target_pose = MagicMock()

        primitive = factory.create_navigate_to_pose(target_pose=target_pose)

        assert isinstance(primitive, NavigateToPosePrimitive)

    def test_factory_creates_pick_primitive(self):
        """RED: Factory should create PickObjectPrimitive."""
        from agent_ros_bridge.ai.motion_primitives import (
            MotionPrimitiveFactory,
            PickObjectPrimitive,
        )

        factory = MotionPrimitiveFactory()
        grasp_pose = MagicMock()

        primitive = factory.create_pick_object(object_id="obj_123", grasp_pose=grasp_pose)

        assert isinstance(primitive, PickObjectPrimitive)

    def test_factory_creates_from_dict(self):
        """RED: Factory should create primitive from dict config."""
        from agent_ros_bridge.ai.motion_primitives import MotionPrimitiveFactory

        factory = MotionPrimitiveFactory()
        config = {
            "type": "NAVIGATE",
            "primitive_id": "navigate_to_pose",
            "parameters": {"max_velocity": 0.8},
        }

        primitive = factory.create_from_dict(config)

        assert primitive is not None
        assert primitive.type == "NAVIGATE"
