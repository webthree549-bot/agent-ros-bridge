"""Motion primitive library for Agent ROS Bridge.

This module provides motion primitives for robot navigation and manipulation.
All primitives include parameter validation and safety constraints.
"""

from dataclasses import dataclass, field
from typing import Any

# Mock PoseStamped for when ROS2 is not available
try:
    from geometry_msgs.msg import PoseStamped
except ImportError:

    class PoseStamped:
        """Mock PoseStamped for non-ROS environments."""

        def __init__(self):
            self.pose = MockPose()
            self.header = MockHeader()

    class MockPose:
        def __init__(self):
            self.position = MockPosition()
            self.orientation = MockOrientation()

    class MockPosition:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class MockOrientation:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0

    class MockHeader:
        def __init__(self):
            self.stamp = None
            self.frame_id = ""


@dataclass
class MotionPrimitive:
    """Base class for motion primitives.

    All motion primitives inherit from this class and implement
    their own validation logic.

    Attributes:
        type: Primitive type ("NAVIGATE", "MANIPULATE", "GRIPPER")
        primitive_id: Unique identifier for this primitive type
        target_pose: Optional target pose for the motion
        parameters: Dictionary of primitive-specific parameters
        expected_duration: Estimated time to complete the motion (seconds)
    """

    type: str  # "NAVIGATE", "MANIPULATE", "GRIPPER"
    primitive_id: str
    target_pose: Any | None = None
    parameters: dict[str, Any] = field(default_factory=dict)
    expected_duration: float = 0.0

    def validate(self) -> bool:
        """Validate primitive parameters.

        Override in subclasses to implement specific validation.

        Returns:
            True if parameters are valid, False otherwise
        """
        raise NotImplementedError("Subclasses must implement validate()")

    def to_dict(self) -> dict[str, Any]:
        """Convert primitive to dictionary representation.

        Returns:
            Dictionary containing primitive data
        """
        return {
            "type": self.type,
            "primitive_id": self.primitive_id,
            "parameters": self.parameters,
            "expected_duration": self.expected_duration,
            "target_pose": self._pose_to_dict(self.target_pose) if self.target_pose else None,
        }

    def _pose_to_dict(self, pose) -> dict[str, Any]:
        """Convert pose to dictionary.

        Args:
            pose: PoseStamped or mock pose object

        Returns:
            Dictionary representation of pose
        """
        if pose is None:
            return None

        return {
            "position": {
                "x": pose.pose.position.x,
                "y": pose.pose.position.y,
                "z": pose.pose.position.z,
            },
            "orientation": {
                "x": pose.pose.orientation.x,
                "y": pose.pose.orientation.y,
                "z": pose.pose.orientation.z,
                "w": pose.pose.orientation.w,
            },
            "frame_id": pose.header.frame_id if hasattr(pose, "header") else "",
        }


class NavigateToPosePrimitive(MotionPrimitive):
    """Navigate to a specific pose.

    This primitive commands the robot to navigate to a target position
    and orientation in the environment.

    Attributes:
        target_pose: Target pose to navigate to
        max_velocity: Maximum linear velocity (m/s)

    Example:
        >>> target = PoseStamped()
        >>> target.pose.position.x = 1.0
        >>> target.pose.position.y = 2.0
        >>> primitive = NavigateToPosePrimitive(target_pose=target, max_velocity=0.5)
        >>> primitive.validate()
        True
    """

    def __init__(self, target_pose: Any | None, max_velocity: float = 0.5):
        """Initialize navigate to pose primitive.

        Args:
            target_pose: Target pose to navigate to
            max_velocity: Maximum linear velocity (default: 0.5 m/s)
        """
        # Calculate duration before calling super().__init__
        expected_duration = self._estimate_duration_static(max_velocity)

        super().__init__(
            type="NAVIGATE",
            primitive_id="navigate_to_pose",
            target_pose=target_pose,
            parameters={"max_velocity": max_velocity},
            expected_duration=expected_duration,
        )

    def validate(self) -> bool:
        """Validate navigation parameters.

        Checks:
        - target_pose is not None
        - max_velocity > 0

        Returns:
            True if parameters are valid
        """
        if self.target_pose is None:
            return False
        return not self.parameters.get("max_velocity", 0) <= 0

    @staticmethod
    def _estimate_duration_static(max_velocity: float) -> float:
        """Estimate navigation duration (static version).

        Simplified estimation based on typical navigation distances.

        Args:
            max_velocity: Maximum linear velocity

        Returns:
            Estimated duration in seconds
        """
        # Simplified estimation - assumes ~5m travel at avg speed
        if max_velocity > 0:
            # Assume 5 meters at average speed (accounting for acceleration)
            avg_speed = max_velocity * 0.7
            return 5.0 / avg_speed if avg_speed > 0 else 10.0
        return 10.0  # seconds

    def _estimate_duration(self) -> float:
        """Estimate navigation duration.

        Simplified estimation based on typical navigation distances.

        Returns:
            Estimated duration in seconds
        """
        max_velocity = self.parameters.get("max_velocity", 0.5)
        return self._estimate_duration_static(max_velocity)


class PickObjectPrimitive(MotionPrimitive):
    """Pick up an object.

    This primitive commands the robot arm to pick up an object at
    a specified grasp pose.

    Attributes:
        object_id: Unique identifier of the object to pick
        target_pose: Grasp pose for the object
        gripper_force: Force to apply when grasping (N)

    Example:
        >>> grasp = PoseStamped()
        >>> primitive = PickObjectPrimitive(
        ...     object_id="obj_123",
        ...     grasp_pose=grasp,
        ...     gripper_force=10.0
        ... )
    """

    def __init__(self, object_id: str, grasp_pose: Any, gripper_force: float = 10.0):
        """Initialize pick object primitive.

        Args:
            object_id: Unique identifier of the object
            grasp_pose: Pose at which to grasp the object
            gripper_force: Force to apply when grasping (default: 10.0 N)
        """
        super().__init__(
            type="MANIPULATE",
            primitive_id="pick_object",
            target_pose=grasp_pose,
            parameters={"object_id": object_id, "gripper_force": gripper_force},
            expected_duration=15.0,
        )

    def validate(self) -> bool:
        """Validate pick parameters.

        Checks:
        - object_id is not empty or None
        - gripper_force > 0

        Returns:
            True if parameters are valid
        """
        object_id = self.parameters.get("object_id")
        if not object_id:
            return False
        return not self.parameters.get("gripper_force", 0) <= 0


class PlaceObjectPrimitive(MotionPrimitive):
    """Place an object at a specified location.

    This primitive commands the robot arm to place a held object
    at a target location.

    Attributes:
        object_id: Unique identifier of the object being placed
        place_pose: Target pose for placing the object
        release_height: Height offset for release (m)
    """

    def __init__(self, object_id: str, place_pose: Any, release_height: float = 0.02):
        """Initialize place object primitive.

        Args:
            object_id: Unique identifier of the object
            place_pose: Target pose for placing the object
            release_height: Height offset for release (default: 0.02 m)
        """
        super().__init__(
            type="MANIPULATE",
            primitive_id="place_object",
            target_pose=place_pose,
            parameters={"object_id": object_id, "release_height": release_height},
            expected_duration=12.0,
        )

    def validate(self) -> bool:
        """Validate place parameters.

        Returns:
            True if parameters are valid
        """
        object_id = self.parameters.get("object_id")
        if not object_id:
            return False
        return self.target_pose is not None


class GripperControlPrimitive(MotionPrimitive):
    """Control the gripper (open/close).

    This primitive controls the robot's gripper for opening
    and closing operations.

    Attributes:
        action: Either "open" or "close"
        force: Force to apply when closing (N), ignored for open
        position: Target position for the gripper (m)
    """

    VALID_ACTIONS = ["open", "close"]

    def __init__(self, action: str, force: float = 5.0, position: float | None = None):
        """Initialize gripper control primitive.

        Args:
            action: "open" or "close"
            force: Force for closing (default: 5.0 N)
            position: Target position in meters (optional)
        """
        super().__init__(
            type="GRIPPER",
            primitive_id="gripper_control",
            target_pose=None,
            parameters={
                "action": action.lower(),
                "force": force,
                "position": (
                    position if position is not None else (0.0 if action == "close" else 0.08)
                ),
            },
            expected_duration=3.0,
        )

    def validate(self) -> bool:
        """Validate gripper parameters.

        Checks:
        - action is either "open" or "close"

        Returns:
            True if parameters are valid
        """
        action = self.parameters.get("action")
        return action in self.VALID_ACTIONS


class RotateInPlacePrimitive(MotionPrimitive):
    """Rotate the robot in place.

    This primitive commands the robot to rotate around its z-axis
    without changing position.

    Attributes:
        angle: Rotation angle in radians (positive = counter-clockwise)
        angular_velocity: Maximum angular velocity (rad/s)
    """

    def __init__(self, angle: float, angular_velocity: float = 0.5):
        """Initialize rotate in place primitive.

        Args:
            angle: Rotation angle in radians
            angular_velocity: Maximum angular velocity (default: 0.5 rad/s)
        """
        # Calculate duration before calling super().__init__
        expected_duration = self._estimate_duration_static(angle, angular_velocity)

        super().__init__(
            type="NAVIGATE",
            primitive_id="rotate_in_place",
            target_pose=None,
            parameters={"angle": angle, "angular_velocity": angular_velocity},
            expected_duration=expected_duration,
        )

    @staticmethod
    def _estimate_duration_static(angle: float, angular_velocity: float) -> float:
        """Estimate rotation duration (static version).

        Args:
            angle: Rotation angle in radians
            angular_velocity: Maximum angular velocity

        Returns:
            Estimated duration in seconds
        """
        if angular_velocity > 0:
            # Account for acceleration/deceleration
            return (abs(angle) / angular_velocity) * 1.2 + 1.0
        return 5.0

    def validate(self) -> bool:
        """Validate rotation parameters.

        Checks:
        - angular_velocity > 0

        Returns:
            True if parameters are valid
        """
        return not self.parameters.get("angular_velocity", 0) <= 0

    def _estimate_duration(self) -> float:
        """Estimate rotation duration.

        Returns:
            Estimated duration in seconds
        """
        angle = self.parameters.get("angle", 0)
        angular_velocity = self.parameters.get("angular_velocity", 0.5)
        return self._estimate_duration_static(angle, angular_velocity)


class MoveCartesianPrimitive(MotionPrimitive):
    """Move the end effector in Cartesian space.

    This primitive commands linear motion of the end effector
    in Cartesian coordinates.

    Attributes:
        target_pose: Target end effector pose
        linear_velocity: Maximum linear velocity (m/s)
    """

    def __init__(self, target_pose: Any, linear_velocity: float = 0.1):
        """Initialize Cartesian motion primitive.

        Args:
            target_pose: Target end effector pose
            linear_velocity: Maximum linear velocity (default: 0.1 m/s)
        """
        super().__init__(
            type="MANIPULATE",
            primitive_id="move_cartesian",
            target_pose=target_pose,
            parameters={"linear_velocity": linear_velocity},
            expected_duration=10.0,
        )

    def validate(self) -> bool:
        """Validate Cartesian motion parameters.

        Returns:
            True if parameters are valid
        """
        if self.target_pose is None:
            return False
        return not self.parameters.get("linear_velocity", 0) <= 0


class MotionPrimitiveFactory:
    """Factory for creating motion primitives.

    Provides convenient methods for creating primitives and
    can instantiate primitives from configuration dictionaries.

    Example:
        >>> factory = MotionPrimitiveFactory()
        >>> nav = factory.create_navigate_to_pose(target_pose=pose)
        >>> pick = factory.create_pick_object(object_id="obj_1", grasp_pose=pose)
    """

    def __init__(self):
        """Initialize the primitive factory."""
        self._primitive_types = {
            "navigate_to_pose": NavigateToPosePrimitive,
            "pick_object": PickObjectPrimitive,
            "place_object": PlaceObjectPrimitive,
            "gripper_control": GripperControlPrimitive,
            "rotate_in_place": RotateInPlacePrimitive,
            "move_cartesian": MoveCartesianPrimitive,
        }

    def create_navigate_to_pose(
        self, target_pose: Any, max_velocity: float = 0.5
    ) -> NavigateToPosePrimitive:
        """Create a navigate to pose primitive.

        Args:
            target_pose: Target pose to navigate to
            max_velocity: Maximum linear velocity

        Returns:
            NavigateToPosePrimitive instance
        """
        return NavigateToPosePrimitive(target_pose=target_pose, max_velocity=max_velocity)

    def create_pick_object(
        self, object_id: str, grasp_pose: Any, gripper_force: float = 10.0
    ) -> PickObjectPrimitive:
        """Create a pick object primitive.

        Args:
            object_id: Unique identifier of the object
            grasp_pose: Grasp pose for the object
            gripper_force: Force to apply when grasping

        Returns:
            PickObjectPrimitive instance
        """
        return PickObjectPrimitive(
            object_id=object_id, grasp_pose=grasp_pose, gripper_force=gripper_force
        )

    def create_place_object(
        self, object_id: str, place_pose: Any, release_height: float = 0.02
    ) -> PlaceObjectPrimitive:
        """Create a place object primitive.

        Args:
            object_id: Unique identifier of the object
            place_pose: Target pose for placing
            release_height: Height offset for release

        Returns:
            PlaceObjectPrimitive instance
        """
        return PlaceObjectPrimitive(
            object_id=object_id, place_pose=place_pose, release_height=release_height
        )

    def create_gripper_control(
        self, action: str, force: float = 5.0, position: float | None = None
    ) -> GripperControlPrimitive:
        """Create a gripper control primitive.

        Args:
            action: "open" or "close"
            force: Force for closing
            position: Target position

        Returns:
            GripperControlPrimitive instance
        """
        return GripperControlPrimitive(action=action, force=force, position=position)

    def create_rotate_in_place(
        self, angle: float, angular_velocity: float = 0.5
    ) -> RotateInPlacePrimitive:
        """Create a rotate in place primitive.

        Args:
            angle: Rotation angle in radians
            angular_velocity: Maximum angular velocity

        Returns:
            RotateInPlacePrimitive instance
        """
        return RotateInPlacePrimitive(angle=angle, angular_velocity=angular_velocity)

    def create_move_cartesian(
        self, target_pose: Any, linear_velocity: float = 0.1
    ) -> MoveCartesianPrimitive:
        """Create a Cartesian motion primitive.

        Args:
            target_pose: Target end effector pose
            linear_velocity: Maximum linear velocity

        Returns:
            MoveCartesianPrimitive instance
        """
        return MoveCartesianPrimitive(target_pose=target_pose, linear_velocity=linear_velocity)

    def create_from_dict(self, config: dict[str, Any]) -> MotionPrimitive | None:
        """Create a primitive from a configuration dictionary.

        Args:
            config: Dictionary containing primitive configuration
                Must have keys: 'type' or 'primitive_id', and 'parameters'

        Returns:
            MotionPrimitive instance or None if creation fails

        Example:
            >>> config = {
            ...     "primitive_id": "navigate_to_pose",
            ...     "parameters": {"max_velocity": 0.8}
            ... }
            >>> primitive = factory.create_from_dict(config)
        """
        primitive_id = config.get("primitive_id") or config.get("type")
        if not primitive_id:
            return None

        # Normalize primitive_id
        primitive_id = primitive_id.lower().replace(" ", "_")

        params = config.get("parameters", {})

        # Handle special cases
        if primitive_id in ["navigate", "navigate_to_pose"]:
            # Need to reconstruct pose from config if available
            pose_data = config.get("target_pose")
            target_pose = self._dict_to_pose(pose_data) if pose_data else None
            return NavigateToPosePrimitive(
                target_pose=target_pose, max_velocity=params.get("max_velocity", 0.5)
            )

        elif primitive_id in ["pick", "pick_object"]:
            pose_data = config.get("target_pose") or config.get("grasp_pose")
            grasp_pose = self._dict_to_pose(pose_data) if pose_data else None
            return PickObjectPrimitive(
                object_id=params.get("object_id", ""),
                grasp_pose=grasp_pose,
                gripper_force=params.get("gripper_force", 10.0),
            )

        elif primitive_id in ["place", "place_object"]:
            pose_data = config.get("target_pose") or config.get("place_pose")
            place_pose = self._dict_to_pose(pose_data) if pose_data else None
            return PlaceObjectPrimitive(
                object_id=params.get("object_id", ""),
                place_pose=place_pose,
                release_height=params.get("release_height", 0.02),
            )

        elif primitive_id in ["gripper", "gripper_control"]:
            return GripperControlPrimitive(
                action=params.get("action", "open"),
                force=params.get("force", 5.0),
                position=params.get("position"),
            )

        elif primitive_id in ["rotate", "rotate_in_place"]:
            return RotateInPlacePrimitive(
                angle=params.get("angle", 0.0), angular_velocity=params.get("angular_velocity", 0.5)
            )

        elif primitive_id in ["cartesian", "move_cartesian"]:
            pose_data = config.get("target_pose")
            target_pose = self._dict_to_pose(pose_data) if pose_data else None
            return MoveCartesianPrimitive(
                target_pose=target_pose, linear_velocity=params.get("linear_velocity", 0.1)
            )

        return None

    def _dict_to_pose(self, pose_dict: dict[str, Any] | None) -> Any | None:
        """Convert dictionary to PoseStamped.

        Args:
            pose_dict: Dictionary containing pose data

        Returns:
            PoseStamped instance or None
        """
        if pose_dict is None:
            return None

        pose = PoseStamped()

        if "position" in pose_dict:
            pos = pose_dict["position"]
            pose.pose.position.x = pos.get("x", 0.0)
            pose.pose.position.y = pos.get("y", 0.0)
            pose.pose.position.z = pos.get("z", 0.0)

        if "orientation" in pose_dict:
            ori = pose_dict["orientation"]
            pose.pose.orientation.x = ori.get("x", 0.0)
            pose.pose.orientation.y = ori.get("y", 0.0)
            pose.pose.orientation.z = ori.get("z", 0.0)
            pose.pose.orientation.w = ori.get("w", 1.0)

        if "frame_id" in pose_dict:
            pose.header.frame_id = pose_dict["frame_id"]

        return pose

    def register_primitive_type(self, name: str, primitive_class: type):
        """Register a custom primitive type.

        Args:
            name: Name identifier for the primitive
            primitive_class: Class that inherits from MotionPrimitive
        """
        self._primitive_types[name] = primitive_class


# Convenience functions for quick primitive creation
def navigate_to_pose(target_pose: Any, max_velocity: float = 0.5) -> NavigateToPosePrimitive:
    """Create a navigate to pose primitive.

    Args:
        target_pose: Target pose to navigate to
        max_velocity: Maximum linear velocity

    Returns:
        NavigateToPosePrimitive instance
    """
    return NavigateToPosePrimitive(target_pose=target_pose, max_velocity=max_velocity)


def pick_object(
    object_id: str, grasp_pose: Any, gripper_force: float = 10.0
) -> PickObjectPrimitive:
    """Create a pick object primitive.

    Args:
        object_id: Unique identifier of the object
        grasp_pose: Grasp pose for the object
        gripper_force: Force to apply when grasping

    Returns:
        PickObjectPrimitive instance
    """
    return PickObjectPrimitive(
        object_id=object_id, grasp_pose=grasp_pose, gripper_force=gripper_force
    )


def place_object(
    object_id: str, place_pose: Any, release_height: float = 0.02
) -> PlaceObjectPrimitive:
    """Create a place object primitive.

    Args:
        object_id: Unique identifier of the object
        place_pose: Target pose for placing
        release_height: Height offset for release

    Returns:
        PlaceObjectPrimitive instance
    """
    return PlaceObjectPrimitive(
        object_id=object_id, place_pose=place_pose, release_height=release_height
    )


def gripper_control(
    action: str, force: float = 5.0, position: float | None = None
) -> GripperControlPrimitive:
    """Create a gripper control primitive.

    Args:
        action: "open" or "close"
        force: Force for closing
        position: Target position

    Returns:
        GripperControlPrimitive instance
    """
    return GripperControlPrimitive(action=action, force=force, position=position)


def rotate_in_place(angle: float, angular_velocity: float = 0.5) -> RotateInPlacePrimitive:
    """Create a rotate in place primitive.

    Args:
        angle: Rotation angle in radians
        angular_velocity: Maximum angular velocity

    Returns:
        RotateInPlacePrimitive instance
    """
    return RotateInPlacePrimitive(angle=angle, angular_velocity=angular_velocity)


def move_cartesian(target_pose: Any, linear_velocity: float = 0.1) -> MoveCartesianPrimitive:
    """Create a Cartesian motion primitive.

    Args:
        target_pose: Target end effector pose
        linear_velocity: Maximum linear velocity

    Returns:
        MoveCartesianPrimitive instance
    """
    return MoveCartesianPrimitive(target_pose=target_pose, linear_velocity=linear_velocity)
