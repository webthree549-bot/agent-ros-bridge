"""Motion planner node for Agent ROS Bridge.

This module provides motion planning capabilities with integration to
Nav2 for navigation and MoveIt2 for manipulation. All plans are validated
by the safety validator before being returned.
"""

import asyncio
import time
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any, Union
from enum import Enum

# Import motion primitives
from agent_ros_bridge.ai.motion_primitives import (
    MotionPrimitive,
    NavigateToPosePrimitive,
    PickObjectPrimitive,
    PlaceObjectPrimitive,
    GripperControlPrimitive,
    RotateInPlacePrimitive,
    MoveCartesianPrimitive,
)


@dataclass
class SafetyCertificate:
    """Safety certificate for a motion plan.
    
    Contains validation results and constraints that the plan must adhere to.
    
    Attributes:
        valid: Whether the certificate is valid
        issued_at: Timestamp when certificate was issued
        expires_at: Timestamp when certificate expires
        constraints: Dictionary of safety constraints
        plan_hash: Hash of the validated plan for integrity
    """
    valid: bool = True
    issued_at: float = field(default_factory=time.time)
    expires_at: float = field(default_factory=lambda: time.time() + 30.0)
    constraints: Dict[str, Any] = field(default_factory=dict)
    plan_hash: str = ""
    
    def is_expired(self) -> bool:
        """Check if the certificate has expired.
        
        Returns:
            True if certificate is expired
        """
        return time.time() > self.expires_at
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert certificate to dictionary.
        
        Returns:
            Dictionary representation
        """
        return {
            "valid": self.valid,
            "issued_at": self.issued_at,
            "expires_at": self.expires_at,
            "constraints": self.constraints,
            "plan_hash": self.plan_hash
        }


@dataclass
class ValidationResult:
    """Result of safety validation.
    
    Attributes:
        approved: Whether the plan is approved
        rejection_reason: Reason for rejection if not approved
        certificate: Safety certificate if approved
    """
    approved: bool
    rejection_reason: str = ""
    certificate: Optional[SafetyCertificate] = None


@dataclass
class MotionPlan:
    """A motion plan containing primitives and safety certificate.
    
    Attributes:
        primitives: List of motion primitives in the plan
        safety_certificate: Safety certificate for this plan
        expected_duration: Expected duration in seconds
        planning_time: Time taken to generate the plan
    """
    primitives: List[MotionPrimitive] = field(default_factory=list)
    safety_certificate: Optional[SafetyCertificate] = None
    expected_duration: float = 0.0
    planning_time: float = 0.0
    
    def add_primitive(self, primitive: MotionPrimitive):
        """Add a primitive to the plan.
        
        Args:
            primitive: Motion primitive to add
        """
        self.primitives.append(primitive)
        self.expected_duration += primitive.expected_duration
    
    def is_valid(self) -> bool:
        """Check if the plan is valid (has certificate and not expired).
        
        Returns:
            True if plan is valid
        """
        if self.safety_certificate is None:
            return False
        return self.safety_certificate.valid and not self.safety_certificate.is_expired()


@dataclass
class PlanMotionResult:
    """Result of plan motion action.
    
    Attributes:
        success: Whether planning was successful
        plan: The generated motion plan (if successful)
        error_message: Error message (if failed)
    """
    success: bool = False
    plan: Optional[MotionPlan] = None
    error_message: str = ""


@dataclass
class Trajectory:
    """Robot trajectory for execution.
    
    Attributes:
        joint_names: Names of joints in the trajectory
        points: List of trajectory points
        timestamps: Timestamps for each point
    """
    joint_names: List[str] = field(default_factory=list)
    points: List[Dict[str, Any]] = field(default_factory=list)
    timestamps: List[float] = field(default_factory=list)


@dataclass
class NavigationPlan:
    """Navigation plan from Nav2.
    
    Attributes:
        trajectory: Planned trajectory
        path_length: Length of the path in meters
        planning_time: Time taken to plan
    """
    trajectory: Optional[Trajectory] = None
    path_length: float = 0.0
    planning_time: float = 0.0


@dataclass
class ManipulationPlan:
    """Manipulation plan from MoveIt2.
    
    Attributes:
        trajectory: Planned trajectory
        planning_group: Planning group used
        planning_time: Time taken to plan
    """
    trajectory: Optional[Trajectory] = None
    planning_group: str = ""
    planning_time: float = 0.0


class SafetyValidator:
    """Safety validator for motion primitives and plans.
    
    Validates primitives against safety constraints before execution.
    
    Attributes:
        max_linear_velocity: Maximum allowed linear velocity (m/s)
        max_angular_velocity: Maximum allowed angular velocity (rad/s)
        max_gripper_force: Maximum allowed gripper force (N)
    """
    
    def __init__(self):
        """Initialize the safety validator with default limits."""
        self.max_linear_velocity = 1.0  # m/s
        self.max_angular_velocity = 1.0  # rad/s
        self.max_gripper_force = 50.0  # N
        self.validation_count = 0
        self.rejection_count = 0
    
    def validate_primitive(self, primitive: MotionPrimitive) -> ValidationResult:
        """Validate a single motion primitive.
        
        Args:
            primitive: Motion primitive to validate
            
        Returns:
            ValidationResult with approval status
        """
        self.validation_count += 1
        
        # First check if primitive is valid itself
        if not primitive.validate():
            self.rejection_count += 1
            return ValidationResult(
                approved=False,
                rejection_reason="Primitive failed internal validation"
            )
        
        # Check velocity limits based on primitive type
        if primitive.type == "NAVIGATE":
            max_vel = primitive.parameters.get("max_velocity", 0.5)
            if max_vel > self.max_linear_velocity:
                self.rejection_count += 1
                return ValidationResult(
                    approved=False,
                    rejection_reason=f"Linear velocity {max_vel} exceeds limit {self.max_linear_velocity}"
                )
            
            angular_vel = primitive.parameters.get("angular_velocity", 0.5)
            if angular_vel > self.max_angular_velocity:
                self.rejection_count += 1
                return ValidationResult(
                    approved=False,
                    rejection_reason=f"Angular velocity {angular_vel} exceeds limit {self.max_angular_velocity}"
                )
        
        elif primitive.type == "MANIPULATE":
            # Check manipulation-specific constraints
            linear_vel = primitive.parameters.get("linear_velocity", 0.1)
            if linear_vel > self.max_linear_velocity:
                self.rejection_count += 1
                return ValidationResult(
                    approved=False,
                    rejection_reason=f"Cartesian velocity {linear_vel} exceeds limit"
                )
        
        elif primitive.type == "GRIPPER":
            force = primitive.parameters.get("force", 5.0)
            if force > self.max_gripper_force:
                self.rejection_count += 1
                return ValidationResult(
                    approved=False,
                    rejection_reason=f"Gripper force {force} exceeds limit {self.max_gripper_force}"
                )
        
        # All checks passed - generate certificate
        certificate = SafetyCertificate(
            valid=True,
            constraints={
                "max_linear_velocity": self.max_linear_velocity,
                "max_angular_velocity": self.max_angular_velocity,
                "max_gripper_force": self.max_gripper_force
            },
            plan_hash=self._compute_plan_hash(primitive)
        )
        
        return ValidationResult(approved=True, certificate=certificate)
    
    def validate_plan(self, plan: MotionPlan) -> ValidationResult:
        """Validate an entire motion plan.
        
        Args:
            plan: Motion plan to validate
            
        Returns:
            ValidationResult with approval status
        """
        if not plan.primitives:
            return ValidationResult(
                approved=False,
                rejection_reason="Plan contains no primitives"
            )
        
        # Validate each primitive
        for i, primitive in enumerate(plan.primitives):
            result = self.validate_primitive(primitive)
            if not result.approved:
                return ValidationResult(
                    approved=False,
                    rejection_reason=f"Primitive {i} failed: {result.rejection_reason}"
                )
        
        # Generate plan-level certificate
        certificate = SafetyCertificate(
            valid=True,
            constraints={
                "max_linear_velocity": self.max_linear_velocity,
                "max_angular_velocity": self.max_angular_velocity,
                "max_gripper_force": self.max_gripper_force,
                "primitive_count": len(plan.primitives),
                "expected_duration": plan.expected_duration
            },
            plan_hash=self._compute_plan_hash(plan)
        )
        
        return ValidationResult(approved=True, certificate=certificate)
    
    def _compute_plan_hash(self, obj: Union[MotionPrimitive, MotionPlan]) -> str:
        """Compute a hash for the plan/primitive.
        
        Args:
            obj: Object to hash
            
        Returns:
            Hash string
        """
        import hashlib
        
        if isinstance(obj, MotionPrimitive):
            data = f"{obj.type}:{obj.primitive_id}:{obj.parameters}"
        elif isinstance(obj, MotionPlan):
            data = f"plan:{len(obj.primitives)}:{obj.expected_duration}"
        else:
            data = str(obj)
        
        return hashlib.md5(data.encode()).hexdigest()


class Nav2Integration:
    """Integration with Nav2 for navigation planning.
    
    Provides interface to Nav2's navigation stack for planning
    and executing navigation primitives.
    """
    
    def __init__(self):
        """Initialize Nav2 integration."""
        self.action_client = None
        self.planner_service = None
        self.default_max_velocity = 0.5
        self.planning_timeout = 5.0
    
    async def plan_navigation(self, target_pose: Any, max_velocity: float = 0.5) -> NavigationPlan:
        """Plan navigation to target pose.
        
        Args:
            target_pose: Target pose to navigate to
            max_velocity: Maximum linear velocity
            
        Returns:
            NavigationPlan with trajectory
        """
        # Simulate planning (in real implementation, call Nav2 services)
        await asyncio.sleep(0.01)  # Simulate planning delay
        
        # Create a simple trajectory
        trajectory = Trajectory(
            joint_names=["base_x", "base_y", "base_theta"],
            points=[
                {"positions": [0.0, 0.0, 0.0]},
                {"positions": [target_pose.pose.position.x if hasattr(target_pose, 'pose') else 1.0,
                              target_pose.pose.position.y if hasattr(target_pose, 'pose') else 0.0,
                              0.0]}
            ],
            timestamps=[0.0, 5.0 / max_velocity]
        )
        
        return NavigationPlan(
            trajectory=trajectory,
            path_length=5.0,  # Simplified
            planning_time=0.01
        )
    
    async def is_navigator_ready(self) -> bool:
        """Check if Nav2 navigator is ready.
        
        Returns:
            True if navigator is ready
        """
        # In real implementation, check Nav2 status
        return True


class MoveIt2Integration:
    """Integration with MoveIt2 for manipulation planning.
    
    Provides interface to MoveIt2's motion planning capabilities
    for arm manipulation primitives.
    """
    
    def __init__(self):
        """Initialize MoveIt2 integration."""
        self.move_group = None
        self.planning_scene = None
        self.default_planning_group = "manipulator"
        self.planning_timeout = 5.0
    
    async def plan_manipulation(self, target_pose: Any, planning_group: str = "manipulator") -> ManipulationPlan:
        """Plan manipulation motion to target pose.
        
        Args:
            target_pose: Target end effector pose
            planning_group: Planning group to use
            
        Returns:
            ManipulationPlan with trajectory
        """
        # Simulate planning (in real implementation, call MoveIt2 services)
        await asyncio.sleep(0.01)  # Simulate planning delay
        
        # Create a simple trajectory
        trajectory = Trajectory(
            joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            points=[
                {"positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                {"positions": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}
            ],
            timestamps=[0.0, 10.0]
        )
        
        return ManipulationPlan(
            trajectory=trajectory,
            planning_group=planning_group,
            planning_time=0.01
        )
    
    async def plan_pick(self, object_id: str, grasp_pose: Any) -> ManipulationPlan:
        """Plan pick motion.
        
        Args:
            object_id: Object to pick
            grasp_pose: Grasp pose
            
        Returns:
            ManipulationPlan with trajectory
        """
        await asyncio.sleep(0.01)
        
        trajectory = Trajectory(
            joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            points=[
                {"positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                {"positions": [0.3, 0.3, 0.3, 0.3, 0.3, 0.3]},
                {"positions": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}
            ],
            timestamps=[0.0, 7.0, 15.0]
        )
        
        return ManipulationPlan(
            trajectory=trajectory,
            planning_group="manipulator",
            planning_time=0.01
        )
    
    async def plan_place(self, object_id: str, place_pose: Any) -> ManipulationPlan:
        """Plan place motion.
        
        Args:
            object_id: Object to place
            place_pose: Place pose
            
        Returns:
            ManipulationPlan with trajectory
        """
        await asyncio.sleep(0.01)
        
        trajectory = Trajectory(
            joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            points=[
                {"positions": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]},
                {"positions": [0.3, 0.3, 0.3, 0.3, 0.3, 0.3]},
                {"positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
            ],
            timestamps=[0.0, 6.0, 12.0]
        )
        
        return ManipulationPlan(
            trajectory=trajectory,
            planning_group="manipulator",
            planning_time=0.01
        )
    
    async def is_move_group_ready(self) -> bool:
        """Check if MoveGroup is ready.
        
        Returns:
            True if MoveGroup is ready
        """
        # In real implementation, check MoveIt2 status
        return True


class MotionPlannerNode:
    """Motion planner node for Agent ROS Bridge.
    
    Provides motion planning services with integration to Nav2 and MoveIt2.
    All plans are validated by the safety validator before being returned.
    
    Attributes:
        node_name: Name of the ROS node
        safety_validator: Safety validator instance
        nav2_integration: Nav2 integration instance
        moveit_integration: MoveIt2 integration instance
        plan_motion_server: Action server for PlanMotion
    """
    
    def __init__(self, node_name: str = "motion_planner"):
        """Initialize the motion planner node.
        
        Args:
            node_name: Name for the ROS node
        """
        self.node_name = node_name
        
        # Initialize components
        self.safety_validator = SafetyValidator()
        self.nav2_integration = Nav2Integration()
        self.moveit_integration = MoveIt2Integration()
        
        # Action server placeholder (would be ROS2 action server in real impl)
        self.plan_motion_server = MagicMock()
        self.plan_motion_server.register_goal_callback = MagicMock()
    
    async def plan_motion(self, primitive: MotionPrimitive) -> PlanMotionResult:
        """Plan motion for a single primitive.
        
        Args:
            primitive: Motion primitive to plan for
            
        Returns:
            PlanMotionResult with plan or error
        """
        start_time = time.time()
        
        # Validate the primitive first
        validation_result = self.safety_validator.validate_primitive(primitive)
        if not validation_result.approved:
            return PlanMotionResult(
                success=False,
                error_message=f"Safety validation failed: {validation_result.rejection_reason}"
            )
        
        # Create motion plan
        plan = MotionPlan(
            primitives=[primitive],
            planning_time=0.0
        )
        
        # Plan based on primitive type
        try:
            if primitive.type == "NAVIGATE":
                if primitive.primitive_id == "rotate_in_place":
                    # Rotation doesn't need Nav2 planning
                    pass
                else:
                    # Plan navigation
                    nav_plan = await self.nav2_integration.plan_navigation(
                        primitive.target_pose,
                        primitive.parameters.get("max_velocity", 0.5)
                    )
                    plan.expected_duration = nav_plan.planning_time
            
            elif primitive.type == "MANIPULATE":
                if primitive.primitive_id == "pick_object":
                    # Plan pick motion
                    manip_plan = await self.moveit_integration.plan_pick(
                        primitive.parameters.get("object_id", ""),
                        primitive.target_pose
                    )
                    plan.expected_duration = manip_plan.planning_time
                
                elif primitive.primitive_id == "place_object":
                    # Plan place motion
                    manip_plan = await self.moveit_integration.plan_place(
                        primitive.parameters.get("object_id", ""),
                        primitive.target_pose
                    )
                    plan.expected_duration = manip_plan.planning_time
                
                else:
                    # Plan general manipulation
                    manip_plan = await self.moveit_integration.plan_manipulation(
                        primitive.target_pose,
                        primitive.parameters.get("planning_group", "manipulator")
                    )
                    plan.expected_duration = manip_plan.planning_time
            
            elif primitive.type == "GRIPPER":
                # Gripper actions don't need complex planning
                plan.expected_duration = primitive.expected_duration
        
        except Exception as e:
            return PlanMotionResult(
                success=False,
                error_message=f"Planning failed: {str(e)}"
            )
        
        # Set planning time
        plan.planning_time = time.time() - start_time
        
        # Attach safety certificate
        plan.safety_certificate = validation_result.certificate
        
        return PlanMotionResult(success=True, plan=plan)
    
    async def plan_motion_sequence(self, primitives: List[MotionPrimitive]) -> PlanMotionResult:
        """Plan motion for a sequence of primitives.
        
        Args:
            primitives: List of motion primitives
            
        Returns:
            PlanMotionResult with combined plan or error
        """
        start_time = time.time()
        
        if not primitives:
            return PlanMotionResult(
                success=False,
                error_message="No primitives provided"
            )
        
        # Create combined plan
        combined_plan = MotionPlan(
            primitives=[],
            planning_time=0.0
        )
        
        # Plan each primitive
        for i, primitive in enumerate(primitives):
            result = await self.plan_motion(primitive)
            
            if not result.success:
                return PlanMotionResult(
                    success=False,
                    error_message=f"Primitive {i} failed: {result.error_message}"
                )
            
            # Add primitive to combined plan
            combined_plan.add_primitive(primitive)
        
        # Validate the combined plan
        validation_result = self.safety_validator.validate_plan(combined_plan)
        if not validation_result.approved:
            return PlanMotionResult(
                success=False,
                error_message=f"Combined plan validation failed: {validation_result.rejection_reason}"
            )
        
        # Set planning time and certificate
        combined_plan.planning_time = time.time() - start_time
        combined_plan.safety_certificate = validation_result.certificate
        
        return PlanMotionResult(success=True, plan=combined_plan)
    
    async def start(self):
        """Start the motion planner node."""
        # In real implementation, start ROS2 node and action servers
        pass
    
    async def stop(self):
        """Stop the motion planner node."""
        # In real implementation, shutdown ROS2 node
        pass


# Mock for MagicMock (used in testing)
class MagicMock:
    """Simple mock class for testing without unittest.mock."""
    
    def __init__(self, **kwargs):
        """Initialize mock with attributes."""
        for key, value in kwargs.items():
            setattr(self, key, value)
        self._callbacks = {}
    
    def __call__(self, *args, **kwargs):
        """Make mock callable."""
        return MagicMock()
    
    def register_goal_callback(self, callback):
        """Register a goal callback."""
        self._callbacks['goal'] = callback


# Convenience function for creating planner
def create_motion_planner(node_name: str = "motion_planner") -> MotionPlannerNode:
    """Create a motion planner node.
    
    Args:
        node_name: Name for the ROS node
        
    Returns:
        MotionPlannerNode instance
    """
    return MotionPlannerNode(node_name=node_name)
