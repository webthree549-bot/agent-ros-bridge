"""Unit tests for motion planner node.

TDD Approach: Write failing tests first, then implement to pass.
"""

from unittest.mock import MagicMock

import pytest


class TestMotionPlannerNode:
    """Tests for MotionPlannerNode."""

    def test_motion_planner_node_exists(self):
        """RED: MotionPlannerNode should exist."""
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode

        assert MotionPlannerNode is not None

    def test_motion_planner_node_initializes(self):
        """RED: MotionPlannerNode should initialize without errors."""
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode

        node = MotionPlannerNode()
        assert node is not None
        assert node.node_name == "motion_planner"

    def test_plan_motion_action_available(self):
        """RED: PlanMotion action server should be available."""
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode

        node = MotionPlannerNode()

        # Check that action server is created
        assert hasattr(node, "plan_motion_server")
        assert node.plan_motion_server is not None

    def test_motion_planner_has_safety_validator(self):
        """RED: MotionPlanner should have safety validator."""
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode

        node = MotionPlannerNode()

        assert hasattr(node, "safety_validator")
        assert node.safety_validator is not None

    def test_motion_planner_has_nav2_integration(self):
        """RED: MotionPlanner should have Nav2 integration."""
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode

        node = MotionPlannerNode()

        assert hasattr(node, "nav2_integration")
        assert node.nav2_integration is not None

    def test_motion_planner_has_moveit_integration(self):
        """RED: MotionPlanner should have MoveIt2 integration."""
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode

        node = MotionPlannerNode()

        assert hasattr(node, "moveit_integration")
        assert node.moveit_integration is not None


class TestPlanNavigation:
    """Tests for planning navigation primitives."""

    @pytest.mark.asyncio
    async def test_plan_navigate_to_pose(self):
        """RED: Should plan navigation to target pose."""
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        node = MotionPlannerNode()

        # Create a navigation primitive
        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose)

        # Plan the motion
        result = await node.plan_motion(primitive)

        assert result.success is True
        assert result.plan is not None
        assert len(result.plan.primitives) > 0

    @pytest.mark.asyncio
    async def test_plan_navigate_includes_safety_certificate(self):
        """RED: All plans must include safety certificate."""
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        node = MotionPlannerNode()

        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose)

        result = await node.plan_motion(primitive)

        assert result.success is True
        assert result.plan.safety_certificate is not None
        assert result.plan.safety_certificate.valid is True

    @pytest.mark.asyncio
    async def test_plan_rejects_invalid_primitive(self):
        """RED: Should reject invalid primitive."""
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        node = MotionPlannerNode()

        # Create an invalid primitive (no target pose)
        primitive = NavigateToPosePrimitive(target_pose=None)

        result = await node.plan_motion(primitive)

        assert result.success is False
        assert (
            "validation" in result.error_message.lower()
            or "invalid" in result.error_message.lower()
        )


class TestPlanManipulation:
    """Tests for planning manipulation primitives."""

    @pytest.mark.asyncio
    async def test_plan_pick_object(self):
        """RED: Should plan pick object motion."""
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode
        from agent_ros_bridge.ai.motion_primitives import PickObjectPrimitive

        node = MotionPlannerNode()

        grasp_pose = MagicMock()
        primitive = PickObjectPrimitive(object_id="obj_123", grasp_pose=grasp_pose)

        result = await node.plan_motion(primitive)

        assert result.success is True
        assert result.plan is not None

    @pytest.mark.asyncio
    async def test_plan_place_object(self):
        """RED: Should plan place object motion."""
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode
        from agent_ros_bridge.ai.motion_primitives import PlaceObjectPrimitive

        node = MotionPlannerNode()

        place_pose = MagicMock()
        primitive = PlaceObjectPrimitive(object_id="obj_123", place_pose=place_pose)

        result = await node.plan_motion(primitive)

        assert result.success is True
        assert result.plan is not None


class TestSafetyValidation:
    """Tests for safety validation in planning."""

    @pytest.mark.asyncio
    async def test_rejects_unsafe_plan(self):
        """RED: Plan fails safety validation → rejected."""
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        node = MotionPlannerNode()

        # Create a primitive with unsafe velocity
        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(
            target_pose=target_pose, max_velocity=5.0
        )  # Exceeds 1.0 limit

        result = await node.plan_motion(primitive)

        assert result.success is False
        assert (
            "safety" in result.error_message.lower() or "velocity" in result.error_message.lower()
        )

    @pytest.mark.asyncio
    async def test_generates_safety_certificate_for_safe_plan(self):
        """RED: Safe plans get safety certificate."""
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        node = MotionPlannerNode()

        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose)

        result = await node.plan_motion(primitive)

        assert result.success is True
        assert result.plan.safety_certificate is not None
        assert hasattr(result.plan.safety_certificate, "issued_at")
        assert hasattr(result.plan.safety_certificate, "expires_at")


class TestPlanMultiplePrimitives:
    """Tests for planning sequences of primitives."""

    @pytest.mark.asyncio
    async def test_plans_multiple_primitives(self):
        """RED: Should plan sequence of primitives."""
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode
        from agent_ros_bridge.ai.motion_primitives import (
            NavigateToPosePrimitive,
            PickObjectPrimitive,
        )

        node = MotionPlannerNode()

        # Create multiple primitives
        target_pose = MagicMock()
        grasp_pose = MagicMock()

        primitives = [
            NavigateToPosePrimitive(target_pose=target_pose),
            PickObjectPrimitive(object_id="obj_123", grasp_pose=grasp_pose),
        ]

        result = await node.plan_motion_sequence(primitives)

        assert result.success is True
        assert result.plan is not None
        assert len(result.plan.primitives) == 2

    @pytest.mark.asyncio
    async def test_sequence_includes_safety_certificate(self):
        """RED: Sequence plans include safety certificate."""
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode
        from agent_ros_bridge.ai.motion_primitives import (
            GripperControlPrimitive,
            NavigateToPosePrimitive,
        )

        node = MotionPlannerNode()

        target_pose = MagicMock()
        primitives = [
            NavigateToPosePrimitive(target_pose=target_pose),
            GripperControlPrimitive(action="open"),
        ]

        result = await node.plan_motion_sequence(primitives)

        assert result.success is True
        assert result.plan.safety_certificate is not None


class TestMotionPlanResult:
    """Tests for MotionPlan result structure."""

    def test_motion_plan_has_primitives(self):
        """RED: MotionPlan should have primitives list."""
        from agent_ros_bridge.ai.motion_planner import MotionPlan

        plan = MotionPlan()

        assert hasattr(plan, "primitives")
        assert isinstance(plan.primitives, list)

    def test_motion_plan_has_safety_certificate(self):
        """RED: MotionPlan should have safety certificate."""
        from agent_ros_bridge.ai.motion_planner import MotionPlan

        plan = MotionPlan()

        assert hasattr(plan, "safety_certificate")

    def test_motion_plan_has_expected_duration(self):
        """RED: MotionPlan should have expected duration."""
        from agent_ros_bridge.ai.motion_planner import MotionPlan

        plan = MotionPlan()

        assert hasattr(plan, "expected_duration")
        assert isinstance(plan.expected_duration, float)


class TestPlanMotionResult:
    """Tests for PlanMotion action result."""

    def test_result_has_success_flag(self):
        """RED: Result should have success boolean."""
        from agent_ros_bridge.ai.motion_planner import PlanMotionResult

        result = PlanMotionResult()

        assert hasattr(result, "success")
        assert isinstance(result.success, bool)

    def test_result_has_plan(self):
        """RED: Result should have plan."""
        from agent_ros_bridge.ai.motion_planner import PlanMotionResult

        result = PlanMotionResult()

        assert hasattr(result, "plan")

    def test_result_has_error_message(self):
        """RED: Result should have error message."""
        from agent_ros_bridge.ai.motion_planner import PlanMotionResult

        result = PlanMotionResult()

        assert hasattr(result, "error_message")
        assert isinstance(result.error_message, str)


class TestSafetyCertificate:
    """Tests for SafetyCertificate."""

    def test_safety_certificate_has_valid_flag(self):
        """RED: Certificate should have valid flag."""
        from agent_ros_bridge.ai.motion_planner import SafetyCertificate

        cert = SafetyCertificate()

        assert hasattr(cert, "valid")
        assert isinstance(cert.valid, bool)

    def test_safety_certificate_has_issued_at(self):
        """RED: Certificate should have issued_at timestamp."""
        from agent_ros_bridge.ai.motion_planner import SafetyCertificate

        cert = SafetyCertificate()

        assert hasattr(cert, "issued_at")

    def test_safety_certificate_has_expires_at(self):
        """RED: Certificate should have expires_at timestamp."""
        from agent_ros_bridge.ai.motion_planner import SafetyCertificate

        cert = SafetyCertificate()

        assert hasattr(cert, "expires_at")

    def test_safety_certificate_has_constraints(self):
        """RED: Certificate should have constraints."""
        from agent_ros_bridge.ai.motion_planner import SafetyCertificate

        cert = SafetyCertificate()

        assert hasattr(cert, "constraints")


class TestNav2Integration:
    """Tests for Nav2 integration."""

    def test_nav2_integration_exists(self):
        """RED: Nav2Integration should exist."""
        from agent_ros_bridge.ai.motion_planner import Nav2Integration

        assert Nav2Integration is not None

    def test_nav2_integration_initializes(self):
        """RED: Nav2Integration should initialize."""
        from agent_ros_bridge.ai.motion_planner import Nav2Integration

        nav2 = Nav2Integration()
        assert nav2 is not None

    @pytest.mark.asyncio
    async def test_nav2_plan_navigation(self):
        """RED: Nav2Integration should plan navigation."""
        from agent_ros_bridge.ai.motion_planner import Nav2Integration

        nav2 = Nav2Integration()
        target_pose = MagicMock()

        plan = await nav2.plan_navigation(target_pose, max_velocity=0.5)

        assert plan is not None
        assert hasattr(plan, "trajectory")


class TestMoveIt2Integration:
    """Tests for MoveIt2 integration."""

    def test_moveit_integration_exists(self):
        """RED: MoveIt2Integration should exist."""
        from agent_ros_bridge.ai.motion_planner import MoveIt2Integration

        assert MoveIt2Integration is not None

    def test_moveit_integration_initializes(self):
        """RED: MoveIt2Integration should initialize."""
        from agent_ros_bridge.ai.motion_planner import MoveIt2Integration

        moveit = MoveIt2Integration()
        assert moveit is not None

    @pytest.mark.asyncio
    async def test_moveit_plan_manipulation(self):
        """RED: MoveIt2Integration should plan manipulation."""
        from agent_ros_bridge.ai.motion_planner import MoveIt2Integration

        moveit = MoveIt2Integration()
        target_pose = MagicMock()

        plan = await moveit.plan_manipulation(target_pose, planning_group="manipulator")

        assert plan is not None
        assert hasattr(plan, "trajectory")


class TestSafetyValidator:
    """Tests for SafetyValidator integration."""

    def test_safety_validator_exists(self):
        """RED: SafetyValidator should exist."""
        from agent_ros_bridge.ai.motion_planner import SafetyValidator

        assert SafetyValidator is not None

    def test_safety_validator_validates_plan(self):
        """RED: SafetyValidator should validate motion plans."""
        from agent_ros_bridge.ai.motion_planner import SafetyValidator
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        validator = SafetyValidator()

        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose)

        result = validator.validate_primitive(primitive)

        assert result.approved is True

    def test_safety_validator_rejects_invalid_velocity(self):
        """RED: SafetyValidator should reject excessive velocity."""
        from agent_ros_bridge.ai.motion_planner import SafetyValidator
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        validator = SafetyValidator()
        validator.max_linear_velocity = 1.0  # Set limit

        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose, max_velocity=2.0)

        result = validator.validate_primitive(primitive)

        assert result.approved is False
