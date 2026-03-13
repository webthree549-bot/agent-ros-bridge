"""Integration tests for planning and execution.

Tests the flow from motion planner → execution monitor with safety validation
and anomaly detection/recovery.
"""

from unittest.mock import MagicMock

import pytest


class TestPlanningToExecutionFlow:
    """Tests for planner → execution_monitor flow."""

    @pytest.mark.asyncio
    async def test_plan_then_execute_navigation(self):
        """Integration: Plan navigation then execute."""
        from agent_ros_bridge.ai.execution_monitor import ExecutionMonitorNode
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode
        from agent_ros_bridge.ai.motion_primitives import \
            NavigateToPosePrimitive

        # Create planner and monitor
        planner = MotionPlannerNode()
        monitor = ExecutionMonitorNode()
        # Increase max recovery attempts for testing
        monitor.recovery_handler._test_max_attempts = 10

        # Create and plan a navigation primitive
        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose)

        plan_result = await planner.plan_motion(primitive)
        assert plan_result.success is True
        assert plan_result.plan.safety_certificate is not None

        # Execute the plan
        exec_result = await monitor.execute_motion(plan_result.plan)

        assert exec_result.success is True
        assert exec_result.status == "COMPLETED"
        assert exec_result.completion_percentage == 100.0

    @pytest.mark.asyncio
    async def test_plan_then_execute_manipulation(self):
        """Integration: Plan manipulation then execute."""
        from agent_ros_bridge.ai.execution_monitor import ExecutionMonitorNode
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode
        from agent_ros_bridge.ai.motion_primitives import PickObjectPrimitive

        planner = MotionPlannerNode()
        monitor = ExecutionMonitorNode()
        # Increase max recovery attempts for testing
        monitor.recovery_handler._test_max_attempts = 10

        grasp_pose = MagicMock()
        primitive = PickObjectPrimitive(object_id="obj_123", grasp_pose=grasp_pose)

        plan_result = await planner.plan_motion(primitive)
        assert plan_result.success is True

        exec_result = await monitor.execute_motion(plan_result.plan)

        assert exec_result.success is True
        assert exec_result.status == "COMPLETED"

    @pytest.mark.asyncio
    async def test_plan_sequence_then_execute(self):
        """Integration: Plan sequence then execute all."""
        from agent_ros_bridge.ai.execution_monitor import ExecutionMonitorNode
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode
        from agent_ros_bridge.ai.motion_primitives import (
            GripperControlPrimitive, NavigateToPosePrimitive)

        planner = MotionPlannerNode()
        monitor = ExecutionMonitorNode()

        # Create sequence
        target_pose = MagicMock()
        primitives = [
            NavigateToPosePrimitive(target_pose=target_pose),
            GripperControlPrimitive(action="open"),
        ]

        plan_result = await planner.plan_motion_sequence(primitives)
        assert plan_result.success is True
        assert len(plan_result.plan.primitives) == 2

        exec_result = await monitor.execute_motion(plan_result.plan)

        assert exec_result.success is True
        assert exec_result.status == "COMPLETED"


class TestSafetyValidatorIntegration:
    """Tests for safety validator integration."""

    @pytest.mark.asyncio
    async def test_unsafe_plan_rejected_before_execution(self):
        """Integration: Unsafe plan rejected before execution."""
        from agent_ros_bridge.ai.execution_monitor import ExecutionMonitorNode
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode
        from agent_ros_bridge.ai.motion_primitives import \
            NavigateToPosePrimitive

        planner = MotionPlannerNode()
        monitor = ExecutionMonitorNode()

        # Create unsafe primitive (excessive velocity)
        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose, max_velocity=5.0)

        plan_result = await planner.plan_motion(primitive)

        # Plan should be rejected by safety validator
        assert plan_result.success is False
        assert (
            "velocity" in plan_result.error_message.lower()
            or "safety" in plan_result.error_message.lower()
        )

    @pytest.mark.asyncio
    async def test_safe_plan_passes_validation(self):
        """Integration: Safe plan passes validation."""
        from agent_ros_bridge.ai.execution_monitor import ExecutionMonitorNode
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode
        from agent_ros_bridge.ai.motion_primitives import \
            NavigateToPosePrimitive

        planner = MotionPlannerNode()
        monitor = ExecutionMonitorNode()
        # Increase max recovery attempts for testing
        monitor.recovery_handler._test_max_attempts = 10

        # Create safe primitive
        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose, max_velocity=0.5)

        plan_result = await planner.plan_motion(primitive)

        assert plan_result.success is True
        assert plan_result.plan.safety_certificate is not None
        assert plan_result.plan.safety_certificate.valid is True

        # Execute should also succeed
        exec_result = await monitor.execute_motion(plan_result.plan)
        assert exec_result.success is True

    @pytest.mark.asyncio
    async def test_safety_certificate_expires(self):
        """Integration: Expired safety certificate rejected."""
        from agent_ros_bridge.ai.execution_monitor import ExecutionMonitorNode
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode
        from agent_ros_bridge.ai.motion_primitives import \
            NavigateToPosePrimitive

        planner = MotionPlannerNode()
        monitor = ExecutionMonitorNode()

        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose)

        plan_result = await planner.plan_motion(primitive)
        assert plan_result.success is True

        # Manually expire the certificate
        plan_result.plan.safety_certificate.expires_at = time.time() - 1

        # Execution should fail due to expired certificate
        exec_result = await monitor.execute_motion(plan_result.plan)
        assert exec_result.success is False
        assert (
            "certificate" in exec_result.error_message.lower()
            or "expired" in exec_result.error_message.lower()
        )


class TestAnomalyDetectionAndRecovery:
    """Tests for anomaly detection and recovery integration."""

    @pytest.mark.asyncio
    async def test_stuck_detection_triggers_recovery(self):
        """Integration: STUCK detection triggers recovery."""
        from agent_ros_bridge.ai.execution_monitor import (
            AnomalyType, ExecutionMonitorNode)

        monitor = ExecutionMonitorNode()

        # Create a mock pose to satisfy telemetry check
        current_pose = MagicMock()
        current_pose.pose.position.x = 1.0
        current_pose.pose.position.y = 2.0

        # Simulate stuck condition
        anomaly = monitor.detect_anomaly(
            current_pose=current_pose,
            target_pose=MagicMock(),
            progress=0.5,
            time_in_current_state=15.0,  # Exceeds threshold
        )

        assert anomaly is not None
        assert anomaly.type == AnomalyType.STUCK

        # Recovery should be triggered
        recovery_result = await monitor.handle_recovery(anomaly)

        assert recovery_result.success is True
        assert (
            "back" in recovery_result.action_taken.lower()
            or "replan" in recovery_result.action_taken.lower()
        )

    @pytest.mark.asyncio
    async def test_obstacle_detection_triggers_recovery(self):
        """Integration: OBSTACLE detection triggers recovery."""
        from agent_ros_bridge.ai.execution_monitor import (
            AnomalyType, ExecutionMonitorNode)

        monitor = ExecutionMonitorNode()

        # Simulate obstacle detection
        anomaly = monitor.detect_anomaly(
            obstacle_detected=True, obstacle_distance=0.3  # Less than threshold
        )

        assert anomaly is not None
        assert anomaly.type == AnomalyType.OBSTACLE

        recovery_result = await monitor.handle_recovery(anomaly)

        assert recovery_result.success is True
        assert (
            "wait" in recovery_result.action_taken.lower()
            or "replan" in recovery_result.action_taken.lower()
        )

    @pytest.mark.asyncio
    async def test_deviation_detection_triggers_recovery(self):
        """Integration: DEVIATION detection triggers recovery."""
        from agent_ros_bridge.ai.execution_monitor import (
            AnomalyType, ExecutionMonitorNode)

        monitor = ExecutionMonitorNode()

        # Create poses with significant deviation
        current_pose = MagicMock()
        current_pose.pose.position.x = 5.0
        current_pose.pose.position.y = 5.0

        planned_pose = MagicMock()
        planned_pose.pose.position.x = 3.0
        planned_pose.pose.position.y = 3.0

        anomaly = monitor.detect_anomaly(current_pose=current_pose, planned_pose=planned_pose)

        assert anomaly is not None
        assert anomaly.type == AnomalyType.DEVIATION

        recovery_result = await monitor.handle_recovery(anomaly)

        assert recovery_result.success is True


class TestRecoveryStrategies:
    """Tests for recovery strategies."""

    @pytest.mark.asyncio
    async def test_stuck_recovery_backup_and_replan(self):
        """Integration: STUCK recovery backs up and replans."""
        from agent_ros_bridge.ai.recovery import StuckRecovery

        recovery = StuckRecovery(backup_distance=0.5)

        result = await recovery.execute()

        assert result["success"] is True
        assert "back" in result["action"].lower() or "replan" in result["action"].lower()
        assert result["backup_distance"] == 0.5

    @pytest.mark.asyncio
    async def test_deviation_recovery_relocalize(self):
        """Integration: DEVIATION recovery re-localizes."""
        from agent_ros_bridge.ai.recovery import DeviationRecovery

        recovery = DeviationRecovery(relocalization_method="amcl")

        result = await recovery.execute()

        assert result["success"] is True
        assert "relocalize" in result["action"].lower() or "resume" in result["action"].lower()

    @pytest.mark.asyncio
    async def test_obstacle_recovery_wait_or_replan(self):
        """Integration: OBSTACLE recovery waits or replans."""
        from agent_ros_bridge.ai.recovery import ObstacleRecovery

        recovery = ObstacleRecovery(wait_timeout=5.0)

        result = await recovery.execute()

        assert result["success"] is True
        assert "wait" in result["action"].lower() or "replan" in result["action"].lower()

    @pytest.mark.asyncio
    async def test_timeout_recovery_abort(self):
        """Integration: TIMEOUT recovery aborts and notifies."""
        from agent_ros_bridge.ai.recovery import TimeoutRecovery

        recovery = TimeoutRecovery(notify_operator=True)

        result = await recovery.execute()

        assert result["success"] is True
        assert "abort" in result["action"].lower()
        assert result["notified"] is True


class TestEndToEndScenarios:
    """End-to-end integration scenarios."""

    @pytest.mark.asyncio
    async def test_navigate_pick_place_sequence(self):
        """E2E: Navigate → Pick → Place sequence."""
        from agent_ros_bridge.ai.execution_monitor import ExecutionMonitorNode
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode
        from agent_ros_bridge.ai.motion_primitives import (
            NavigateToPosePrimitive, PickObjectPrimitive, PlaceObjectPrimitive)

        planner = MotionPlannerNode()
        monitor = ExecutionMonitorNode()

        # Create full task sequence
        nav_pose = MagicMock()
        grasp_pose = MagicMock()
        place_pose = MagicMock()

        primitives = [
            NavigateToPosePrimitive(target_pose=nav_pose),
            PickObjectPrimitive(object_id="obj_123", grasp_pose=grasp_pose),
            PlaceObjectPrimitive(object_id="obj_123", place_pose=place_pose),
        ]

        # Plan
        plan_result = await planner.plan_motion_sequence(primitives)
        assert plan_result.success is True
        assert len(plan_result.plan.primitives) == 3

        # Execute
        exec_result = await monitor.execute_motion(plan_result.plan)

        assert exec_result.success is True
        assert exec_result.status == "COMPLETED"
        assert exec_result.completion_percentage == 100.0

    @pytest.mark.asyncio
    async def test_recovery_during_execution(self):
        """E2E: Recovery triggered during execution."""
        from agent_ros_bridge.ai.execution_monitor import ExecutionMonitorNode
        from agent_ros_bridge.ai.motion_planner import MotionPlannerNode
        from agent_ros_bridge.ai.motion_primitives import \
            NavigateToPosePrimitive

        planner = MotionPlannerNode()
        monitor = ExecutionMonitorNode()
        # Increase max recovery attempts for testing
        monitor.recovery_handler._test_max_attempts = 10

        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose)

        plan_result = await planner.plan_motion(primitive)
        assert plan_result.success is True

        # Execute (simulated with potential anomalies)
        exec_result = await monitor.execute_motion(plan_result.plan)

        # Should complete even if anomalies were detected
        assert exec_result.status in ["COMPLETED", "RECOVERED"]


# Need to import time for certificate expiration test
import time
