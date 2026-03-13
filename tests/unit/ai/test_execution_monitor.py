"""Unit tests for execution monitor node.

TDD Approach: Write failing tests first, then implement to pass.
"""

from unittest.mock import MagicMock

import pytest


class TestExecutionMonitorNode:
    """Tests for ExecutionMonitorNode."""

    def test_execution_monitor_node_exists(self):
        """RED: ExecutionMonitorNode should exist."""
        from agent_ros_bridge.ai.execution_monitor import ExecutionMonitorNode

        assert ExecutionMonitorNode is not None

    def test_execution_monitor_node_initializes(self):
        """RED: ExecutionMonitorNode should initialize without errors."""
        from agent_ros_bridge.ai.execution_monitor import ExecutionMonitorNode

        node = ExecutionMonitorNode()
        assert node is not None
        assert node.node_name == "execution_monitor"

    def test_execute_motion_action_available(self):
        """RED: ExecuteMotion action server should be available."""
        from agent_ros_bridge.ai.execution_monitor import ExecutionMonitorNode

        node = ExecutionMonitorNode()

        # Check that action server is created
        assert hasattr(node, "execute_motion_server")
        assert node.execute_motion_server is not None

    def test_execution_monitor_has_telemetry_subscriber(self):
        """RED: ExecutionMonitor should subscribe to telemetry."""
        from agent_ros_bridge.ai.execution_monitor import ExecutionMonitorNode

        node = ExecutionMonitorNode()

        assert hasattr(node, "telemetry_subscriber")

    def test_execution_monitor_has_recovery_handler(self):
        """RED: ExecutionMonitor should have recovery handler."""
        from agent_ros_bridge.ai.execution_monitor import ExecutionMonitorNode

        node = ExecutionMonitorNode()

        assert hasattr(node, "recovery_handler")
        assert node.recovery_handler is not None


class TestExecuteMotion:
    """Tests for executing motion plans."""

    @pytest.mark.asyncio
    async def test_execute_navigate_motion(self):
        """RED: Should execute navigation motion."""
        from agent_ros_bridge.ai.execution_monitor import ExecutionMonitorNode
        from agent_ros_bridge.ai.motion_planner import MotionPlan, SafetyCertificate
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        node = ExecutionMonitorNode()

        # Create a motion plan with valid safety certificate
        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose)
        plan = MotionPlan(primitives=[primitive], safety_certificate=SafetyCertificate(valid=True))

        # Execute the motion
        result = await node.execute_motion(plan)

        assert result.success is True
        assert result.completion_percentage == 100.0

    @pytest.mark.asyncio
    async def test_execute_manipulation_motion(self):
        """RED: Should execute manipulation motion."""
        from agent_ros_bridge.ai.execution_monitor import ExecutionMonitorNode
        from agent_ros_bridge.ai.motion_planner import MotionPlan, SafetyCertificate
        from agent_ros_bridge.ai.motion_primitives import PickObjectPrimitive

        node = ExecutionMonitorNode()

        grasp_pose = MagicMock()
        primitive = PickObjectPrimitive(object_id="obj_123", grasp_pose=grasp_pose)
        plan = MotionPlan(primitives=[primitive], safety_certificate=SafetyCertificate(valid=True))

        result = await node.execute_motion(plan)

        assert result.success is True


class TestProgressMonitoring:
    """Tests for progress monitoring."""

    @pytest.mark.asyncio
    async def test_monitors_progress(self):
        """RED: Reports execution progress (0-100%)."""
        from agent_ros_bridge.ai.execution_monitor import ExecutionMonitorNode
        from agent_ros_bridge.ai.motion_planner import MotionPlan, SafetyCertificate
        from agent_ros_bridge.ai.motion_primitives import NavigateToPosePrimitive

        node = ExecutionMonitorNode()

        target_pose = MagicMock()
        primitive = NavigateToPosePrimitive(target_pose=target_pose)
        plan = MotionPlan(primitives=[primitive], safety_certificate=SafetyCertificate(valid=True))

        # Track progress updates
        progress_updates = []

        def on_progress(progress):
            progress_updates.append(progress)

        result = await node.execute_motion(plan, progress_callback=on_progress)

        assert result.success is True
        assert len(progress_updates) > 0
        assert progress_updates[-1] == 100.0
        assert 0.0 <= progress_updates[0] <= 100.0

    @pytest.mark.asyncio
    async def test_reports_completion(self):
        """RED: Reports successful completion."""
        from agent_ros_bridge.ai.execution_monitor import ExecutionMonitorNode
        from agent_ros_bridge.ai.motion_planner import MotionPlan, SafetyCertificate
        from agent_ros_bridge.ai.motion_primitives import GripperControlPrimitive

        node = ExecutionMonitorNode()

        primitive = GripperControlPrimitive(action="open")
        plan = MotionPlan(primitives=[primitive], safety_certificate=SafetyCertificate(valid=True))

        result = await node.execute_motion(plan)

        assert result.success is True
        assert result.status == "COMPLETED"
        assert result.completion_percentage == 100.0


class TestAnomalyDetection:
    """Tests for anomaly detection."""

    def test_detects_stuck_condition(self):
        """RED: Detects STUCK anomaly."""
        from agent_ros_bridge.ai.execution_monitor import AnomalyType, ExecutionMonitorNode

        node = ExecutionMonitorNode()

        # Simulate stuck condition (no progress for extended time, less than 90% complete)
        anomaly = node.detect_anomaly(
            current_pose=MagicMock(),
            target_pose=MagicMock(),
            progress=0.5,  # 50% progress
            time_in_current_state=15.0,  # More than STUCK_TIME_THRESHOLD (10s)
        )

        assert anomaly is not None
        assert anomaly.type == AnomalyType.STUCK

    def test_detects_deviation(self):
        """RED: Detects DEVIATION from planned path."""
        from agent_ros_bridge.ai.execution_monitor import AnomalyType, ExecutionMonitorNode

        node = ExecutionMonitorNode()

        # Simulate deviation from path
        current_pose = MagicMock()
        current_pose.pose.position.x = 5.0
        planned_pose = MagicMock()
        planned_pose.pose.position.x = 3.0

        anomaly = node.detect_anomaly(
            current_pose=current_pose,
            planned_pose=planned_pose,
            progress=0.5,
            time_in_current_state=1.0,
        )

        assert anomaly is not None
        assert anomaly.type == AnomalyType.DEVIATION

    def test_detects_obstacle(self):
        """RED: Detects OBSTACLE blocking path."""
        from agent_ros_bridge.ai.execution_monitor import AnomalyType, ExecutionMonitorNode

        node = ExecutionMonitorNode()

        # Simulate obstacle detection
        anomaly = node.detect_anomaly(
            current_pose=MagicMock(),
            target_pose=MagicMock(),
            progress=0.3,
            obstacle_detected=True,
            obstacle_distance=0.3,
        )

        assert anomaly is not None
        assert anomaly.type == AnomalyType.OBSTACLE

    def test_detects_timeout(self):
        """RED: Detects TIMEOUT condition."""
        from agent_ros_bridge.ai.execution_monitor import AnomalyType, ExecutionMonitorNode

        node = ExecutionMonitorNode()

        # Simulate timeout (exceeded expected duration by factor of 2)
        anomaly = node.detect_anomaly(
            current_pose=MagicMock(),
            target_pose=MagicMock(),
            progress=0.5,
            elapsed_time=20.0,
            expected_duration=5.0,
        )

        assert anomaly is not None
        assert anomaly.type == AnomalyType.TIMEOUT


class TestRecoveryStrategies:
    """Tests for recovery strategies."""

    @pytest.mark.asyncio
    async def test_triggers_recovery_on_stuck(self):
        """RED: Triggers recovery on STUCK anomaly."""
        from agent_ros_bridge.ai.execution_monitor import Anomaly, AnomalyType, ExecutionMonitorNode

        node = ExecutionMonitorNode()

        anomaly = Anomaly(
            type=AnomalyType.STUCK, description="Robot stuck for 10s", severity="HIGH"
        )

        recovery_result = await node.handle_recovery(anomaly)

        assert recovery_result.success is True
        assert recovery_result.action_taken is not None

    @pytest.mark.asyncio
    async def test_triggers_recovery_on_deviation(self):
        """RED: Triggers recovery on DEVIATION anomaly."""
        from agent_ros_bridge.ai.execution_monitor import Anomaly, AnomalyType, ExecutionMonitorNode

        node = ExecutionMonitorNode()

        anomaly = Anomaly(
            type=AnomalyType.DEVIATION, description="Path deviation detected", severity="MEDIUM"
        )

        recovery_result = await node.handle_recovery(anomaly)

        assert recovery_result.success is True

    @pytest.mark.asyncio
    async def test_triggers_recovery_on_obstacle(self):
        """RED: Triggers recovery on OBSTACLE anomaly."""
        from agent_ros_bridge.ai.execution_monitor import Anomaly, AnomalyType, ExecutionMonitorNode

        node = ExecutionMonitorNode()

        anomaly = Anomaly(
            type=AnomalyType.OBSTACLE, description="Obstacle at 0.3m", severity="HIGH"
        )

        recovery_result = await node.handle_recovery(anomaly)

        assert recovery_result.success is True

    @pytest.mark.asyncio
    async def test_triggers_recovery_on_timeout(self):
        """RED: Triggers recovery on TIMEOUT anomaly."""
        from agent_ros_bridge.ai.execution_monitor import Anomaly, AnomalyType, ExecutionMonitorNode

        node = ExecutionMonitorNode()

        anomaly = Anomaly(
            type=AnomalyType.TIMEOUT, description="Execution timeout", severity="CRITICAL"
        )

        recovery_result = await node.handle_recovery(anomaly)

        assert recovery_result.success is True


class TestExecutionResult:
    """Tests for ExecuteMotion action result."""

    def test_result_has_success_flag(self):
        """RED: Result should have success boolean."""
        from agent_ros_bridge.ai.execution_monitor import ExecuteMotionResult

        result = ExecuteMotionResult()

        assert hasattr(result, "success")
        assert isinstance(result.success, bool)

    def test_result_has_status(self):
        """RED: Result should have status."""
        from agent_ros_bridge.ai.execution_monitor import ExecuteMotionResult

        result = ExecuteMotionResult()

        assert hasattr(result, "status")
        assert isinstance(result.status, str)

    def test_result_has_completion_percentage(self):
        """RED: Result should have completion percentage."""
        from agent_ros_bridge.ai.execution_monitor import ExecuteMotionResult

        result = ExecuteMotionResult()

        assert hasattr(result, "completion_percentage")
        assert isinstance(result.completion_percentage, float)

    def test_result_has_error_message(self):
        """RED: Result should have error message."""
        from agent_ros_bridge.ai.execution_monitor import ExecuteMotionResult

        result = ExecuteMotionResult()

        assert hasattr(result, "error_message")
        assert isinstance(result.error_message, str)


class TestAnomalyType:
    """Tests for AnomalyType enum."""

    def test_anomaly_type_has_stuck(self):
        """RED: AnomalyType should have STUCK."""
        from agent_ros_bridge.ai.execution_monitor import AnomalyType

        assert hasattr(AnomalyType, "STUCK")

    def test_anomaly_type_has_deviation(self):
        """RED: AnomalyType should have DEVIATION."""
        from agent_ros_bridge.ai.execution_monitor import AnomalyType

        assert hasattr(AnomalyType, "DEVIATION")

    def test_anomaly_type_has_obstacle(self):
        """RED: AnomalyType should have OBSTACLE."""
        from agent_ros_bridge.ai.execution_monitor import AnomalyType

        assert hasattr(AnomalyType, "OBSTACLE")

    def test_anomaly_type_has_timeout(self):
        """RED: AnomalyType should have TIMEOUT."""
        from agent_ros_bridge.ai.execution_monitor import AnomalyType

        assert hasattr(AnomalyType, "TIMEOUT")


class TestAnomaly:
    """Tests for Anomaly dataclass."""

    def test_anomaly_has_type(self):
        """RED: Anomaly should have type."""
        from agent_ros_bridge.ai.execution_monitor import Anomaly, AnomalyType

        anomaly = Anomaly(type=AnomalyType.STUCK, description="Test")

        assert hasattr(anomaly, "type")
        assert anomaly.type == AnomalyType.STUCK

    def test_anomaly_has_description(self):
        """RED: Anomaly should have description."""
        from agent_ros_bridge.ai.execution_monitor import Anomaly, AnomalyType

        anomaly = Anomaly(type=AnomalyType.STUCK, description="Robot stuck")

        assert hasattr(anomaly, "description")
        assert anomaly.description == "Robot stuck"

    def test_anomaly_has_severity(self):
        """RED: Anomaly should have severity."""
        from agent_ros_bridge.ai.execution_monitor import Anomaly, AnomalyType

        anomaly = Anomaly(type=AnomalyType.STUCK, description="Test", severity="HIGH")

        assert hasattr(anomaly, "severity")
        assert anomaly.severity == "HIGH"


class TestRecoveryResult:
    """Tests for RecoveryResult."""

    def test_recovery_result_has_success(self):
        """RED: RecoveryResult should have success flag."""
        from agent_ros_bridge.ai.execution_monitor import RecoveryResult

        result = RecoveryResult()

        assert hasattr(result, "success")

    def test_recovery_result_has_action_taken(self):
        """RED: RecoveryResult should have action_taken."""
        from agent_ros_bridge.ai.execution_monitor import RecoveryResult

        result = RecoveryResult()

        assert hasattr(result, "action_taken")


class TestRecoveryHandler:
    """Tests for RecoveryHandler."""

    def test_recovery_handler_exists(self):
        """RED: RecoveryHandler should exist."""
        from agent_ros_bridge.ai.execution_monitor import RecoveryHandler

        assert RecoveryHandler is not None

    def test_recovery_handler_initializes(self):
        """RED: RecoveryHandler should initialize."""
        from agent_ros_bridge.ai.execution_monitor import RecoveryHandler

        handler = RecoveryHandler()
        assert handler is not None

    @pytest.mark.asyncio
    async def test_recovery_handler_handles_stuck(self):
        """RED: RecoveryHandler should handle STUCK."""
        from agent_ros_bridge.ai.execution_monitor import Anomaly, AnomalyType, RecoveryHandler

        handler = RecoveryHandler()

        anomaly = Anomaly(type=AnomalyType.STUCK, description="Stuck")
        result = await handler.handle(anomaly)

        assert result.success is True
        assert "back" in result.action_taken.lower() or "replan" in result.action_taken.lower()

    @pytest.mark.asyncio
    async def test_recovery_handler_handles_deviation(self):
        """RED: RecoveryHandler should handle DEVIATION."""
        from agent_ros_bridge.ai.execution_monitor import Anomaly, AnomalyType, RecoveryHandler

        handler = RecoveryHandler()

        anomaly = Anomaly(type=AnomalyType.DEVIATION, description="Deviation")
        result = await handler.handle(anomaly)

        assert result.success is True
        assert (
            "relocalize" in result.action_taken.lower() or "resume" in result.action_taken.lower()
        )

    @pytest.mark.asyncio
    async def test_recovery_handler_handles_obstacle(self):
        """RED: RecoveryHandler should handle OBSTACLE."""
        from agent_ros_bridge.ai.execution_monitor import Anomaly, AnomalyType, RecoveryHandler

        handler = RecoveryHandler()

        anomaly = Anomaly(type=AnomalyType.OBSTACLE, description="Obstacle")
        result = await handler.handle(anomaly)

        assert result.success is True
        assert "wait" in result.action_taken.lower() or "replan" in result.action_taken.lower()

    @pytest.mark.asyncio
    async def test_recovery_handler_handles_timeout(self):
        """RED: RecoveryHandler should handle TIMEOUT."""
        from agent_ros_bridge.ai.execution_monitor import Anomaly, AnomalyType, RecoveryHandler

        handler = RecoveryHandler()

        anomaly = Anomaly(type=AnomalyType.TIMEOUT, description="Timeout")
        result = await handler.handle(anomaly)

        assert result.success is True
        assert "abort" in result.action_taken.lower() or "notify" in result.action_taken.lower()


class TestTelemetrySubscriber:
    """Tests for telemetry subscription."""

    def test_telemetry_subscriber_exists(self):
        """RED: TelemetrySubscriber should exist."""
        from agent_ros_bridge.ai.execution_monitor import TelemetrySubscriber

        assert TelemetrySubscriber is not None

    def test_telemetry_subscriber_initializes(self):
        """RED: TelemetrySubscriber should initialize."""
        from agent_ros_bridge.ai.execution_monitor import TelemetrySubscriber

        sub = TelemetrySubscriber()
        assert sub is not None

    def test_telemetry_subscriber_receives_pose(self):
        """RED: TelemetrySubscriber should receive pose updates."""
        from agent_ros_bridge.ai.execution_monitor import TelemetrySubscriber

        sub = TelemetrySubscriber()

        pose = MagicMock()
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0

        sub.on_pose_received(pose)

        assert sub.current_pose == pose
