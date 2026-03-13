"""Tests for AI module initialization and exports."""


from agent_ros_bridge import ai


class TestAIModuleImports:
    """Test AI module imports and availability flags."""

    def test_has_ros_nodes_flag_exists(self):
        """_HAS_ROS_NODES flag is defined."""
        assert hasattr(ai, "_HAS_ROS_NODES")
        assert isinstance(ai._HAS_ROS_NODES, bool)

    def test_motion_planner_ros_available_flag_exists(self):
        """_MOTION_PLANNER_ROS_AVAILABLE flag is defined."""
        assert hasattr(ai, "_MOTION_PLANNER_ROS_AVAILABLE")
        assert isinstance(ai._MOTION_PLANNER_ROS_AVAILABLE, bool)

    def test_llm_available_flag_exists(self):
        """_LLM_AVAILABLE flag is defined."""
        assert hasattr(ai, "_LLM_AVAILABLE")
        assert isinstance(ai._LLM_AVAILABLE, bool)

    def test_context_available_flag_exists(self):
        """_CONTEXT_AVAILABLE flag is defined."""
        assert hasattr(ai, "_CONTEXT_AVAILABLE")
        assert isinstance(ai._CONTEXT_AVAILABLE, bool)

    def test_multilang_available_flag_exists(self):
        """_MULTILANG_AVAILABLE flag is defined."""
        assert hasattr(ai, "_MULTILANG_AVAILABLE")
        assert isinstance(ai._MULTILANG_AVAILABLE, bool)


class TestMotionPrimitivesExports:
    """Test motion primitive exports."""

    def test_motion_primitive_class_exported(self):
        """MotionPrimitive class is exported."""
        assert hasattr(ai, "MotionPrimitive")
        assert ai.MotionPrimitive is not None

    def test_navigate_to_pose_primitive_exported(self):
        """NavigateToPosePrimitive class is exported."""
        assert hasattr(ai, "NavigateToPosePrimitive")
        assert ai.NavigateToPosePrimitive is not None

    def test_pick_object_primitive_exported(self):
        """PickObjectPrimitive class is exported."""
        assert hasattr(ai, "PickObjectPrimitive")
        assert ai.PickObjectPrimitive is not None

    def test_place_object_primitive_exported(self):
        """PlaceObjectPrimitive class is exported."""
        assert hasattr(ai, "PlaceObjectPrimitive")
        assert ai.PlaceObjectPrimitive is not None

    def test_gripper_control_primitive_exported(self):
        """GripperControlPrimitive class is exported."""
        assert hasattr(ai, "GripperControlPrimitive")
        assert ai.GripperControlPrimitive is not None

    def test_rotate_in_place_primitive_exported(self):
        """RotateInPlacePrimitive class is exported."""
        assert hasattr(ai, "RotateInPlacePrimitive")
        assert ai.RotateInPlacePrimitive is not None

    def test_move_cartesian_primitive_exported(self):
        """MoveCartesianPrimitive class is exported."""
        assert hasattr(ai, "MoveCartesianPrimitive")
        assert ai.MoveCartesianPrimitive is not None

    def test_motion_primitive_factory_exported(self):
        """MotionPrimitiveFactory class is exported."""
        assert hasattr(ai, "MotionPrimitiveFactory")
        assert ai.MotionPrimitiveFactory is not None


class TestMotionPrimitiveFunctions:
    """Test motion primitive convenience functions."""

    def test_navigate_to_pose_function_exported(self):
        """navigate_to_pose function is exported."""
        assert hasattr(ai, "navigate_to_pose")
        assert callable(ai.navigate_to_pose)

    def test_pick_object_function_exported(self):
        """pick_object function is exported."""
        assert hasattr(ai, "pick_object")
        assert callable(ai.pick_object)

    def test_place_object_function_exported(self):
        """place_object function is exported."""
        assert hasattr(ai, "place_object")
        assert callable(ai.place_object)

    def test_gripper_control_function_exported(self):
        """gripper_control function is exported."""
        assert hasattr(ai, "gripper_control")
        assert callable(ai.gripper_control)

    def test_rotate_in_place_function_exported(self):
        """rotate_in_place function is exported."""
        assert hasattr(ai, "rotate_in_place")
        assert callable(ai.rotate_in_place)

    def test_move_cartesian_function_exported(self):
        """move_cartesian function is exported."""
        assert hasattr(ai, "move_cartesian")
        assert callable(ai.move_cartesian)


class TestMotionPlannerExports:
    """Test motion planner exports."""

    def test_motion_planner_node_exported(self):
        """MotionPlannerNode class is exported."""
        assert hasattr(ai, "MotionPlannerNode")
        assert ai.MotionPlannerNode is not None

    def test_motion_plan_exported(self):
        """MotionPlan class is exported."""
        assert hasattr(ai, "MotionPlan")
        assert ai.MotionPlan is not None

    def test_plan_motion_result_exported(self):
        """PlanMotionResult class is exported."""
        assert hasattr(ai, "PlanMotionResult")
        assert ai.PlanMotionResult is not None

    def test_safety_certificate_exported(self):
        """SafetyCertificate class is exported."""
        assert hasattr(ai, "SafetyCertificate")
        assert ai.SafetyCertificate is not None

    def test_safety_validator_exported(self):
        """SafetyValidator class is exported."""
        assert hasattr(ai, "SafetyValidator")
        assert ai.SafetyValidator is not None

    def test_nav2_integration_exported(self):
        """Nav2Integration class is exported."""
        assert hasattr(ai, "Nav2Integration")
        assert ai.Nav2Integration is not None

    def test_moveit2_integration_exported(self):
        """MoveIt2Integration class is exported."""
        assert hasattr(ai, "MoveIt2Integration")
        assert ai.MoveIt2Integration is not None

    def test_create_motion_planner_exported(self):
        """create_motion_planner function is exported."""
        assert hasattr(ai, "create_motion_planner")
        assert callable(ai.create_motion_planner)


class TestExecutionMonitorExports:
    """Test execution monitor exports."""

    def test_execution_monitor_node_exported(self):
        """ExecutionMonitorNode class is exported."""
        assert hasattr(ai, "ExecutionMonitorNode")
        assert ai.ExecutionMonitorNode is not None

    def test_execute_motion_result_exported(self):
        """ExecuteMotionResult class is exported."""
        assert hasattr(ai, "ExecuteMotionResult")
        assert ai.ExecuteMotionResult is not None

    def test_anomaly_exported(self):
        """Anomaly class is exported."""
        assert hasattr(ai, "Anomaly")
        assert ai.Anomaly is not None

    def test_anomaly_type_exported(self):
        """AnomalyType class is exported."""
        assert hasattr(ai, "AnomalyType")
        assert ai.AnomalyType is not None

    def test_recovery_result_exported(self):
        """RecoveryResult class is exported."""
        assert hasattr(ai, "RecoveryResult")
        assert ai.RecoveryResult is not None

    def test_recovery_handler_exported(self):
        """RecoveryHandler class is exported."""
        assert hasattr(ai, "RecoveryHandler")
        assert ai.RecoveryHandler is not None

    def test_telemetry_subscriber_exported(self):
        """TelemetrySubscriber class is exported."""
        assert hasattr(ai, "TelemetrySubscriber")
        assert ai.TelemetrySubscriber is not None

    def test_create_execution_monitor_exported(self):
        """create_execution_monitor function is exported."""
        assert hasattr(ai, "create_execution_monitor")
        assert callable(ai.create_execution_monitor)


class TestAllExports:
    """Test __all__ exports."""

    def test_all_list_exists(self):
        """__all__ list is defined."""
        assert hasattr(ai, "__all__")
        assert isinstance(ai.__all__, list)
        assert len(ai.__all__) > 0

    def test_all_exports_available(self):
        """All items in __all__ are exported."""
        for name in ai.__all__:
            assert hasattr(ai, name), f"{name} not exported"
