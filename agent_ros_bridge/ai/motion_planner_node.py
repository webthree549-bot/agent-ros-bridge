#!/usr/bin/env python3
"""
Motion Planner ROS2 Node
Agent ROS Bridge v0.6.1 - Week 2 Implementation

Exposes motion planning as ROS2 actions:
- /ai/plan_motion (PlanMotion)
- /ai/execute_motion (ExecuteMotion)

Integrates with:
- Safety Validator (/safety/validate_trajectory)
- Nav2 (navigation)
- MoveIt2 (manipulation)

Target: <100ms planning response time
"""

import asyncio
import time
from typing import Any

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

# Import core motion planner
from .motion_planner import MotionPlan, MotionPlannerNode, SafetyCertificate

# Try to import ROS message types
try:
    from agent_ros_bridge_msgs.action import ExecuteMotion, PlanMotion
    from agent_ros_bridge_msgs.msg import MotionPlan as MotionPlanMsg
    from agent_ros_bridge_msgs.msg import SafetyLimits
    from agent_ros_bridge_msgs.srv import ValidateTrajectory

    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False
    PlanMotion = None
    ExecuteMotion = None
    ValidateTrajectory = None
    SafetyLimits = None
    MotionPlanMsg = None


class MotionPlannerROSNode(Node):
    """
    Motion Planner ROS2 Node

    Wraps MotionPlannerNode with ROS2 action interfaces.
    Provides motion planning and execution with safety validation.
    """

    def __init__(self):
        super().__init__("motion_planner")

        if not MSGS_AVAILABLE:
            self.get_logger().error(
                "agent_ros_bridge_msgs not available. Motion planning disabled."
            )
            return

        # Core motion planner
        self._planner = MotionPlannerNode()

        # Callback group for concurrent handling
        self._callback_group = ReentrantCallbackGroup()

        # Action servers
        self._plan_action = ActionServer(
            self,
            PlanMotion,
            "/ai/plan_motion",
            self._handle_plan_motion,
            callback_group=self._callback_group,
        )

        self._execute_action = ActionServer(
            self,
            ExecuteMotion,
            "/ai/execute_motion",
            self._handle_execute_motion,
            callback_group=self._callback_group,
        )

        # Safety validator client
        self._safety_client = self.create_client(
            ValidateTrajectory, "/safety/validate_trajectory", callback_group=self._callback_group
        )

        # Wait for safety validator
        self.get_logger().info("Waiting for safety validator...")
        if not self._safety_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Safety validator not available. Plans will not be validated.")

        self.get_logger().info("Motion Planner Node initialized")
        self.get_logger().info("Actions:")
        self.get_logger().info("  - /ai/plan_motion")
        self.get_logger().info("  - /ai/execute_motion")

    async def _handle_plan_motion(self, goal_handle):
        """
        Handle PlanMotion action request

        Target: <100ms planning response time
        """
        start_time = time.time()
        request = goal_handle.request

        self.get_logger().info(f"Planning motion: {request.intent_type}")

        # Create feedback
        feedback = PlanMotion.Feedback()
        feedback.status = "planning"
        feedback.progress = 0.1
        goal_handle.publish_feedback(feedback)

        # Convert request to planner format
        goal = self._goal_msg_to_dict(request)

        # Generate plan using core planner
        feedback.status = "generating_plan"
        feedback.progress = 0.3
        goal_handle.publish_feedback(feedback)

        try:
            plan = self._planner.plan_motion(
                intent_type=request.intent_type,
                goal=goal,
                context=self._context_msg_to_dict(request.context),
            )
        except Exception as e:
            self.get_logger().error(f"Planning failed: {e}")
            goal_handle.abort()
            return PlanMotion.Result(success=False, error_message=str(e))

        # Validate plan with safety validator
        feedback.status = "validating"
        feedback.progress = 0.6
        goal_handle.publish_feedback(feedback)

        if self._safety_client.service_is_ready():
            validation_result = await self._validate_plan_with_safety(plan)
            if not validation_result["approved"]:
                self.get_logger().warn(
                    f"Plan rejected by safety validator: {validation_result.get('reason')}"
                )
                goal_handle.abort()
                return PlanMotion.Result(
                    success=False,
                    error_message=f"Safety validation failed: {validation_result.get('reason')}",
                )
            plan.safety_certificate = validation_result.get("certificate")
        else:
            self.get_logger().warn("Safety validator unavailable, skipping validation")

        # Complete
        feedback.status = "complete"
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)

        elapsed_ms = (time.time() - start_time) * 1000
        self.get_logger().info(f"Planning completed in {elapsed_ms:.2f}ms")

        if elapsed_ms > 100.0:
            self.get_logger().warn(f"Planning took {elapsed_ms:.2f}ms (target: <100ms)")

        goal_handle.succeed()

        return PlanMotion.Result(
            success=True, plan=self._plan_to_msg(plan), planning_time_ms=elapsed_ms
        )

    async def _handle_execute_motion(self, goal_handle):
        """
        Handle ExecuteMotion action request

        Executes a motion plan with progress feedback
        """
        request = goal_handle.request
        plan_msg = request.plan

        self.get_logger().info("Executing motion plan")

        # Check if plan has safety certificate
        if not plan_msg.safety_certificate or not plan_msg.safety_certificate.valid:
            self.get_logger().error("Plan has no valid safety certificate")
            goal_handle.abort()
            return ExecuteMotion.Result(
                success=False, error_message="Plan lacks valid safety certificate"
            )

        # Convert message to plan
        plan = self._msg_to_plan(plan_msg)

        # Execute plan
        feedback = ExecuteMotion.Feedback()

        try:
            for i, primitive in enumerate(plan.primitives):
                # Update progress
                progress = float(i) / len(plan.primitives)
                feedback.progress = progress
                feedback.current_primitive = primitive.name
                feedback.status = f"executing_{primitive.name}"
                goal_handle.publish_feedback(feedback)

                # Execute primitive
                result = await self._execute_primitive(primitive)

                if not result["success"]:
                    self.get_logger().error(f"Primitive execution failed: {result.get('error')}")
                    goal_handle.abort()
                    return ExecuteMotion.Result(
                        success=False,
                        error_message=(
                            f"Execution failed at {primitive.name}: " f"{result.get('error')}"
                        ),
                    )

            # Complete
            feedback.progress = 1.0
            feedback.status = "complete"
            goal_handle.publish_feedback(feedback)

            goal_handle.succeed()
            return ExecuteMotion.Result(
                success=True,
                execution_time_ms=(time.time() - goal_handle.request.start_time) * 1000,
            )

        except Exception as e:
            self.get_logger().error(f"Execution failed: {e}")
            goal_handle.abort()
            return ExecuteMotion.Result(success=False, error_message=str(e))

    async def _validate_plan_with_safety(self, plan: MotionPlan) -> dict[str, Any]:
        """Validate plan with safety validator service"""
        request = ValidateTrajectory.Request()

        # Convert plan to trajectory message
        request.trajectory = self._plan_to_trajectory_msg(plan)

        # Call safety validator
        future = self._safety_client.call_async(request)
        await future

        response = future.result()

        return {
            "approved": response.approved,
            "reason": response.rejection_reason,
            "certificate": (
                self._msg_to_certificate(response.certificate) if response.approved else None
            ),
        }

    async def _execute_primitive(self, primitive) -> dict[str, Any]:
        """Execute a single motion primitive"""
        # This would integrate with Nav2/MoveIt2
        # For now, simulate execution
        await asyncio.sleep(0.1)  # Simulate execution time
        return {"success": True}

    def _goal_msg_to_dict(self, msg) -> dict[str, Any]:
        """Convert goal ROS message to dictionary"""
        return {
            "target_pose": {
                "position": {
                    "x": msg.target_pose.position.x,
                    "y": msg.target_pose.position.y,
                    "z": msg.target_pose.position.z,
                },
                "orientation": {
                    "x": msg.target_pose.orientation.x,
                    "y": msg.target_pose.orientation.y,
                    "z": msg.target_pose.orientation.z,
                    "w": msg.target_pose.orientation.w,
                },
            },
            "target_location": msg.target_location,
            "object_name": msg.object_name,
        }

    def _context_msg_to_dict(self, msg) -> dict[str, Any]:
        """Convert context ROS message to dictionary"""
        return {
            "robot_pose": msg.robot_pose if hasattr(msg, "robot_pose") else None,
            "environment": msg.environment if hasattr(msg, "environment") else {},
        }

    def _plan_to_msg(self, plan: MotionPlan) -> MotionPlanMsg:
        """Convert MotionPlan to ROS message"""
        msg = MotionPlanMsg()
        msg.primitives = [p.name for p in plan.primitives]
        msg.expected_duration = plan.expected_duration
        msg.planning_time = plan.planning_time

        if plan.safety_certificate:
            msg.safety_certificate.valid = plan.safety_certificate.valid
            msg.safety_certificate.issued_at = plan.safety_certificate.issued_at
            msg.safety_certificate.expires_at = plan.safety_certificate.expires_at
            msg.safety_certificate.plan_hash = plan.safety_certificate.plan_hash

        return msg

    def _msg_to_plan(self, msg) -> MotionPlan:
        """Convert ROS message to MotionPlan"""
        plan = MotionPlan()
        plan.expected_duration = msg.expected_duration
        plan.planning_time = msg.planning_time

        if msg.safety_certificate and msg.safety_certificate.valid:
            plan.safety_certificate = SafetyCertificate(
                valid=msg.safety_certificate.valid,
                issued_at=msg.safety_certificate.issued_at,
                expires_at=msg.safety_certificate.expires_at,
                plan_hash=msg.safety_certificate.plan_hash,
            )

        return plan

    def _plan_to_trajectory_msg(self, plan: MotionPlan):
        """Convert plan to trajectory message for safety validation"""
        # Placeholder - implement based on actual message definition
        from agent_ros_bridge_msgs.msg import Trajectory

        traj = Trajectory()
        traj.waypoints = []  # Would extract waypoints from primitives
        return traj

    def _msg_to_certificate(self, msg) -> SafetyCertificate:
        """Convert ROS certificate message to SafetyCertificate"""
        return SafetyCertificate(
            valid=msg.valid,
            issued_at=msg.issued_at,
            expires_at=msg.expires_at,
            plan_hash=msg.plan_hash,
        )


def main(args=None):
    """Main entry point for motion planner node"""
    rclpy.init(args=args)

    if not MSGS_AVAILABLE:
        print("ERROR: agent_ros_bridge_msgs not available")
        print("Please build the message package:")
        print("  cd ws && colcon build --packages-select agent_ros_bridge_msgs")
        return 1

    node = MotionPlannerROSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    main()
