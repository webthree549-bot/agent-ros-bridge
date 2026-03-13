#!/usr/bin/env python3
"""
Safety Validator ROS2 Node
Agent ROS Bridge v0.6.1 - Week 2 Implementation

Exposes safety validation as ROS2 services:
- /safety/validate_trajectory (ValidateTrajectory)
- /safety/get_limits (GetSafetyLimits)
- /safety/get_status (GetSafetyStatus)

Target: <10ms validation response time
"""

import time
from typing import Any

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

# Import safety validator core
from .validator import SafetyValidatorNode

# Try to import ROS message types
try:
    from agent_ros_bridge_msgs.msg import SafetyCertificate, SafetyLimits
    from agent_ros_bridge_msgs.srv import (GetSafetyLimits, GetSafetyStatus,
                                           ValidateTrajectory)

    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False
    ValidateTrajectory = None
    GetSafetyLimits = None
    GetSafetyStatus = None
    SafetyCertificate = None
    SafetyLimits = None


class SafetyValidatorROSNode(Node):
    """
    Safety Validator ROS2 Node

    Wraps SafetyValidatorNode with ROS2 service interfaces.
    Provides safety validation services for motion planning.
    """

    def __init__(self):
        super().__init__("safety_validator")

        if not MSGS_AVAILABLE:
            self.get_logger().error(
                "agent_ros_bridge_msgs not available. Safety services disabled."
            )
            return

        # Core validator
        self._validator = SafetyValidatorNode()

        # Callback group for concurrent service handling
        self._callback_group = ReentrantCallbackGroup()

        # Services
        self._validate_srv = self.create_service(
            ValidateTrajectory,
            "/safety/validate_trajectory",
            self._handle_validate_trajectory,
            callback_group=self._callback_group,
        )

        self._get_limits_srv = self.create_service(
            GetSafetyLimits,
            "/safety/get_limits",
            self._handle_get_limits,
            callback_group=self._callback_group,
        )

        self._get_status_srv = self.create_service(
            GetSafetyStatus,
            "/safety/get_status",
            self._handle_get_status,
            callback_group=self._callback_group,
        )

        # Default safety limits
        self._default_limits = {
            "max_velocity": 1.0,  # m/s
            "max_acceleration": 2.0,  # m/s^2
            "max_force": 100.0,  # N
            "workspace_bounds": {
                "x_min": -5.0,
                "x_max": 5.0,
                "y_min": -5.0,
                "y_max": 5.0,
                "z_min": 0.0,
                "z_max": 2.0,
            },
        }

        self.get_logger().info("Safety Validator Node initialized")
        self.get_logger().info("Services:")
        self.get_logger().info("  - /safety/validate_trajectory")
        self.get_logger().info("  - /safety/get_limits")
        self.get_logger().info("  - /safety/get_status")

    def _handle_validate_trajectory(self, request, response):
        """
        Handle ValidateTrajectory service request

        Target: <10ms response time
        """
        start_time = time.time()

        # Convert ROS message to dict
        trajectory = self._trajectory_msg_to_dict(request.trajectory)
        limits = (
            self._limits_msg_to_dict(request.limits) if request.limits else self._default_limits
        )

        # Validate
        result = self._validator.validate_trajectory(trajectory, limits)

        # Convert result to ROS message
        response.approved = result["approved"]
        response.certificate = self._certificate_dict_to_msg(result.get("certificate"))
        response.validation_time_ms = result.get("validation_time_ms", 0.0)
        response.rejection_reason = result.get("rejection_reason", "")

        elapsed_ms = (time.time() - start_time) * 1000
        self.get_logger().debug(f"Validation completed in {elapsed_ms:.2f}ms")

        if elapsed_ms > 10.0:
            self.get_logger().warn(f"Validation took {elapsed_ms:.2f}ms (target: <10ms)")

        return response

    def _handle_get_limits(self, request, response):
        """Handle GetSafetyLimits service request"""
        response.limits = self._limits_dict_to_msg(self._default_limits)
        return response

    def _handle_get_status(self, request, response):
        """Handle GetSafetyStatus service request"""
        stats = self._validator.get_statistics()
        response.validation_count = stats["validation_count"]
        response.rejection_count = stats["rejection_count"]
        response.average_validation_time_ms = stats["average_validation_time_ms"]
        response.uptime_seconds = stats.get("uptime_seconds", 0.0)
        return response

    def _trajectory_msg_to_dict(self, msg) -> dict[str, Any]:
        """Convert trajectory ROS message to dictionary"""
        # Placeholder - implement based on actual message definition
        return {"waypoints": [], "velocities": [], "accelerations": []}

    def _limits_msg_to_dict(self, msg) -> dict[str, Any]:
        """Convert limits ROS message to dictionary"""
        return {
            "max_velocity": msg.max_velocity,
            "max_acceleration": msg.max_acceleration,
            "max_force": msg.max_force,
            "workspace_bounds": {
                "x_min": msg.workspace_x_min,
                "x_max": msg.workspace_x_max,
                "y_min": msg.workspace_y_min,
                "y_max": msg.workspace_y_max,
                "z_min": msg.workspace_z_min,
                "z_max": msg.workspace_z_max,
            },
        }

    def _limits_dict_to_msg(self, limits: dict[str, Any]):
        """Convert limits dictionary to ROS message"""
        msg = SafetyLimits()
        msg.max_velocity = limits.get("max_velocity", 1.0)
        msg.max_acceleration = limits.get("max_acceleration", 2.0)
        msg.max_force = limits.get("max_force", 100.0)
        bounds = limits.get("workspace_bounds", {})
        msg.workspace_x_min = bounds.get("x_min", -5.0)
        msg.workspace_x_max = bounds.get("x_max", 5.0)
        msg.workspace_y_min = bounds.get("y_min", -5.0)
        msg.workspace_y_max = bounds.get("y_max", 5.0)
        msg.workspace_z_min = bounds.get("z_min", 0.0)
        msg.workspace_z_max = bounds.get("z_max", 2.0)
        return msg

    def _certificate_dict_to_msg(self, cert: dict[str, Any] | None):
        """Convert certificate dictionary to ROS message"""
        if cert is None:
            return SafetyCertificate()

        msg = SafetyCertificate()
        msg.certificate_id = cert.get("certificate_id", "")
        msg.trajectory_hash = cert.get("trajectory_hash", "")
        msg.issued_at = cert.get("issued_at", "")
        msg.expires_at = cert.get("expires_at", "")
        msg.constraints_checked = cert.get("constraints_checked", [])
        return msg


def main(args=None):
    """Main entry point for safety validator node"""
    rclpy.init(args=args)

    if not MSGS_AVAILABLE:
        print("ERROR: agent_ros_bridge_msgs not available")
        print("Please build the message package:")
        print("  cd ws && colcon build --packages-select agent_ros_bridge_msgs")
        return 1

    node = SafetyValidatorROSNode()

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
