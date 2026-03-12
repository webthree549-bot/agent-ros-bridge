#!/usr/bin/env python3
"""
Context Manager Node - Week 2 Implementation.

ROS2 node for resolving contextual references in natural language.

Features:
- Location database (kitchen → pose mapping)
- Anaphoric reference resolution (it, there, here)
- Robot state tracking (pose from /tf)
- Session-based conversation context

Architecture:
    Query → Context Resolution → Resolved Pose/Entity
               ↓
    Location DB / Robot Pose / Conversation History
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import time
from typing import Dict, Optional, Any
from dataclasses import dataclass, field

from agent_ros_bridge_msgs.srv import ResolveContext
from agent_ros_bridge_msgs.msg import ContextQuery, ContextResponse, Intent
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf2_ros import Buffer, TransformListener


@dataclass
class ConversationContext:
    """Context for a conversation session."""

    last_object: Optional[str] = None
    last_location: Optional[str] = None
    last_intent: Optional[Intent] = None
    mentioned_objects: list = field(default_factory=list)
    visited_locations: list = field(default_factory=list)


class ContextManagerNode(Node):
    """
    Context Manager Node - Week 2 Implementation.

    Resolves contextual references (e.g., "it", "there", "kitchen")
    to concrete entities or poses.

    Services:
        /ai/resolve_context (ResolveContext): Resolve context query

    Subscriptions:
        /tf: Robot transforms for pose tracking

    Performance:
        - Context resolution: < 20ms (95th percentile)
    """

    # Location database: name → (x, y, z, frame_id)
    LOCATION_DATABASE: Dict[str, tuple] = {
        "kitchen": (5.0, 3.0, 0.0, "map"),
        "office": (10.0, 5.0, 0.0, "map"),
        "bedroom": (2.0, 8.0, 0.0, "map"),
        "living_room": (7.0, 7.0, 0.0, "map"),
        "garage": (0.0, 0.0, 0.0, "map"),
        "lobby": (12.0, 2.0, 0.0, "map"),
        "bathroom": (3.0, 5.0, 0.0, "map"),
        "hallway": (6.0, 5.0, 0.0, "map"),
        "entrance": (11.0, 1.0, 0.0, "map"),
        "exit": (11.5, 1.0, 0.0, "map"),
    }

    def __init__(self):
        super().__init__("context_manager")

        # Service for resolving context
        self.resolve_service = self.create_service(
            ResolveContext, "/ai/resolve_context", self.resolve_context_callback
        )

        # TF2 for robot pose tracking
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Robot pose cache: robot_id → PoseStamped
        self._robot_poses: Dict[str, PoseStamped] = {}

        # Conversation context: robot_id → ConversationContext
        self._conversation_context: Dict[str, ConversationContext] = {}

        # Session context: session_id → ConversationContext
        self._session_context: Dict[str, ConversationContext] = {}

        # Timer for updating robot poses from TF
        self._pose_update_timer = self.create_timer(0.1, self._update_robot_poses)  # 10Hz

        self.get_logger().info("Context Manager Node initialized")

    def resolve_context_callback(
        self, request: ResolveContext.Request, response: ResolveContext.Response
    ) -> ResolveContext.Response:
        """
        Resolve contextual reference to concrete entity or pose.

        Args:
            request: ResolveContext request with query
            response: ResolveContext response to populate

        Returns:
            Populated response with resolved context
        """
        start_time = time.time()
        query = request.query

        # Initialize response
        response.response = ContextResponse()
        response.response.found = False
        response.response.timestamp = self.get_clock().now().to_msg()

        # Get or create conversation context
        context = self._get_conversation_context(query.robot_id, query.session_id)

        # Resolve based on reference type
        if query.reference_type == "LOCATION":
            self._resolve_location(query, response.response, context)
        elif query.reference_type == "OBJECT":
            self._resolve_object(query, response.response, context)
        elif query.reference_type == "POSE":
            self._resolve_pose(query, response.response, context)
        elif query.reference_type == "ROBOT":
            self._resolve_robot(query, response.response, context)
        else:
            response.response.found = False
            response.error_message = f"Unknown reference type: {query.reference_type}"

        # Set response metadata
        response.success = True
        response.latency_ms = (time.time() - start_time) * 1000

        return response

    def _get_conversation_context(
        self, robot_id: str, session_id: Optional[str]
    ) -> ConversationContext:
        """Get or create conversation context for robot/session."""
        if session_id and session_id in self._session_context:
            return self._session_context[session_id]

        if robot_id not in self._conversation_context:
            self._conversation_context[robot_id] = ConversationContext()

        return self._conversation_context[robot_id]

    def _resolve_location(
        self, query: ContextQuery, response: ContextResponse, context: ConversationContext
    ) -> None:
        """Resolve location reference to pose."""
        ref_text = query.reference_text.lower().strip()

        # Handle anaphoric references
        if ref_text in ["there", "that place"]:
            if context.last_location:
                ref_text = context.last_location
            else:
                response.found = False
                response.description = "No previous location in context"
                return

        if ref_text == "here":
            # Current robot pose
            if query.robot_id in self._robot_poses:
                pose = self._robot_poses[query.robot_id]
                response.found = True
                response.resolved_type = "POSE"
                response.pose = pose
                response.entity_id = "current_pose"
                response.description = f"Current robot position at ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
                response.confidence = 0.95
            else:
                response.found = False
                response.description = "Robot pose not available"
            return

        # Look up in location database
        if ref_text in self.LOCATION_DATABASE:
            x, y, z, frame_id = self.LOCATION_DATABASE[ref_text]

            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0  # Default orientation

            response.found = True
            response.resolved_type = "POSE"
            response.pose = pose
            response.entity_id = ref_text
            response.description = f"{ref_text} at ({x}, {y}, {z}) in {frame_id} frame"
            response.confidence = 0.95

            # Update context
            context.last_location = ref_text
            if ref_text not in context.visited_locations:
                context.visited_locations.append(ref_text)
        else:
            response.found = False
            response.description = f"Unknown location: {ref_text}"
            response.confidence = 0.0

    def _resolve_object(
        self, query: ContextQuery, response: ContextResponse, context: ConversationContext
    ) -> None:
        """Resolve object reference."""
        ref_text = query.reference_text.lower().strip()

        # Handle anaphoric references
        if ref_text in ["it", "that", "this"]:
            if context.last_object:
                response.found = True
                response.resolved_type = "OBJECT"
                response.entity_id = context.last_object
                response.description = f"Previous object: {context.last_object}"
                response.confidence = 0.85
            else:
                response.found = False
                response.description = "No previous object in context"
                response.confidence = 0.0
            return

        # Direct object reference
        response.found = True
        response.resolved_type = "OBJECT"
        response.entity_id = ref_text
        response.description = f"Object: {ref_text}"
        response.confidence = 0.90

        # Update context
        context.last_object = ref_text
        if ref_text not in context.mentioned_objects:
            context.mentioned_objects.append(ref_text)

    def _resolve_pose(
        self, query: ContextQuery, response: ContextResponse, context: ConversationContext
    ) -> None:
        """Resolve pose reference."""
        ref_text = query.reference_text.lower().strip()

        if ref_text in ["here", "current position", "current pose"]:
            if query.robot_id in self._robot_poses:
                pose = self._robot_poses[query.robot_id]
                response.found = True
                response.resolved_type = "POSE"
                response.pose = pose
                response.entity_id = "current_pose"
                response.description = (
                    f"Current pose at ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
                )
                response.confidence = 0.95
            else:
                response.found = False
                response.description = "Robot pose not available"
        else:
            response.found = False
            response.description = f"Unknown pose reference: {ref_text}"

    def _resolve_robot(
        self, query: ContextQuery, response: ContextResponse, context: ConversationContext
    ) -> None:
        """Resolve robot reference."""
        ref_text = query.reference_text.lower().strip()

        if ref_text in ["you", "yourself"]:
            response.found = True
            response.resolved_type = "ROBOT"
            response.entity_id = query.robot_id
            response.description = f"Robot: {query.robot_id}"
            response.confidence = 0.95
        else:
            response.found = True
            response.resolved_type = "ROBOT"
            response.entity_id = ref_text
            response.description = f"Robot: {ref_text}"
            response.confidence = 0.90

    def _update_robot_poses(self):
        """Update robot poses from TF tree."""
        # For now, just update known robots
        # In a real implementation, this would query TF for all robot frames
        for robot_id in list(self._robot_poses.keys()):
            try:
                # Try to get transform from map to robot base
                transform = self._tf_buffer.lookup_transform(
                    "map", f"{robot_id}/base_link", rclpy.time.Time()
                )

                pose = PoseStamped()
                pose.header = transform.header
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation = transform.transform.rotation

                self._robot_poses[robot_id] = pose
            except Exception:
                # TF not available, use cached pose
                pass

    def update_context(self, robot_id: str, session_id: Optional[str], intent: Intent) -> None:
        """
        Update conversation context with parsed intent.

        This method is called by integration layer to maintain
        conversation state across multiple commands.

        Args:
            robot_id: Robot identifier
            session_id: Session identifier (optional)
            intent: Parsed intent to add to context
        """
        context = self._get_conversation_context(robot_id, session_id)
        context.last_intent = intent

        # Extract entities for context
        for entity in intent.entities:
            if entity.type == "OBJECT":
                context.last_object = entity.value
                if entity.value not in context.mentioned_objects:
                    context.mentioned_objects.append(entity.value)
            elif entity.type == "LOCATION":
                context.last_location = entity.value
                if entity.value not in context.visited_locations:
                    context.visited_locations.append(entity.value)


def main(args=None):
    """Main entry point for the context manager node."""
    rclpy.init(args=args)
    node = ContextManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down context manager node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
