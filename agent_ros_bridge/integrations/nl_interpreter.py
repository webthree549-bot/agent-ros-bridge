"""Rule-based natural language interpreter.

Converts natural language commands to ROS tool calls.
Works without LLM as a fallback.
"""

import re
from typing import Dict, Any, Optional, List, Tuple
from dataclasses import dataclass

from .nl_params import infer_speed, infer_distance, infer_angle


@dataclass
class InterpretationResult:
    """Result of NL interpretation."""

    tool: str
    params: Dict[str, Any]
    confidence: float = 1.0
    explanation: str = ""


class RuleBasedInterpreter:
    """Rule-based natural language interpreter.

    Converts natural language to structured ROS commands
    using regex patterns and rules.

    Examples:
        "Move forward 2 meters" → ros2_publish to /cmd_vel
        "Turn left 90 degrees" → ros2_publish with angular velocity
        "Stop" → ros2_publish with zero velocity
    """

    def __init__(self):
        self.patterns = self._compile_patterns()

    def _compile_patterns(self) -> List[Tuple[re.Pattern, callable]]:
        """Compile regex patterns for common commands."""
        return [
            # Movement patterns
            (
                re.compile(
                    r"(?:move|drive|go)\s+(forward|backward|back)\s+(?:for\s+)?([\d.]+)?\s*(?:m|meters?)?(?:\s+(\w+))?",
                    re.I,
                ),
                self._handle_move,
            ),
            (
                re.compile(r"(?:move|drive|go)\s+(forward|backward|back)(?:\s+(\w+))?", re.I),
                self._handle_move_vague,
            ),
            # Turn patterns
            (
                re.compile(r"turn\s+(left|right)(?:\s+([\d.]+))?\s*(?:degrees?|deg)?", re.I),
                self._handle_turn,
            ),
            (re.compile(r"turn\s+(left|right)(?:\s+(\w+))?", re.I), self._handle_turn_vague),
            # Spin patterns
            (re.compile(r"spin\s+(around|360|three sixty)", re.I), self._handle_spin),
            (re.compile(r"rotate\s+(360|180|90)\s*(?:degrees?)?", re.I), self._handle_spin_degrees),
            # Stop patterns
            (re.compile(r"^(stop|halt)$", re.I), self._handle_stop),
            (re.compile(r"stop\s+(?:moving|now|immediately)", re.I), self._handle_stop),
            # Navigation patterns
            (
                re.compile(r"(?:go|navigate|drive)\s+to\s+(?:the\s+)?(.+)", re.I),
                self._handle_navigate,
            ),
            (re.compile(r"(?:return|go back)\s+to\s+(?:the\s+)?(.+)", re.I), self._handle_navigate),
            # Sensor patterns
            (re.compile(r"what\s+(?:do\s+)?you\s+see", re.I), self._handle_camera),
            (re.compile(r"show\s+(?:me\s+)?(?:the\s+)?(?:camera|view)", re.I), self._handle_camera),
            (re.compile(r"(?:take|capture)\s+(?:a\s+)?photo", re.I), self._handle_camera),
            # Status patterns
            (re.compile(r"(?:what\s+is\s+your\s+)?status", re.I), self._handle_status),
            (re.compile(r"how\s+(?:is\s+your\s+)?battery", re.I), self._handle_battery),
            (re.compile(r"where\s+are\s+you", re.I), self._handle_location),
            # Fleet patterns
            (
                re.compile(r"list\s+(?:all\s+)?(?:available\s+)?robots", re.I),
                self._handle_list_robots,
            ),
            (re.compile(r"which\s+robots\s+(?:are|do)", re.I), self._handle_list_robots),
            # Safety patterns
            (re.compile(r"(?:emergency\s+)?stop", re.I), self._handle_estop),
            (re.compile(r"e-stop", re.I), self._handle_estop),
            (re.compile(r"halt\s+(?:all\s+)?(?:operations?)?", re.I), self._handle_estop),
        ]

    def interpret(self, nl_command: str, context: Optional[Dict] = None) -> Dict[str, Any]:
        """Interpret natural language command.

        Args:
            nl_command: Natural language command
            context: Optional context dict with known_locations, etc.

        Returns:
            Dict with tool name and parameters
        """
        nl_clean = nl_command.strip()

        for pattern, handler in self.patterns:
            match = pattern.match(nl_clean)
            if match:
                return handler(match, context)

        # Unknown command
        return {
            "error": "Unknown command",
            "command": nl_command,
            "suggestion": "Try: 'move forward 2 meters', 'turn left 90 degrees', 'stop', 'go to kitchen'",
        }

    def _handle_move(self, match, context):
        """Handle precise move command with distance."""
        direction = match.group(1).lower()
        distance_str = match.group(2) if match.group(2) else "1"
        speed_modifier = match.group(3) if len(match.groups()) > 2 and match.group(3) else ""

        distance = float(distance_str)
        speed = infer_speed(speed_modifier) if speed_modifier else 0.5

        linear_x = speed if direction in ["forward"] else -speed
        duration = distance / speed

        return {
            "tool": "ros2_publish",
            "topic": "/cmd_vel",
            "message_type": "geometry_msgs/Twist",
            "message": {
                "linear": {"x": linear_x, "y": 0, "z": 0},
                "angular": {"x": 0, "y": 0, "z": 0},
            },
            "duration": duration,
            "explanation": f"Move {direction} {distance}m at {speed}m/s for {duration:.1f}s",
        }

    def _handle_move_vague(self, match, context):
        """Handle vague move command without distance."""
        direction = match.group(1).lower()
        speed_modifier = match.group(2) if len(match.groups()) > 1 and match.group(2) else ""

        distance = 1.0  # Default 1 meter
        speed = infer_speed(speed_modifier) if speed_modifier else 0.5

        linear_x = speed if direction in ["forward"] else -speed
        duration = distance / speed

        return {
            "tool": "ros2_publish",
            "topic": "/cmd_vel",
            "message_type": "geometry_msgs/Twist",
            "message": {
                "linear": {"x": linear_x, "y": 0, "z": 0},
                "angular": {"x": 0, "y": 0, "z": 0},
            },
            "duration": duration,
            "explanation": f"Move {direction} {distance}m at {speed}m/s (default distance)",
        }

    def _handle_turn(self, match, context):
        """Handle precise turn command with angle."""
        direction = match.group(1).lower()
        angle_str = match.group(2) if match.group(2) else "90"

        angle = float(angle_str)
        angular_z = 0.5 if direction == "left" else -0.5
        duration = (angle * 3.14159 / 180) / abs(angular_z)

        return {
            "tool": "ros2_publish",
            "topic": "/cmd_vel",
            "message_type": "geometry_msgs/Twist",
            "message": {
                "linear": {"x": 0, "y": 0, "z": 0},
                "angular": {"x": 0, "y": 0, "z": angular_z},
            },
            "duration": duration,
            "explanation": f"Turn {direction} {angle}° at {abs(angular_z)}rad/s for {duration:.1f}s",
        }

    def _handle_turn_vague(self, match, context):
        """Handle vague turn command."""
        direction = match.group(1).lower()
        modifier = match.group(2) if len(match.groups()) > 1 and match.group(2) else ""

        angle = infer_angle(modifier) if modifier else 90
        angular_z = 0.5 if direction == "left" else -0.5
        duration = (angle * 3.14159 / 180) / abs(angular_z)

        return {
            "tool": "ros2_publish",
            "topic": "/cmd_vel",
            "message_type": "geometry_msgs/Twist",
            "message": {
                "linear": {"x": 0, "y": 0, "z": 0},
                "angular": {"x": 0, "y": 0, "z": angular_z},
            },
            "duration": duration,
            "explanation": f"Turn {direction} {angle}° (inferred from '{modifier}')",
        }

    def _handle_spin(self, match, context):
        """Handle spin around command."""
        angular_z = 1.0  # 1 rad/s
        duration = 6.28  # 2π seconds for 360°

        return {
            "tool": "ros2_publish",
            "topic": "/cmd_vel",
            "message_type": "geometry_msgs/Twist",
            "message": {
                "linear": {"x": 0, "y": 0, "z": 0},
                "angular": {"x": 0, "y": 0, "z": angular_z},
            },
            "duration": duration,
            "explanation": "Spin 360° at 1 rad/s",
        }

    def _handle_spin_degrees(self, match, context):
        """Handle spin with specific degrees."""
        angle = float(match.group(1))
        angular_z = 1.0
        duration = (angle * 3.14159 / 180) / abs(angular_z)

        return {
            "tool": "ros2_publish",
            "topic": "/cmd_vel",
            "message_type": "geometry_msgs/Twist",
            "message": {
                "linear": {"x": 0, "y": 0, "z": 0},
                "angular": {"x": 0, "y": 0, "z": angular_z},
            },
            "duration": duration,
            "explanation": f"Spin {angle}° at 1 rad/s",
        }

    def _handle_stop(self, match, context):
        """Handle stop command."""
        return {
            "tool": "ros2_publish",
            "topic": "/cmd_vel",
            "message_type": "geometry_msgs/Twist",
            "message": {"linear": {"x": 0, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}},
            "explanation": "Stop all motion",
        }

    def _handle_navigate(self, match, context):
        """Handle navigation command."""
        location = match.group(1).strip()

        # Check if location is known in context
        if context and "known_locations" in context:
            loc = context["known_locations"].get(location.lower())
            if loc:
                return {
                    "tool": "ros2_action_goal",
                    "action_name": "/navigate_to_pose",
                    "action_type": "nav2_msgs/NavigateToPose",
                    "goal": {"pose": {"position": loc}},
                    "explanation": f"Navigate to known location: {location}",
                }

        # Generic navigation - will need to be resolved at runtime
        return {
            "tool": "ros2_action_goal",
            "action_name": "/navigate_to_pose",
            "action_type": "nav2_msgs/NavigateToPose",
            "goal": {"pose": {"position": {"x": 0, "y": 0}}},
            "location_name": location,
            "explanation": f"Navigate to: {location} (coordinates to be resolved)",
            "note": f"Location '{location}' not in known places",
        }

    def _handle_camera(self, match, context):
        """Handle camera/sensor command."""
        return {
            "tool": "ros2_camera_snapshot",
            "topic": "/camera/image_raw",
            "explanation": "Capture camera image",
        }

    def _handle_status(self, match, context):
        """Handle status query."""
        return {"tool": "bridge_get_robot_status", "explanation": "Get robot status"}

    def _handle_battery(self, match, context):
        """Handle battery query."""
        return {
            "tool": "ros2_subscribe_once",
            "topic": "/battery_state",
            "explanation": "Check battery level",
        }

    def _handle_location(self, match, context):
        """Handle location query."""
        return {
            "tool": "ros2_subscribe_once",
            "topic": "/odom",
            "explanation": "Get current location",
        }

    def _handle_list_robots(self, match, context):
        """Handle fleet listing."""
        return {"tool": "bridge_list_robots", "explanation": "List all robots"}

    def _handle_estop(self, match, context):
        """Handle emergency stop."""
        return {
            "tool": "safety_trigger_estop",
            "reason": "User requested emergency stop",
            "explanation": "EMERGENCY STOP triggered",
        }
