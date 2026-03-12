"""AI Layer for Agent ROS Bridge.

This module contains ROS2 nodes for natural language understanding and motion planning:
- intent_parser: Parse utterances into structured intents
- context_manager: Resolve contextual references
- motion_primitives: Motion primitive library
- motion_planner: Motion planning with Nav2/MoveIt2 integration
"""

# Existing AI components (may have ROS dependencies)
try:
    from .intent_parser import IntentParserNode
    from .context_manager import ContextManagerNode

    _HAS_ROS_NODES = True
except ImportError:
    # ROS2 not available (rclpy not installed)
    IntentParserNode = None
    ContextManagerNode = None
    _HAS_ROS_NODES = False

# Motion planning components (ROS-agnostic core)
from .motion_primitives import (
    MotionPrimitive,
    NavigateToPosePrimitive,
    PickObjectPrimitive,
    PlaceObjectPrimitive,
    GripperControlPrimitive,
    RotateInPlacePrimitive,
    MoveCartesianPrimitive,
    MotionPrimitiveFactory,
    navigate_to_pose,
    pick_object,
    place_object,
    gripper_control,
    rotate_in_place,
    move_cartesian,
)

from .motion_planner import (
    MotionPlannerNode,
    MotionPlan,
    PlanMotionResult,
    SafetyCertificate,
    SafetyValidator,
    Nav2Integration,
    MoveIt2Integration,
    create_motion_planner,
)

# ROS2 action server (new in v0.6.1)
try:
    from .motion_planner_node import MotionPlannerROSNode

    _MOTION_PLANNER_ROS_AVAILABLE = True
except ImportError:
    MotionPlannerROSNode = None
    _MOTION_PLANNER_ROS_AVAILABLE = False

# Advanced features (Week 6)
try:
    from .llm_parser import LLMIntentParser, LLMIntentResult

    _LLM_AVAILABLE = True
except ImportError:
    LLMIntentParser = None
    LLMIntentResult = None
    _LLM_AVAILABLE = False

try:
    from .context_aware_parser import ContextAwareParser, ConversationContext

    _CONTEXT_AVAILABLE = True
except ImportError:
    ContextAwareParser = None
    ConversationContext = None
    _CONTEXT_AVAILABLE = False

try:
    from .multi_language_parser import MultiLanguageParser, LanguagePatterns

    _MULTILANG_AVAILABLE = True
except ImportError:
    MultiLanguageParser = None
    LanguagePatterns = None
    _MULTILANG_AVAILABLE = False

from .execution_monitor import (
    ExecutionMonitorNode,
    ExecuteMotionResult,
    Anomaly,
    AnomalyType,
    RecoveryResult,
    RecoveryHandler,
    TelemetrySubscriber,
    create_execution_monitor,
)

__all__ = [
    # Existing AI components (may be None if ROS not available)
    "IntentParserNode",
    "ContextManagerNode",
    # Motion primitives
    "MotionPrimitive",
    "NavigateToPosePrimitive",
    "PickObjectPrimitive",
    "PlaceObjectPrimitive",
    "GripperControlPrimitive",
    "RotateInPlacePrimitive",
    "MoveCartesianPrimitive",
    "MotionPrimitiveFactory",
    "navigate_to_pose",
    "pick_object",
    "place_object",
    "gripper_control",
    "rotate_in_place",
    "move_cartesian",
    # Motion planner
    "MotionPlannerNode",
    "MotionPlan",
    "PlanMotionResult",
    "SafetyCertificate",
    "SafetyValidator",
    "Nav2Integration",
    "MoveIt2Integration",
    "create_motion_planner",
    # ROS2 action server
    "MotionPlannerROSNode",
    # Advanced features (Week 6)
    "LLMIntentParser",
    "LLMIntentResult",
    "ContextAwareParser",
    "ConversationContext",
    "MultiLanguageParser",
    "LanguagePatterns",
    # Execution monitor
    "ExecutionMonitorNode",
    "ExecuteMotionResult",
    "Anomaly",
    "AnomalyType",
    "RecoveryResult",
    "RecoveryHandler",
    "TelemetrySubscriber",
    "create_execution_monitor",
]
