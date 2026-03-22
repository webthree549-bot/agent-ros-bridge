"""Shadow Mode - AI-Human Decision Comparison Framework.

This module provides the infrastructure for shadow mode operation,
where AI proposals are logged and compared against human actions
to validate AI decision quality before full autonomy.

Usage:
    from agent_ros_bridge.shadow import DecisionLogger, DecisionComparator

    logger = DecisionLogger()
    logger.log_ai_proposal(robot_id="bot1", proposal=ai_decision)
    logger.log_human_action(robot_id="bot1", action=human_decision)
"""

from .decision_logger import DecisionLogger
from .comparator import DecisionComparator
from .models import DecisionRecord, AIProposal, HumanAction, DecisionOutcome

__all__ = [
    "DecisionLogger",
    "DecisionComparator",
    "DecisionRecord",
    "AIProposal",
    "HumanAction",
    "DecisionOutcome",
]
