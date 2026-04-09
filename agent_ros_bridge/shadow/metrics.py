"""Shadow mode metrics calculation."""

from typing import Any


def calculate_agreement_rate(decisions: list[dict[str, Any]]) -> float:
    """Calculate AI-human agreement rate.

    Args:
        decisions: List of decision records with 'agreement' boolean

    Returns:
        Agreement rate as percentage (0-100)
    """
    if not decisions:
        return 0.0

    agreements = sum(1 for d in decisions if d.get("agreement", False))
    rate = (agreements / len(decisions)) * 100
    return round(rate, 2)


def calculate_safety_score(decisions: list[dict[str, Any]]) -> float:
    """Calculate safety score based on decisions.

    Args:
        decisions: List of decision records

    Returns:
        Safety score (0-100)
    """
    if not decisions:
        return 100.0

    # Count safe vs unsafe decisions
    safe_count = sum(1 for d in decisions if d.get("safety_violations", 0) == 0)
    score = (safe_count / len(decisions)) * 100
    return round(score, 2)


def calculate_average_confidence(decisions: list[dict[str, Any]]) -> float:
    """Calculate average AI confidence.

    Args:
        decisions: List of decision records with 'ai_confidence'

    Returns:
        Average confidence (0-1)
    """
    if not decisions:
        return 0.0

    confidences = [d.get("ai_confidence", 0) for d in decisions if "ai_confidence" in d]
    if not confidences:
        return 0.0

    return round(sum(confidences) / len(confidences), 2)
