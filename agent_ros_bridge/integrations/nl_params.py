"""Natural language parameter inference.

Converts natural language descriptions to numeric parameters.
"""

import re
from typing import Optional

# Natural language to numeric mappings
NL_PARAM_MAPPINGS = {
    "speed": {
        "slowly": 0.1,
        "slow": 0.2,
        "gentle": 0.15,
        "gently": 0.15,
        "crawl": 0.05,
        "creeping": 0.05,
        "normal": 0.5,
        "medium": 0.5,
        "moderate": 0.4,
        "fast": 1.0,
        "quickly": 1.5,
        "quick": 1.2,
        "sprint": 2.0,
        "fastest": 2.0,
        "max": 2.0,
        "maximum": 2.0,
    },
    "distance": {
        "a bit": 0.5,
        "abit": 0.5,
        "a little": 0.3,
        "alittle": 0.3,
        "slightly": 0.2,
        "some": 1.0,
        "a lot": 2.0,
        "alot": 2.0,
        "far": 5.0,
        "long way": 10.0,
    },
    "angle": {
        "a bit": 15,
        "abit": 15,
        "slightly": 10,
        "a little": 15,
        "sharp": 90,
        "full": 360,
        "complete": 360,
        "half": 180,
        "quarter": 90,
        "ninety": 90,
        "forty five": 45,
    },
    "duration": {
        "briefly": 1,
        "moment": 2,
        "second": 1,
        "seconds": 1,
        "minute": 60,
        "minutes": 60,
        "hour": 3600,
    },
}


def infer_parameter(param_type: str, nl_value: str) -> Optional[float]:
    """Convert natural language to numeric parameter.

    Args:
        param_type: Type of parameter (speed, distance, angle, duration)
        nl_value: Natural language description

    Returns:
        Numeric value or None if not found

    Examples:
        >>> infer_parameter("speed", "slowly")
        0.1
        >>> infer_parameter("distance", "a bit")
        0.5
        >>> infer_parameter("angle", "90 degrees")
        90.0
    """
    if param_type not in NL_PARAM_MAPPINGS:
        return None

    mapping = NL_PARAM_MAPPINGS[param_type]
    nl_value_lower = nl_value.lower().strip()

    # Exact match
    if nl_value_lower in mapping:
        return mapping[nl_value_lower]

    # Try to extract number (e.g., "90 degrees" → 90)
    numeric = parse_numeric(nl_value)
    if numeric is not None:
        return numeric

    # Partial match (e.g., "slowly please" → "slowly")
    for key, value in mapping.items():
        if key in nl_value_lower:
            return value

    return None


def parse_numeric(value: str) -> Optional[float]:
    """Extract number from string.

    Handles:
    - "2 meters" → 2.0
    - "90 degrees" → 90.0
    - "1.5 m" → 1.5
    - "about 3" → 3.0

    Returns:
        Numeric value or None if no number found
    """
    # Remove common words
    cleaned = re.sub(r"\b(about|around|approximately|roughly)\b", "", value, flags=re.I)

    # Extract number
    match = re.search(r"(\d+(?:\.\d+)?)", cleaned)
    if match:
        return float(match.group(1))

    return None


def infer_speed(description: str) -> float:
    """Infer speed from description.

    Returns default 0.5 m/s if not specified.
    """
    speed = infer_parameter("speed", description)
    return speed if speed is not None else 0.5


def infer_distance(description: str) -> float:
    """Infer distance from description.

    Returns default 1.0 m if not specified.
    """
    distance = infer_parameter("distance", description)
    return distance if distance is not None else 1.0


def infer_angle(description: str) -> float:
    """Infer angle from description.

    Returns default 90 degrees if not specified.
    """
    angle = infer_parameter("angle", description)
    return angle if angle is not None else 90.0
