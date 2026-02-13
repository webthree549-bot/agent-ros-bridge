"""
Legacy v1 code - DEPRECATED

This module contains the v1 codebase preserved for backward compatibility.
Please migrate to v2 (openclaw_ros_bridge.gateway_v2).

See docs/MIGRATION.md for migration guide.
"""

import warnings

warnings.warn(
    "openclaw_ros_bridge.legacy is deprecated. "
    "Please use openclaw_ros_bridge.gateway_v2 instead. "
    "See docs/MIGRATION.md for migration guide.",
    DeprecationWarning,
    stacklevel=2
)

# Re-export legacy modules for backward compatibility
# These will be removed in a future version
