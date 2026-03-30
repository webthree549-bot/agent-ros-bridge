"""GazeboBatchRunner - Compatibility shim for refactored module.

This module re-exports from the new location for backward compatibility.
New code should import directly from gazebo_batch_runner, gazebo_types, or gazebo_metrics.
"""

# Re-export from new locations for backward compatibility
from .gazebo_batch_runner import GazeboBatchRunner
from .gazebo_metrics import MetricsCollector
from .gazebo_types import BatchConfig, ExecutionMetrics, WorldConfig, WorldResult

__all__ = [
    "BatchConfig",
    "ExecutionMetrics",
    "GazeboBatchRunner",
    "MetricsCollector",
    "WorldConfig",
    "WorldResult",
]
