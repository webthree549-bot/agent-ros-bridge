"""GazeboBatchRunner - Compatibility shim for refactored module.

This module re-exports from the new location for backward compatibility.
New code should import directly from gazebo_batch_runner, gazebo_types, or gazebo_metrics.
"""

# Re-export from new locations for backward compatibility
from .gazebo_batch_runner import GazeboBatchRunner
from .gazebo_types import WorldConfig, WorldResult, BatchConfig, ExecutionMetrics
from .gazebo_metrics import MetricsCollector

__all__ = [
    "GazeboBatchRunner",
    "WorldConfig",
    "WorldResult",
    "BatchConfig",
    "ExecutionMetrics",
    "MetricsCollector",
]
