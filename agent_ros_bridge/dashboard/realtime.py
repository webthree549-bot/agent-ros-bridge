"""Real-time dashboard for monitoring fleet and tasks."""

from dataclasses import dataclass, field
from typing import Any
from datetime import datetime


@dataclass
class RealTimeDashboard:
    """Real-time dashboard for fleet monitoring."""
    
    fleet_status: dict[str, Any] = field(default_factory=dict)
    task_visualizations: dict[str, Any] = field(default_factory=dict)
    performance_history: list[dict[str, Any]] = field(default_factory=list)
    
    def update_fleet_status(self, fleet_data: dict[str, Any]) -> None:
        """Update fleet status in dashboard."""
        self.fleet_status = {
            **fleet_data,
            "last_updated": datetime.now().isoformat(),
        }
    
    def get_current_status(self) -> dict[str, Any]:
        """Get current fleet status."""
        return self.fleet_status
    
    def visualize_tasks(self, tasks: list[dict[str, Any]]) -> dict[str, Any]:
        """Create visualization data for tasks."""
        viz = {}
        for task in tasks:
            task_id = task.get("id", "unknown")
            viz[task_id] = {
                "robot": task.get("robot", "unassigned"),
                "progress": task.get("progress", 0),
                "status": task.get("status", "unknown"),
                "color": self._get_status_color(task.get("status")),
            }
        
        self.task_visualizations = viz
        return viz
    
    def _get_status_color(self, status: str | None) -> str:
        """Get color for task status."""
        colors = {
            "pending": "gray",
            "executing": "blue",
            "completed": "green",
            "failed": "red",
            "cancelled": "orange",
        }
        return colors.get(status, "gray")
    
    def get_performance_metrics(self) -> dict[str, float]:
        """Calculate performance metrics."""
        # Mock metrics - in real implementation, calculate from history
        return {
            "avg_task_completion_time": 45.5,
            "fleet_utilization": 78.5,
            "success_rate": 94.2,
            "avg_battery_level": 67.3,
            "active_robots": len(self.fleet_status.get("robots", [])),
        }
    
    def check_anomalies(self, robot_data: dict[str, Any]) -> list[str]:
        """Check for anomalies and return alerts."""
        alerts = []
        
        # Check battery
        battery = robot_data.get("battery", 100)
        if battery < 10:
            alerts.append(f"CRITICAL: Robot {robot_data.get('id')} battery at {battery}%")
        elif battery < 20:
            alerts.append(f"WARNING: Robot {robot_data.get('id')} low battery ({battery}%)")
        
        # Check status anomalies
        if robot_data.get("status") == "error":
            alerts.append(f"ERROR: Robot {robot_data.get('id')} in error state")
        
        return alerts
