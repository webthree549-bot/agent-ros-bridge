"""Advanced mission planning with conditionals and optimization."""

import math
from typing import Any


class AdvancedMissionPlanner:
    """Advanced mission planner with conditionals and dynamic replanning."""
    
    def evaluate_condition(self, step: dict[str, Any], context: dict[str, Any]) -> str:
        """Evaluate if/then/else condition in mission step.
        
        Returns:
            "then" if condition is true, "else" otherwise
        """
        condition = step.get("if", {})
        
        # Check battery condition
        if "battery" in condition:
            battery_cond = condition["battery"]
            current_battery = context.get("battery", 100)
            
            if "lt" in battery_cond:
                if current_battery < battery_cond["lt"]:
                    return "then"
            if "gt" in battery_cond:
                if current_battery > battery_cond["gt"]:
                    return "then"
            if "eq" in battery_cond:
                if current_battery == battery_cond["eq"]:
                    return "then"
        
        # Default to else branch
        return "else"
    
    def get_parallel_tasks(self, mission: dict[str, Any]) -> list[dict[str, Any]]:
        """Extract parallel tasks from mission."""
        return mission.get("parallel", [])
    
    def replan_on_failure(
        self,
        original_plan: list[dict[str, Any]],
        failed_step: dict[str, Any]
    ) -> list[dict[str, Any]]:
        """Generate alternative plan when step fails."""
        new_plan = []
        
        for step in original_plan:
            if step.get("action") == failed_step.get("action"):
                # Add alternative action
                new_plan.append({
                    **step,
                    "alternative": True,
                    "note": "Retry with caution",
                })
            else:
                new_plan.append(step)
        
        return new_plan
    
    def optimize_task_order(
        self,
        tasks: list[dict[str, Any]],
        start_location: tuple[float, float]
    ) -> list[dict[str, Any]]:
        """Optimize task order to minimize travel distance."""
        if not tasks:
            return tasks
        
        # Simple nearest-neighbor algorithm
        remaining = tasks.copy()
        optimized = []
        current_pos = start_location
        
        while remaining:
            # Find closest task
            closest = min(
                remaining,
                key=lambda t: self._distance(
                    current_pos,
                    t.get("location", (0, 0))
                )
            )
            
            optimized.append(closest)
            remaining.remove(closest)
            current_pos = closest.get("location", current_pos)
        
        return optimized
    
    def _distance(
        self,
        p1: tuple[float, float],
        p2: tuple[float, float]
    ) -> float:
        """Calculate Euclidean distance."""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
