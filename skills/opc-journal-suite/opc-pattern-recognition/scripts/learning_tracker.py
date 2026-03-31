"""Learning tracker for pattern recognition.

Implements the "three-confirmation" learning promotion mechanism:
- 1st occurrence → Daily memory
- 3rd occurrence → Project memory
- Persistent → SOUL.md/TOOLS.md (manual review)
"""
import json
from datetime import datetime, timedelta
from collections import defaultdict
from typing import Dict, List, Optional, Any


# Learning promotion thresholds
LEARNING_THRESHOLDS = {
    "daily_memory": 1,      # First occurrence
    "project_memory": 3,    # Third occurrence (within 30 days)
    "permanent": 10,        # Ten occurrences (consider for SOUL.md)
}


class LearningTracker:
    """Tracks observations and promotes them through memory tiers."""
    
    def __init__(self, customer_id: str):
        self.customer_id = customer_id
        self.observations = defaultdict(lambda: {
            "count": 0,
            "first_seen": None,
            "last_seen": None,
            "instances": [],
            "promoted_to": None
        })
    
    def record_observation(self, 
                          category: str,
                          observation: str,
                          context: Dict[str, Any],
                          timestamp: Optional[str] = None) -> Dict:
        """Record an observation and check for promotion.
        
        Args:
            category: Type of observation (e.g., "work_hours", "decision_style")
            observation: The specific observation text
            context: Additional context
            timestamp: ISO timestamp (default: now)
            
        Returns:
            Dict with promotion status and recommendations
        """
        if timestamp is None:
            timestamp = datetime.now().isoformat()
        
        key = f"{category}:{observation}"
        obs = self.observations[key]
        
        # Initialize first seen
        if obs["first_seen"] is None:
            obs["first_seen"] = timestamp
        
        obs["count"] += 1
        obs["last_seen"] = timestamp
        obs["instances"].append({
            "timestamp": timestamp,
            "context": context
        })
        
        # Keep only last 10 instances
        obs["instances"] = obs["instances"][-10:]
        
        # Check for promotion
        promotion = self._check_promotion(key, obs)
        
        return {
            "observation_key": key,
            "category": category,
            "observation": observation,
            "count": obs["count"],
            "promotion": promotion,
            "recommendation": self._generate_recommendation(promotion, category, observation)
        }
    
    def _check_promotion(self, key: str, obs: Dict) -> Optional[str]:
        """Check if observation should be promoted to next tier."""
        count = obs["count"]
        current_tier = obs["promoted_to"]
        
        # Check time window for project_memory (30 days)
        if count >= LEARNING_THRESHOLDS["project_memory"]:
            first = datetime.fromisoformat(obs["first_seen"])
            if datetime.now() - first > timedelta(days=30):
                # Reset if too old
                return None
        
        # Determine promotion
        if count >= LEARNING_THRESHOLDS["permanent"] and current_tier != "permanent":
            obs["promoted_to"] = "permanent"
            return "permanent"
        
        if count >= LEARNING_THRESHOLDS["project_memory"] and current_tier not in ["project_memory", "permanent"]:
            obs["promoted_to"] = "project_memory"
            return "project_memory"
        
        if count >= LEARNING_THRESHOLDS["daily_memory"] and current_tier is None:
            obs["promoted_to"] = "daily_memory"
            return "daily_memory"
        
        return None
    
    def _generate_recommendation(self, 
                                promotion: Optional[str],
                                category: str, 
                                observation: str) -> str:
        """Generate human-readable recommendation."""
        if promotion == "daily_memory":
            return f"First observation of {category}: '{observation}'. Monitoring for patterns."
        
        elif promotion == "project_memory":
            templates = {
                "work_hours": f"Pattern confirmed: You tend to '{observation}'. Consider scheduling important tasks accordingly.",
                "decision_style": f"Decision pattern detected: You '{observation}'. This may affect your risk assessment.",
                "stress_triggers": f"Stress trigger identified: '{observation}'. Consider proactive mitigation.",
                "productivity_patterns": f"Productivity insight: You '{observation}'. Leverage this for better outcomes."
            }
            return templates.get(category, f"Confirmed pattern: {observation}")
        
        elif promotion == "permanent":
            return f"Strong pattern established: '{observation}'. Consider adding to SOUL.md for persistent behavior."
        
        return "Continue monitoring."
    
    def get_promoted_observations(self, tier: Optional[str] = None) -> List[Dict]:
        """Get all observations at or above a specific tier."""
        results = []
        
        tier_order = ["daily_memory", "project_memory", "permanent"]
        
        for key, obs in self.observations.items():
            if tier and obs["promoted_to"]:
                current_idx = tier_order.index(obs["promoted_to"]) if obs["promoted_to"] in tier_order else -1
                target_idx = tier_order.index(tier) if tier in tier_order else -1
                if current_idx < target_idx:
                    continue
            
            if obs["promoted_to"]:
                results.append({
                    "key": key,
                    **obs
                })
        
        return sorted(results, key=lambda x: x["count"], reverse=True)


def main(context: Dict) -> Dict:
    """Learning tracker main function.
    
    Args:
        context: Dictionary containing:
            - customer_id: Customer identifier
            - input: Action and parameters
    
    Returns:
        Dictionary with tracking results
    """
    try:
        customer_id = context.get("customer_id")
        input_data = context.get("input", {})
        action = input_data.get("action", "record")
        
        if not customer_id:
            return {
                "status": "error",
                "result": None,
                "message": "customer_id is required"
            }
        
        tracker = LearningTracker(customer_id)
        
        if action == "record":
            category = input_data.get("category", "general")
            observation = input_data.get("observation", "")
            ctx = input_data.get("context", {})
            
            if not observation:
                return {
                    "status": "error",
                    "result": None,
                    "message": "observation is required"
                }
            
            result = tracker.record_observation(category, observation, ctx)
            
            return {
                "status": "success",
                "result": result,
                "message": f"Recorded observation (count: {result['count']})"
            }
        
        elif action == "get_promoted":
            tier = input_data.get("tier")
            observations = tracker.get_promoted_observations(tier)
            
            return {
                "status": "success",
                "result": {
                    "observations": observations,
                    "count": len(observations),
                    "tier": tier or "all"
                },
                "message": f"Found {len(observations)} promoted observations"
            }
        
        else:
            return {
                "status": "error",
                "result": None,
                "message": f"Unknown action: {action}"
            }
    
    except Exception as e:
        return {
            "status": "error",
            "result": None,
            "message": f"Learning tracker error: {str(e)}"
        }


if __name__ == "__main__":
    # Test examples
    test_cases = [
        {
            "customer_id": "TEST-001",
            "input": {
                "action": "record",
                "category": "work_hours",
                "observation": "work late on Wednesdays",
                "context": {"day": "Wednesday", "hour": 23}
            }
        },
        {
            "customer_id": "TEST-001",
            "input": {
                "action": "get_promoted",
                "tier": "project_memory"
            }
        }
    ]
    
    for ctx in test_cases:
        result = main(ctx)
        print(json.dumps(result, indent=2, ensure_ascii=False))
