"""OPC Journal Suite Cron Scheduler.

Provides autonomous trigger capabilities for scheduled journal tasks.
Supports daily summaries, weekly patterns, and milestone checks.
"""
import json
from datetime import datetime, timedelta
from typing import Dict, List, Optional


# Default cron configurations
DEFAULT_SCHEDULES = {
    "daily_summary": {
        "enabled": True,
        "schedule": "0 8 * * *",  # 8:00 AM daily
        "description": "Generate daily summary at 8 AM"
    },
    "weekly_pattern": {
        "enabled": True,
        "schedule": "0 9 * * 0",  # 9:00 AM Sunday
        "description": "Weekly pattern analysis every Sunday"
    },
    "milestone_check": {
        "enabled": True,
        "schedule": "0 21 * * *",  # 9:00 PM daily
        "description": "Daily milestone detection"
    },
    "memory_compaction": {
        "enabled": True,
        "schedule": "0 23 * * *",  # 11:00 PM daily
        "description": "Daily memory compaction reminder"
    }
}


def should_trigger(schedule: str, last_run: Optional[str] = None) -> bool:
    """Check if a scheduled task should trigger now.
    
    Args:
        schedule: Cron expression (simplified - only HH:MM format supported)
        last_run: ISO timestamp of last execution
        
    Returns:
        True if should trigger
    """
    now = datetime.now()
    
    # Parse simple HH:MM format from cron (e.g., "0 8 * * *" -> 08:00)
    parts = schedule.split()
    if len(parts) >= 2:
        hour = int(parts[1])
        minute = int(parts[0])
        
        # Check if we're in the trigger window (within last 5 minutes)
        current_time = now.hour * 60 + now.minute
        trigger_time = hour * 60 + minute
        
        if last_run:
            last = datetime.fromisoformat(last_run)
            last_time = last.hour * 60 + last.minute
            last_day = last.date()
            
            # Already ran today
            if last_day == now.date():
                return False
        
        # Check if current time is at or past trigger time
        return current_time >= trigger_time
    
    return False


def get_next_trigger(schedule: str, from_time: Optional[datetime] = None) -> str:
    """Calculate next trigger time for a schedule.
    
    Args:
        schedule: Cron expression
        from_time: Calculate from this time (default: now)
        
    Returns:
        ISO timestamp of next trigger
    """
    if from_time is None:
        from_time = datetime.now()
    
    parts = schedule.split()
    if len(parts) >= 2:
        hour = int(parts[1])
        minute = int(parts[0])
        
        next_time = from_time.replace(hour=hour, minute=minute, second=0, microsecond=0)
        
        if next_time <= from_time:
            # Already passed today, schedule for tomorrow
            next_time += timedelta(days=1)
        
        return next_time.isoformat()
    
    return (from_time + timedelta(days=1)).isoformat()


def main(context: Dict) -> Dict:
    """Cron scheduler main function.
    
    Args:
        context: Dictionary containing:
            - customer_id: Customer identifier
            - input: Action type (check_triggers, get_schedule, update_schedule)
            - config: Custom schedule configuration (optional)
    
    Returns:
        Dictionary with status and trigger information
    """
    try:
        customer_id = context.get("customer_id")
        input_data = context.get("input", {})
        action = input_data.get("action", "check_triggers")
        
        if not customer_id:
            return {
                "status": "error",
                "result": None,
                "message": "customer_id is required"
            }
        
        # Get customer-specific config or use defaults
        config = input_data.get("config", DEFAULT_SCHEDULES)
        
        if action == "check_triggers":
            # Check which tasks should trigger
            triggers = []
            last_runs = input_data.get("last_runs", {})
            
            for task_name, task_config in config.items():
                if not task_config.get("enabled", True):
                    continue
                    
                schedule = task_config.get("schedule", "")
                last_run = last_runs.get(task_name)
                
                if should_trigger(schedule, last_run):
                    triggers.append({
                        "task": task_name,
                        "description": task_config.get("description", ""),
                        "target_skill": _get_target_skill(task_name),
                        "schedule": schedule
                    })
            
            return {
                "status": "success",
                "result": {
                    "customer_id": customer_id,
                    "timestamp": datetime.now().isoformat(),
                    "triggers": triggers,
                    "count": len(triggers)
                },
                "message": f"Found {len(triggers)} tasks to trigger"
            }
        
        elif action == "get_schedule":
            # Return full schedule with next trigger times
            schedule_info = {}
            
            for task_name, task_config in config.items():
                if task_config.get("enabled", True):
                    next_trigger = get_next_trigger(task_config.get("schedule", ""))
                else:
                    next_trigger = None
                
                schedule_info[task_name] = {
                    **task_config,
                    "next_trigger": next_trigger
                }
            
            return {
                "status": "success",
                "result": {
                    "customer_id": customer_id,
                    "schedules": schedule_info
                },
                "message": "Schedule retrieved successfully"
            }
        
        elif action == "update_schedule":
            # Update specific task schedule
            task_name = input_data.get("task_name")
            updates = input_data.get("updates", {})
            
            if task_name and task_name in config:
                config[task_name].update(updates)
                return {
                    "status": "success",
                    "result": {
                        "task": task_name,
                        "updated_config": config[task_name]
                    },
                    "message": f"Updated schedule for {task_name}"
                }
            else:
                return {
                    "status": "error",
                    "result": None,
                    "message": f"Task {task_name} not found"
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
            "message": f"Cron scheduler error: {str(e)}"
        }


def _get_target_skill(task_name: str) -> str:
    """Map task name to target skill."""
    mapping = {
        "daily_summary": "opc-insight-generator",
        "weekly_pattern": "opc-pattern-recognition",
        "milestone_check": "opc-milestone-tracker",
        "memory_compaction": "opc-journal-core"
    }
    return mapping.get(task_name, "opc-journal-suite")


if __name__ == "__main__":
    # Test examples
    test_contexts = [
        {
            "customer_id": "OPC-TEST-001",
            "input": {"action": "check_triggers"}
        },
        {
            "customer_id": "OPC-TEST-001",
            "input": {"action": "get_schedule"}
        }
    ]
    
    for ctx in test_contexts:
        result = main(ctx)
        print(json.dumps(result, indent=2, ensure_ascii=False))
