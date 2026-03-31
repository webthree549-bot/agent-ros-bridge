"""opc-async-task-manager create module.

Creates async tasks for background execution.
"""
import json
import uuid
from datetime import datetime, timedelta


def generate_task_id() -> str:
    """Generate unique task ID."""
    today = datetime.now().strftime("%Y%m%d")
    suffix = uuid.uuid4().hex[:6].upper()
    return f"TASK-{today}-{suffix}"


def main(context: dict) -> dict:
    """Create an async task.
    
    Args:
        context: Dictionary containing:
            - customer_id: The customer identifier
            - input: Task description, type, timeout
            - memory: Memory context
    
    Returns:
        Dictionary with status, result, and message
    """
    try:
        customer_id = context.get("customer_id")
        input_data = context.get("input", {})
        
        if not customer_id:
            return {
                "status": "error",
                "result": None,
                "message": "customer_id is required"
            }
        
        task_type = input_data.get("type", "research")
        description = input_data.get("description", "")
        timeout_hours = input_data.get("timeout_hours", 8)
        
        if not description:
            return {
                "status": "error",
                "result": None,
                "message": "description is required"
            }
        
        task_id = generate_task_id()
        
        # Calculate estimated completion
        now = datetime.now()
        eta = now + timedelta(hours=timeout_hours)
        
        task = {
            "task_id": task_id,
            "task_type": task_type,
            "customer_id": customer_id,
            "description": description,
            "status": "created",
            "created_at": now.isoformat(),
            "estimated_completion": eta.isoformat(),
            "timeout_hours": timeout_hours
        }
        
        return {
            "status": "success",
            "result": {
                "task_id": task_id,
                "task": task,
                "storage_hint": "Store task in memory for tracking"
            },
            "message": f"Task {task_id} created, ETA: {eta.strftime('%Y-%m-%d %H:%M')}"
        }
        
    except Exception as e:
        return {
            "status": "error",
            "result": None,
            "message": f"Failed to create task: {str(e)}"
        }


if __name__ == "__main__":
    test_context = {
        "customer_id": "OPC-TEST-001",
        "input": {
            "type": "research",
            "description": "Research competitor pricing strategies",
            "timeout_hours": 8
        }
    }
    
    result = main(test_context)
    print(json.dumps(result, indent=2, ensure_ascii=False))
