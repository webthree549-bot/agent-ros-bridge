"""opc-async-task-manager status module.

Checks status of async tasks.
"""
import json


def main(context: dict) -> dict:
    """Check task status.
    
    Args:
        context: Dictionary containing:
            - customer_id: The customer identifier
            - input: Task ID or filter criteria
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
        
        task_id = input_data.get("task_id")
        
        return {
            "status": "success",
            "result": {
                "search_params": {
                    "query": f"{task_id or 'TASK'} customer_id:{customer_id} task",
                    "customer_id": customer_id
                },
                "tool_hint": "Use 'memory_search' to find task status"
            },
            "message": f"Task status query prepared for {customer_id}"
        }
        
    except Exception as e:
        return {
            "status": "error",
            "result": None,
            "message": f"Status check failed: {str(e)}"
        }


if __name__ == "__main__":
    test_context = {
        "customer_id": "OPC-TEST-001",
        "input": {
            "task_id": "TASK-20260326-A1B2C3"
        }
    }
    
    result = main(test_context)
    print(json.dumps(result, indent=2, ensure_ascii=False))
