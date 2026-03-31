"""opc-journal-core search module.

Searches journal entries using OpenClaw native tools.
"""
import json
from datetime import datetime, timedelta


def main(context: dict) -> dict:
    """Search journal entries.
    
    Args:
        context: Dictionary containing:
            - customer_id: The customer identifier
            - input: Search parameters (query, filters, time_range)
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
        
        # Build search parameters for memory_search tool
        query = input_data.get("query", "")
        filters = input_data.get("filters", {})
        
        # Return search instruction for caller to execute via memory tools
        return {
            "status": "success",
            "result": {
                "search_params": {
                    "query": f"{query} customer_id:{customer_id} journal_entry",
                    "customer_id": customer_id
                },
                "filters": filters,
                "tool_hint": "Use 'memory_search' tool to find relevant entries"
            },
            "message": f"Search prepared for customer {customer_id}"
        }
        
    except Exception as e:
        return {
            "status": "error",
            "result": None,
            "message": f"Search failed: {str(e)}"
        }


if __name__ == "__main__":
    test_context = {
        "customer_id": "OPC-TEST-001",
        "input": {
            "query": "database connection",
            "filters": {
                "time_range": "last_30_days",
                "emotional_states": ["frustrated"]
            }
        }
    }
    
    result = main(test_context)
    print(json.dumps(result, indent=2, ensure_ascii=False))
