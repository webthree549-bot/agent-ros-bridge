"""opc-pattern-recognition analyze module.

Analyzes patterns in user behavior.
"""
import json
from datetime import datetime
from collections import Counter


def main(context: dict) -> dict:
    """Analyze patterns from journal entries.
    
    Args:
        context: Dictionary containing:
            - customer_id: The customer identifier
            - input: Entries to analyze or time range
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
        
        entries = input_data.get("entries", [])
        analysis_type = input_data.get("type", "general")
        
        if not entries:
            return {
                "status": "success",
                "result": {
                    "search_hint": "Use memory_search to retrieve entries for analysis",
                    "customer_id": customer_id
                },
                "message": "No entries provided - search for entries first"
            }
        
        # Simple pattern analysis
        emotional_states = []
        blockers = []
        
        for entry in entries:
            metadata = entry.get("metadata", {})
            if "emotional_state" in metadata:
                emotional_states.append(metadata["emotional_state"])
            if "blockers" in metadata:
                blockers.extend(metadata["blockers"])
        
        patterns = {
            "emotional_trends": Counter(emotional_states).most_common(5),
            "common_blockers": Counter(blockers).most_common(5),
            "total_entries": len(entries)
        }
        
        return {
            "status": "success",
            "result": {
                "customer_id": customer_id,
                "analysis_type": analysis_type,
                "patterns": patterns
            },
            "message": f"Analyzed {len(entries)} entries for {customer_id}"
        }
        
    except Exception as e:
        return {
            "status": "error",
            "result": None,
            "message": f"Pattern analysis failed: {str(e)}"
        }


if __name__ == "__main__":
    test_context = {
        "customer_id": "OPC-TEST-001",
        "input": {
            "entries": [
                {"metadata": {"emotional_state": "frustrated"}},
                {"metadata": {"emotional_state": "excited"}},
                {"metadata": {"emotional_state": "frustrated", "blockers": ["DB-001"]}}
            ],
            "type": "weekly"
        }
    }
    
    result = main(test_context)
    print(json.dumps(result, indent=2, ensure_ascii=False))
