"""opc-milestone-tracker detect module.

Detects milestones in user journey.
"""
import json
from datetime import datetime


MILESTONE_DEFINITIONS = {
    "first_product_launch": {
        "keywords": ["launched", "shipped", "released", "live", "product launch"],
        "description": "First product launch"
    },
    "first_customer": {
        "keywords": ["first customer", "first user", "first sale", "paid user", "got our first", "acquired first"],
        "description": "Acquired first customer"
    },
    "revenue_milestone": {
        "keywords": ["$100", "$1k", "$10k", "mrr", "revenue", "first dollar"],
        "description": "Revenue milestone"
    },
    "mvp_complete": {
        "keywords": ["mvp done", "mvp complete", "prototype done", "mvp is", "prototype is", "done and ready"],
        "description": "MVP completed"
    }
}


def main(context: dict) -> dict:
    """Detect milestones from journal entries.
    
    Args:
        context: Dictionary containing:
            - customer_id: The customer identifier
            - input: Entry content or entries to analyze
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
        
        content = input_data.get("content", "")
        day = input_data.get("day", 1)
        
        # Simple keyword-based detection
        detected = []
        content_lower = content.lower()
        
        for milestone_id, definition in MILESTONE_DEFINITIONS.items():
            for keyword in definition["keywords"]:
                if keyword in content_lower:
                    detected.append({
                        "milestone_id": milestone_id,
                        "description": definition["description"],
                        "matched_keyword": keyword,
                        "day": day,
                        "confidence": 0.8
                    })
                    break
        
        return {
            "status": "success",
            "result": {
                "customer_id": customer_id,
                "day": day,
                "milestones_detected": detected,
                "count": len(detected)
            },
            "message": f"Detected {len(detected)} milestones for {customer_id}"
        }
        
    except Exception as e:
        return {
            "status": "error",
            "result": None,
            "message": f"Milestone detection failed: {str(e)}"
        }


if __name__ == "__main__":
    test_context = {
        "customer_id": "OPC-TEST-001",
        "input": {
            "content": "Finally launched the product today! Got our first customer within hours.",
            "day": 28
        }
    }
    
    result = main(test_context)
    print(json.dumps(result, indent=2, ensure_ascii=False))
