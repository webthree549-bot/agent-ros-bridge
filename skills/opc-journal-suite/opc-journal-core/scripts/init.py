"""opc-journal-core initialization module.

Initializes journal for a new customer using OpenClaw native tools.
Supports both direct execution and delegated execution modes.
"""
import json
import sys
from datetime import datetime
from pathlib import Path

# Add parent directory to path for utils import
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

try:
    from utils.storage import (
        build_memory_path,
        write_memory_file,
        create_tool_call,
        format_result
    )
except ImportError:
    # Fallback for standalone execution
    def build_memory_path(customer_id: str, date: str = None) -> str:
        if date is None:
            date = datetime.now().strftime("%Y-%m-%d")
        return f"~/.openclaw/customers/{customer_id}/memory/{date}.md"
    
    def write_memory_file(path: str, content: str, mode: str = "a"):
        import os
        try:
            full_path = os.path.expanduser(path)
            os.makedirs(os.path.dirname(full_path), exist_ok=True)
            with open(full_path, "a") as f:
                if f.tell() > 0:
                    f.write("\n\n")
                f.write(content)
            return {"success": True, "path": full_path}
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def create_tool_call(tool: str, params: dict, sequence: int = 1):
        return {"tool": tool, "params": params, "sequence": sequence}
    
    def format_result(status: str, result: any, message: str, execution_mode: str):
        return {
            "status": status,
            "result": result,
            "message": message,
            "execution_mode": execution_mode,
            "_schema_version": "1.0"
        }


def format_init_entry(entry: dict) -> str:
    """Format entry as markdown."""
    goals_md = "\n".join(f"- {g}" for g in entry.get("goals", []))
    prefs_json = json.dumps(entry.get("preferences", {}), indent=2, ensure_ascii=False)
    
    return f"""# Journal Init - {entry['timestamp'][:10]}

**Entry Type**: {entry['entry_type']}  
**Customer**: {entry['customer_id']}  
**Day**: {entry['day']}

## Goals
{goals_md}

## Preferences
```json
{prefs_json}
```

---
*Version: {entry['version']}*"""


def main(context: dict) -> dict:
    """Initialize the opc-journal-core skill for a customer.
    
    Args:
        context: Dictionary containing:
            - customer_id: The customer identifier
            - input: Initialization parameters (day, goals, preferences)
            - execution_mode: "direct" (default) or "delegated"
    
    Returns:
        Dictionary with status, result, and message
    """
    execution_mode = context.get("execution_mode", "direct")
    
    try:
        customer_id = context.get("customer_id")
        input_data = context.get("input", {})
        
        if not customer_id:
            return format_result(
                status="error",
                result=None,
                message="customer_id is required",
                execution_mode=execution_mode
            )
        
        # Build initialization entry
        init_entry = {
            "entry_type": "journal_init",
            "customer_id": customer_id,
            "day": input_data.get("day", 1),
            "goals": input_data.get("goals", []),
            "preferences": input_data.get("preferences", {}),
            "timestamp": datetime.now().isoformat(),
            "version": "1.0"
        }
        
        # Format content and build path
        content = format_init_entry(init_entry)
        memory_path = build_memory_path(customer_id)
        
        if execution_mode == "delegated":
            # Return tool_calls for Agent to execute
            return format_result(
                status="needs_tool_execution",
                result={
                    "tool_calls": [
                        create_tool_call(
                            tool="write",
                            params={
                                "path": memory_path,
                                "content": content
                            },
                            sequence=1
                        )
                    ]
                },
                message=f"Tool execution required to initialize journal for {customer_id}",
                execution_mode="delegated"
            )
        else:
            # Direct execution - write file immediately
            write_result = write_memory_file(memory_path, content)
            
            if write_result["success"]:
                return format_result(
                    status="success",
                    result={
                        "customer_id": customer_id,
                        "initialized": True,
                        "day": init_entry["day"],
                        "goals_count": len(init_entry["goals"]),
                        "memory_path": memory_path
                    },
                    message=f"Journal initialized for {customer_id} (Day {init_entry['day']})",
                    execution_mode="direct"
                )
            else:
                return format_result(
                    status="error",
                    result={"error": write_result.get("error")},
                    message=f"Failed to write memory file: {write_result.get('error')}",
                    execution_mode="direct"
                )
        
    except Exception as e:
        return format_result(
            status="error",
            result=None,
            message=f"Initialization failed: {str(e)}",
            execution_mode=execution_mode
        )


if __name__ == "__main__":
    # Test entry point
    test_context = {
        "customer_id": "OPC-TEST-001",
        "input": {
            "day": 1,
            "goals": ["Complete product MVP", "Acquire first paying customer"],
            "preferences": {
                "communication_style": "friendly_professional",
                "work_hours": "09:00-18:00",
                "timezone": "Asia/Shanghai"
            }
        }
    }
    
    result = main(test_context)
    print(json.dumps(result, indent=2, ensure_ascii=False))
