"""opc-journal-core record module.

Records journal entries using OpenClaw native tools.
Supports both direct execution and delegated execution modes.
Version: 2.2.2
"""
import json
import sys
import uuid
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


def generate_entry_id(customer_id: str) -> str:
    """Generate a unique entry ID."""
    today = datetime.now().strftime("%Y%m%d")
    suffix = uuid.uuid4().hex[:6].upper()
    return f"JE-{today}-{suffix}"


def main(context: dict) -> dict:
    """Record a journal entry.
    
    Args:
        context: Dictionary containing:
            - customer_id: The customer identifier
            - input: Entry content and metadata
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
        
        content = input_data.get("content")
        if not content:
            return {
                "status": "error",
                "result": None,
                "message": "content is required"
            }
        
        # Build entry
        entry_id = generate_entry_id(customer_id)
        entry = {
            "entry_id": entry_id,
            "entry_type": "journal_entry",
            "customer_id": customer_id,
            "content": content,
            "metadata": input_data.get("metadata", {}),
            "timestamp": datetime.now().isoformat(),
            "day": input_data.get("day", 1)
        }
        
        # Return entry for caller to store via memory tools
        return {
            "status": "success",
            "result": {
                "entry_id": entry_id,
                "entry": entry,
                "storage_hint": f"Write entry to memory file using 'write' tool"
            },
            "message": f"Entry {entry_id} created successfully"
        }
        
    except Exception as e:
        return {
            "status": "error",
            "result": None,
            "message": f"Failed to record entry: {str(e)}"
        }


if __name__ == "__main__":
    test_context = {
        "customer_id": "OPC-TEST-001",
        "input": {
            "content": "Completed user registration feature today, but encountered database connection issues",
            "day": 1,
            "metadata": {
                "agents_involved": ["DevAgent"],
                "emotional_state": "frustrated_but_determined"
            }
        }
    }
    
    result = main(test_context)
    print(json.dumps(result, indent=2, ensure_ascii=False))
