"""Unified storage utilities for OPC skills."""

import os
from datetime import datetime
from typing import Dict, List, Any, Optional


def build_memory_path(customer_id: str, date: str = None) -> str:
    """Build standard memory file path.
    
    Args:
        customer_id: The customer identifier
        date: Date string (YYYY-MM-DD), defaults to today
        
    Returns:
        Path string with ~ expansion
    """
    if date is None:
        date = datetime.now().strftime("%Y-%m-%d")
    return f"~/.openclaw/customers/{customer_id}/memory/{date}.md"


def build_customer_dir(customer_id: str) -> str:
    """Build customer base directory path.
    
    Args:
        customer_id: The customer identifier
        
    Returns:
        Path string with ~ expansion
    """
    return f"~/.openclaw/customers/{customer_id}"


def write_memory_file(path: str, content: str, mode: str = "a") -> Dict[str, Any]:
    """Write content to memory file (direct mode).
    
    Args:
        path: File path (supports ~ expansion)
        content: Content to write
        mode: File open mode, default "a" (append)
        
    Returns:
        Dict with success status and details
    """
    try:
        full_path = os.path.expanduser(path)
        os.makedirs(os.path.dirname(full_path), exist_ok=True)
        
        with open(full_path, mode) as f:
            if mode == "a" and f.tell() > 0:
                f.write("\n\n")
            f.write(content)
        
        return {
            "success": True,
            "path": full_path,
            "bytes_written": len(content.encode('utf-8'))
        }
    except Exception as e:
        return {
            "success": False,
            "error": str(e),
            "error_type": "storage_error"
        }


def read_memory_file(path: str) -> Dict[str, Any]:
    """Read content from memory file.
    
    Args:
        path: File path (supports ~ expansion)
        
    Returns:
        Dict with success status and content
    """
    try:
        full_path = os.path.expanduser(path)
        
        if not os.path.exists(full_path):
            return {
                "success": False,
                "error": f"File not found: {path}",
                "error_type": "file_not_found"
            }
        
        with open(full_path, "r") as f:
            content = f.read()
        
        return {
            "success": True,
            "content": content,
            "path": full_path
        }
    except Exception as e:
        return {
            "success": False,
            "error": str(e),
            "error_type": "storage_error"
        }


def create_tool_call(tool: str, params: dict, sequence: int = 1) -> dict:
    """Create standardized tool call.
    
    Args:
        tool: Tool name (write, read, memory_search, etc.)
        params: Tool parameters
        sequence: Execution order
        
    Returns:
        Standardized tool call dict
    """
    return {
        "tool": tool,
        "params": params,
        "sequence": sequence
    }


def format_result(
    status: str,
    result: Any,
    message: str,
    execution_mode: str
) -> dict:
    """Format standardized result.
    
    Args:
        status: success | error | needs_tool_execution
        result: Result data
        message: Human-readable message
        execution_mode: direct | delegated
        
    Returns:
        Standardized result dict
    """
    return {
        "status": status,
        "result": result,
        "message": message,
        "execution_mode": execution_mode,
        "_schema_version": "1.0"
    }