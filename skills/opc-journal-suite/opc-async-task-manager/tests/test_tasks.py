"""Tests for opc-async-task-manager skill.

TDD approach: tests define expected behavior.
"""
import pytest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "scripts"))

from create import main as create_task, generate_task_id
from status import main as check_status


class TestCreateTask:
    """Test task creation."""
    
    def test_create_success(self):
        """Should create task with valid description."""
        context = {
            "customer_id": "OPC-001",
            "input": {
                "type": "research",
                "description": "Research competitors",
                "timeout_hours": 8
            }
        }
        
        result = create_task(context)
        
        assert result["status"] == "success"
        assert "task_id" in result["result"]
        assert result["result"]["task"]["description"] == "Research competitors"
        assert result["result"]["task"]["timeout_hours"] == 8
    
    def test_create_missing_customer_id(self):
        """Should fail when customer_id is missing."""
        context = {
            "input": {
                "description": "test task"
            }
        }
        
        result = create_task(context)
        
        assert result["status"] == "error"
        assert "customer_id is required" in result["message"]
    
    def test_create_missing_description(self):
        """Should fail when description is missing."""
        context = {
            "customer_id": "OPC-001",
            "input": {"type": "research"}
        }
        
        result = create_task(context)
        
        assert result["status"] == "error"
        assert "description is required" in result["message"]
    
    def test_generate_task_id_format(self):
        """Task ID should follow TASK-YYYYMMDD-XXXXXX format."""
        task_id = generate_task_id()
        
        assert task_id.startswith("TASK-")
        assert len(task_id) == 20  # TASK-YYYYMMDD-XXXXXX


class TestCheckStatus:
    """Test status checking."""
    
    def test_status_success(self):
        """Should return search parameters for status check."""
        context = {
            "customer_id": "OPC-001",
            "input": {"task_id": "TASK-20260326-A1B2C3"}
        }
        
        result = check_status(context)
        
        assert result["status"] == "success"
        assert "search_params" in result["result"]
    
    def test_status_missing_customer_id(self):
        """Should fail when customer_id is missing."""
        context = {"input": {"task_id": "TASK-001"}}
        
        result = check_status(context)
        
        assert result["status"] == "error"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
