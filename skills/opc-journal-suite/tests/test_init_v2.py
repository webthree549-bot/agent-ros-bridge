"""TDD tests for init.py with execution_mode support."""

import pytest
import sys
from pathlib import Path
import os

# Add paths for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "opc-journal-core" / "scripts"))
sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    from init import main as init_main
except ImportError:
    # Direct import if running from test directory
    sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent / "opc-journal-core" / "scripts"))
    from init import main as init_main


class TestInitDirectMode:
    """Test direct execution mode (default)."""
    
    def test_init_returns_success(self, tmp_path, monkeypatch):
        """Should return success status in direct mode."""
        # Mock the home directory
        monkeypatch.setenv('HOME', str(tmp_path))
        
        context = {
            "customer_id": "OPC-TEST-001",
            "input": {"day": 1, "goals": ["Goal 1"]},
            "execution_mode": "direct"
        }
        
        result = init_main(context)
        
        assert result["status"] == "success"
        assert result["execution_mode"] == "direct"
    
    def test_init_creates_file(self, tmp_path, monkeypatch):
        """Should create customer memory file in direct mode."""
        monkeypatch.setenv('HOME', str(tmp_path))
        
        context = {
            "customer_id": "OPC-TEST-001",
            "input": {"day": 1, "goals": ["Goal 1"]},
            "execution_mode": "direct"
        }
        
        init_main(context)
        
        # Check file was created
        memory_file = tmp_path / ".openclaw" / "customers" / "OPC-TEST-001" / "memory"
        md_files = list(memory_file.glob("*.md"))
        assert len(md_files) > 0
    
    def test_init_returns_entry_data(self, tmp_path, monkeypatch):
        """Should return init entry data."""
        monkeypatch.setenv('HOME', str(tmp_path))
        
        context = {
            "customer_id": "OPC-TEST-001",
            "input": {"day": 1, "goals": ["Goal 1"]},
            "execution_mode": "direct"
        }
        
        result = init_main(context)
        
        assert result["result"]["customer_id"] == "OPC-TEST-001"
        assert result["result"]["day"] == 1
        assert result["result"]["initialized"] is True


class TestInitDelegatedMode:
    """Test delegated execution mode (tool calls)."""
    
    def test_init_returns_needs_tool_execution(self, tmp_path, monkeypatch):
        """Should return needs_tool_execution status."""
        monkeypatch.setenv('HOME', str(tmp_path))
        
        context = {
            "customer_id": "OPC-TEST-001",
            "input": {"day": 1, "goals": ["Goal 1"]},
            "execution_mode": "delegated"
        }
        
        result = init_main(context)
        
        assert result["status"] == "needs_tool_execution"
        assert result["execution_mode"] == "delegated"
    
    def test_init_returns_tool_calls(self, tmp_path, monkeypatch):
        """Should return tool_calls in result."""
        monkeypatch.setenv('HOME', str(tmp_path))
        
        context = {
            "customer_id": "OPC-TEST-001",
            "input": {"day": 1, "goals": ["Goal 1"]},
            "execution_mode": "delegated"
        }
        
        result = init_main(context)
        
        assert "tool_calls" in result["result"]
        assert len(result["result"]["tool_calls"]) > 0
    
    def test_init_tool_call_structure(self, tmp_path, monkeypatch):
        """Tool call should have correct structure."""
        monkeypatch.setenv('HOME', str(tmp_path))
        
        context = {
            "customer_id": "OPC-TEST-001",
            "input": {"day": 1},
            "execution_mode": "delegated"
        }
        
        result = init_main(context)
        
        tool_call = result["result"]["tool_calls"][0]
        assert tool_call["tool"] == "write"
        assert "params" in tool_call
        assert "path" in tool_call["params"]
        assert "content" in tool_call["params"]
        assert "OPC-TEST-001" in tool_call["params"]["path"]
    
    def test_init_no_actual_file_write(self, tmp_path, monkeypatch):
        """Should not write file in delegated mode."""
        monkeypatch.setenv('HOME', str(tmp_path))
        
        context = {
            "customer_id": "OPC-TEST-002",
            "input": {"day": 1},
            "execution_mode": "delegated"
        }
        
        result = init_main(context)
        
        # Verify no file was created
        customer_dir = tmp_path / ".openclaw" / "customers" / "OPC-TEST-002"
        assert not customer_dir.exists()


class TestInitBackwardCompatibility:
    """Test default behavior (backward compatible)."""
    
    def test_default_is_direct_mode(self, tmp_path, monkeypatch):
        """Default execution_mode should be 'direct'."""
        monkeypatch.setenv('HOME', str(tmp_path))
        
        context = {
            "customer_id": "OPC-TEST-001",
            "input": {"day": 1}
            # No execution_mode specified
        }
        
        result = init_main(context)
        
        assert result["execution_mode"] == "direct"
        assert result["status"] == "success"


class TestInitErrorHandling:
    """Test error handling."""
    
    def test_missing_customer_id(self):
        """Should error if customer_id missing."""
        context = {
            "input": {"day": 1}
        }
        
        result = init_main(context)
        
        assert result["status"] == "error"
        assert "customer_id" in result["message"].lower()
    
    def test_schema_version_present(self, tmp_path, monkeypatch):
        """Should include schema version."""
        monkeypatch.setenv('HOME', str(tmp_path))
        
        context = {
            "customer_id": "OPC-TEST-001",
            "input": {"day": 1},
            "execution_mode": "direct"
        }
        
        result = init_main(context)
        
        assert "_schema_version" in result
        assert result["_schema_version"] == "1.0"