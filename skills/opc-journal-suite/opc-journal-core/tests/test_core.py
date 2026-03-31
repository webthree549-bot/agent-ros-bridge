"""Tests for opc-journal-core skill.

TDD approach: tests define expected behavior.
"""
import pytest
import sys
from pathlib import Path

# Add scripts to path
sys.path.insert(0, str(Path(__file__).parent.parent / "scripts"))

from init import main as init_journal
from record import main as record_entry, generate_entry_id
from search import main as search_entries
from export import main as export_entries


class TestInitJournal:
    """Test journal initialization."""
    
    def test_init_success(self):
        """Should initialize journal with valid customer_id."""
        context = {
            "customer_id": "OPC-001",
            "input": {
                "day": 1,
                "goals": ["Complete MVP"],
                "preferences": {"timezone": "Asia/Shanghai"}
            }
        }
        
        result = init_journal(context)
        
        assert result["status"] == "success"
        assert result["result"]["customer_id"] == "OPC-001"
        assert result["result"]["initialized"] is True
        assert result["result"]["day"] == 1
        assert result["result"]["goals_count"] == 1
    
    def test_init_missing_customer_id(self):
        """Should fail when customer_id is missing."""
        context = {"input": {"day": 1}}
        
        result = init_journal(context)
        
        assert result["status"] == "error"
        assert "customer_id is required" in result["message"]
    
    def test_init_default_day(self):
        """Should use day=1 as default."""
        context = {
            "customer_id": "OPC-002",
            "input": {}
        }
        
        result = init_journal(context)
        
        assert result["result"]["day"] == 1


class TestRecordEntry:
    """Test entry recording."""
    
    def test_record_success(self):
        """Should create entry with valid content."""
        context = {
            "customer_id": "OPC-001",
            "input": {
                "content": "Completed feature today",
                "day": 5,
                "metadata": {"emotional_state": "happy"}
            }
        }
        
        result = record_entry(context)
        
        assert result["status"] == "success"
        assert "entry_id" in result["result"]
        assert result["result"]["entry"]["content"] == "Completed feature today"
        assert result["result"]["entry"]["day"] == 5
        assert result["result"]["entry"]["metadata"]["emotional_state"] == "happy"
    
    def test_record_missing_customer_id(self):
        """Should fail when customer_id is missing."""
        context = {"input": {"content": "test"}}
        
        result = record_entry(context)
        
        assert result["status"] == "error"
        assert "customer_id is required" in result["message"]
    
    def test_record_missing_content(self):
        """Should fail when content is missing."""
        context = {
            "customer_id": "OPC-001",
            "input": {"day": 1}
        }
        
        result = record_entry(context)
        
        assert result["status"] == "error"
        assert "content is required" in result["message"]
    
    def test_generate_entry_id_format(self):
        """Entry ID should follow JE-YYYYMMDD-XXXXXX format."""
        entry_id = generate_entry_id("OPC-001")
        
        assert entry_id.startswith("JE-")
        assert len(entry_id) == 18  # JE-YYYYMMDD-XXXXXX


class TestSearchEntries:
    """Test entry searching."""
    
    def test_search_success(self):
        """Should return search parameters."""
        context = {
            "customer_id": "OPC-001",
            "input": {
                "query": "database issue",
                "filters": {"time_range": "last_7_days"}
            }
        }
        
        result = search_entries(context)
        
        assert result["status"] == "success"
        assert "search_params" in result["result"]
        assert "database issue" in result["result"]["search_params"]["query"]
        assert result["result"]["search_params"]["customer_id"] == "OPC-001"
    
    def test_search_missing_customer_id(self):
        """Should fail when customer_id is missing."""
        context = {"input": {"query": "test"}}
        
        result = search_entries(context)
        
        assert result["status"] == "error"


class TestExportEntries:
    """Test entry export."""
    
    def test_export_success(self):
        """Should return export parameters."""
        context = {
            "customer_id": "OPC-001",
            "input": {
                "format": "markdown",
                "time_range": "2026-W12",
                "sections": ["summary", "milestones"]
            }
        }
        
        result = export_entries(context)
        
        assert result["status"] == "success"
        assert result["result"]["export_format"] == "markdown"
        assert result["result"]["time_range"] == "2026-W12"
        assert "summary" in result["result"]["sections"]
    
    def test_export_default_format(self):
        """Should use markdown as default format."""
        context = {
            "customer_id": "OPC-001",
            "input": {}
        }
        
        result = export_entries(context)
        
        assert result["result"]["export_format"] == "markdown"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
