"""Tests for opc-pattern-recognition skill.

TDD approach: tests define expected behavior.
"""
import pytest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "scripts"))

from analyze import main as analyze_patterns


class TestAnalyzePatterns:
    """Test pattern analysis."""
    
    def test_analyze_with_entries(self):
        """Should analyze patterns from entries."""
        context = {
            "customer_id": "OPC-001",
            "input": {
                "entries": [
                    {"metadata": {"emotional_state": "frustrated"}},
                    {"metadata": {"emotional_state": "excited"}},
                    {"metadata": {"emotional_state": "frustrated", "blockers": ["DB-001"]}},
                    {"metadata": {"emotional_state": "frustrated", "blockers": ["API-002"]}}
                ],
                "type": "weekly"
            }
        }
        
        result = analyze_patterns(context)
        
        assert result["status"] == "success"
        assert result["result"]["patterns"]["total_entries"] == 4
        assert "patterns" in result["result"]
        assert "emotional_trends" in result["result"]["patterns"]
        assert "common_blockers" in result["result"]["patterns"]
    
    def test_analyze_empty_entries(self):
        """Should return hint when no entries provided."""
        context = {
            "customer_id": "OPC-001",
            "input": {
                "entries": [],
                "type": "weekly"
            }
        }
        
        result = analyze_patterns(context)
        
        assert result["status"] == "success"
        assert "search_hint" in result["result"]
    
    def test_analyze_missing_customer_id(self):
        """Should fail when customer_id is missing."""
        context = {
            "input": {
                "entries": [{"metadata": {}}]
            }
        }
        
        result = analyze_patterns(context)
        
        assert result["status"] == "error"
        assert "customer_id is required" in result["message"]
    
    def test_analyze_emotional_trends(self):
        """Should count emotional states correctly."""
        context = {
            "customer_id": "OPC-001",
            "input": {
                "entries": [
                    {"metadata": {"emotional_state": "happy"}},
                    {"metadata": {"emotional_state": "happy"}},
                    {"metadata": {"emotional_state": "sad"}}
                ]
            }
        }
        
        result = analyze_patterns(context)
        
        trends = result["result"]["patterns"]["emotional_trends"]
        assert any(t[0] == "happy" and t[1] == 2 for t in trends)
        assert any(t[0] == "sad" and t[1] == 1 for t in trends)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
