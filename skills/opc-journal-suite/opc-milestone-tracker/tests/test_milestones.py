"""Tests for opc-milestone-tracker skill.

TDD approach: tests define expected behavior.
"""
import pytest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "scripts"))

from detect import main as detect_milestones, MILESTONE_DEFINITIONS


class TestDetectMilestones:
    """Test milestone detection."""
    
    def test_detect_product_launch(self):
        """Should detect product launch milestone."""
        context = {
            "customer_id": "OPC-001",
            "input": {
                "content": "Finally launched the product today!",
                "day": 28
            }
        }
        
        result = detect_milestones(context)
        
        assert result["status"] == "success"
        assert result["result"]["count"] >= 1
        assert any(m["milestone_id"] == "first_product_launch" 
                   for m in result["result"]["milestones_detected"])
    
    def test_detect_first_customer(self):
        """Should detect first customer milestone."""
        context = {
            "customer_id": "OPC-001",
            "input": {
                "content": "Got our first paying customer today!",
                "day": 45
            }
        }
        
        result = detect_milestones(context)
        
        assert result["status"] == "success"
        assert any(m["milestone_id"] == "first_customer" 
                   for m in result["result"]["milestones_detected"])
    
    def test_detect_mvp_complete(self):
        """Should detect MVP completion milestone."""
        context = {
            "customer_id": "OPC-001",
            "input": {
                "content": "MVP is finally done and ready for testing",
                "day": 21
            }
        }
        
        result = detect_milestones(context)
        
        assert result["status"] == "success"
        assert any(m["milestone_id"] == "mvp_complete" 
                   for m in result["result"]["milestones_detected"])
    
    def test_detect_no_milestone(self):
        """Should return empty when no milestone detected."""
        context = {
            "customer_id": "OPC-001",
            "input": {
                "content": "Just another regular day of coding",
                "day": 10
            }
        }
        
        result = detect_milestones(context)
        
        assert result["status"] == "success"
        assert result["result"]["count"] == 0
    
    def test_detect_missing_customer_id(self):
        """Should fail when customer_id is missing."""
        context = {
            "input": {
                "content": "launched product",
                "day": 1
            }
        }
        
        result = detect_milestones(context)
        
        assert result["status"] == "error"
    
    def test_milestone_definitions_exist(self):
        """Should have milestone definitions."""
        assert "first_product_launch" in MILESTONE_DEFINITIONS
        assert "first_customer" in MILESTONE_DEFINITIONS
        assert "mvp_complete" in MILESTONE_DEFINITIONS


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
