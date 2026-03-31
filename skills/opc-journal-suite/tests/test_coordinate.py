"""Tests for opc-journal-suite coordination module.

TDD approach: tests define expected behavior.
"""
import pytest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "scripts"))

from coordinate import (
    main as coordinate,
    detect_intent,
    get_skill_for_intent,
    INTENT_PATTERNS
)


class TestDetectIntent:
    """Test intent detection."""
    
    def test_detect_journal_record(self):
        """Should detect journal record intent."""
        intent, confidence = detect_intent("Record a journal entry about today")
        
        assert intent == "journal_record"
        assert confidence > 0.3
    
    def test_detect_journal_init(self):
        """Should detect journal init intent."""
        intent, confidence = detect_intent("Initialize my journal")
        
        assert intent == "journal_init"
    
    def test_detect_pattern_analyze(self):
        """Should detect pattern analysis intent."""
        intent, confidence = detect_intent("Analyze my work habits")
        
        assert intent == "pattern_analyze"
    
    def test_detect_milestone(self):
        """Should detect milestone detection intent."""
        intent, confidence = detect_intent("Detect milestones in my journey")
        
        assert intent == "milestone_detect"
    
    def test_detect_task_create(self):
        """Should detect task creation intent."""
        intent, confidence = detect_intent("Run this in background")
        
        assert intent == "task_create"
    
    def test_detect_task_status(self):
        """Should detect task status check intent."""
        intent, confidence = detect_intent("Check task status")
        
        assert intent == "task_status"
    
    def test_detect_insight(self):
        """Should detect insight generation intent."""
        intent, confidence = detect_intent("Give me advice on what to do")
        
        assert intent == "insight_generate"
    
    def test_detect_daily_summary(self):
        """Should detect daily summary intent."""
        intent, confidence = detect_intent("Generate daily summary")
        
        assert intent == "daily_summary"
    
    def test_detect_chinese_record(self):
        """Should detect Chinese journal record intent."""
        intent, confidence = detect_intent("记录今天的日记")
        
        assert intent == "journal_record"
    
    def test_detect_chinese_analyze(self):
        """Should detect Chinese analysis intent."""
        intent, confidence = detect_intent("分析我的习惯")
        
        assert intent == "pattern_analyze"
    
    def test_detect_unknown(self):
        """Should return unknown for unclear input."""
        intent, confidence = detect_intent("")
        
        assert intent == "unknown"
        assert confidence == 0.0
    
    def test_detect_unclear(self):
        """Should return unknown for unrelated text."""
        intent, confidence = detect_intent("The weather is nice today")
        
        assert intent == "unknown"
        assert confidence == 0.0


class TestGetSkillForIntent:
    """Test intent to skill mapping."""
    
    def test_journal_intents_map_to_core(self):
        """Journal intents should map to opc-journal-core."""
        assert get_skill_for_intent("journal_init") == "opc-journal-core"
        assert get_skill_for_intent("journal_record") == "opc-journal-core"
        assert get_skill_for_intent("journal_search") == "opc-journal-core"
        assert get_skill_for_intent("journal_export") == "opc-journal-core"
    
    def test_pattern_intent_maps_to_recognition(self):
        """Pattern intent should map to opc-pattern-recognition."""
        assert get_skill_for_intent("pattern_analyze") == "opc-pattern-recognition"
    
    def test_milestone_intent_maps_to_tracker(self):
        """Milestone intent should map to opc-milestone-tracker."""
        assert get_skill_for_intent("milestone_detect") == "opc-milestone-tracker"
    
    def test_task_intents_map_to_manager(self):
        """Task intents should map to opc-async-task-manager."""
        assert get_skill_for_intent("task_create") == "opc-async-task-manager"
        assert get_skill_for_intent("task_status") == "opc-async-task-manager"
    
    def test_insight_intents_map_to_generator(self):
        """Insight intents should map to opc-insight-generator."""
        assert get_skill_for_intent("insight_generate") == "opc-insight-generator"
        assert get_skill_for_intent("daily_summary") == "opc-insight-generator"
    
    def test_unknown_intent_returns_none(self):
        """Unknown intent should return None."""
        assert get_skill_for_intent("unknown") is None
        assert get_skill_for_intent("nonexistent") is None


class TestCoordinate:
    """Test main coordination function."""
    
    def test_coordinate_success(self):
        """Should successfully route to sub-skill."""
        context = {
            "customer_id": "OPC-001",
            "input": {"text": "Record my progress today"}
        }
        
        result = coordinate(context)
        
        assert result["status"] == "success"
        assert result["result"]["action"] == "delegate"
        assert result["result"]["delegation"]["target_skill"] == "opc-journal-core"
        assert result["result"]["delegation"]["intent"] == "journal_record"
    
    def test_coordinate_with_explicit_action(self):
        """Should use explicit action if provided."""
        context = {
            "customer_id": "OPC-001",
            "input": {"action": "pattern_analyze", "text": "random text"}
        }
        
        result = coordinate(context)
        
        assert result["status"] == "success"
        assert result["result"]["delegation"]["target_skill"] == "opc-pattern-recognition"
        assert result["result"]["delegation"]["intent"] == "pattern_analyze"
        assert result["result"]["delegation"]["confidence"] == 1.0
    
    def test_coordinate_missing_customer_id(self):
        """Should fail when customer_id is missing."""
        context = {
            "input": {"text": "Record entry"}
        }
        
        result = coordinate(context)
        
        assert result["status"] == "error"
        assert "customer_id is required" in result["message"]
    
    def test_coordinate_unknown_intent(self):
        """Should ask for clarification when intent unclear."""
        context = {
            "customer_id": "OPC-001",
            "input": {"text": "Hello world random text"}
        }
        
        result = coordinate(context)
        
        assert result["status"] == "needs_clarification"
        assert "available_actions" in result["result"]
    
    def test_coordinate_chinese_input(self):
        """Should handle Chinese input."""
        context = {
            "customer_id": "OPC-001",
            "input": {"text": "分析我的工作习惯"}
        }
        
        result = coordinate(context)
        
        assert result["status"] == "success"
        assert result["result"]["delegation"]["intent"] == "pattern_analyze"


class TestIntentPatterns:
    """Test intent pattern definitions."""
    
    def test_all_intents_have_patterns(self):
        """All defined intents should have patterns."""
        for intent, patterns in INTENT_PATTERNS.items():
            assert len(patterns) > 0, f"Intent {intent} has no patterns"
            assert all(isinstance(p, str) for p in patterns)
    
    def test_journal_intents_exist(self):
        """Core journal intents should be defined."""
        assert "journal_init" in INTENT_PATTERNS
        assert "journal_record" in INTENT_PATTERNS
        assert "journal_search" in INTENT_PATTERNS
        assert "journal_export" in INTENT_PATTERNS


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
