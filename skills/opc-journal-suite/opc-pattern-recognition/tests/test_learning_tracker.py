"""Tests for learning tracker module."""
import unittest
import sys
from pathlib import Path

# Add scripts to path
sys.path.insert(0, str(Path(__file__).parent.parent / "scripts"))

from learning_tracker import LearningTracker, main, LEARNING_THRESHOLDS


class TestLearningTracker(unittest.TestCase):
    """Test learning tracker functionality."""

    def test_record_first_observation(self):
        """Test recording first observation."""
        tracker = LearningTracker("TEST-001")
        
        result = tracker.record_observation(
            category="work_hours",
            observation="late night work",
            context={"hour": 23}
        )
        
        self.assertEqual(result["count"], 1)
        self.assertEqual(result["promotion"], "daily_memory")
        self.assertEqual(result["category"], "work_hours")

    def test_promotion_to_project_memory(self):
        """Test promotion after 3 occurrences."""
        tracker = LearningTracker("TEST-001")
        
        # Record 3 times
        for i in range(3):
            result = tracker.record_observation(
                category="decision_style",
                observation="risk averse",
                context={"scenario": f"test_{i}"}
            )
        
        self.assertEqual(result["count"], 3)
        self.assertEqual(result["promotion"], "project_memory")
        self.assertIn("risk averse", result["recommendation"])

    def test_no_promotion_second_time(self):
        """Test no promotion on second occurrence."""
        tracker = LearningTracker("TEST-001")
        
        tracker.record_observation("test", "value", {})
        result = tracker.record_observation("test", "value", {})
        
        self.assertEqual(result["count"], 2)
        self.assertIsNone(result["promotion"])

    def test_get_promoted_observations(self):
        """Test retrieving promoted observations."""
        tracker = LearningTracker("TEST-001")
        
        # Create some observations
        tracker.record_observation("cat1", "obs1", {})  # daily_memory
        for _ in range(3):
            tracker.record_observation("cat2", "obs2", {})  # project_memory
        
        promoted = tracker.get_promoted_observations()
        
        self.assertEqual(len(promoted), 2)

    def test_get_promoted_by_tier(self):
        """Test filtering by tier."""
        tracker = LearningTracker("TEST-001")
        
        tracker.record_observation("cat1", "obs1", {})  # daily
        for _ in range(3):
            tracker.record_observation("cat2", "obs2", {})  # project
        
        project_only = tracker.get_promoted_observations("project_memory")
        
        self.assertEqual(len(project_only), 1)
        self.assertEqual(project_only[0]["promoted_to"], "project_memory")

    def test_main_record(self):
        """Test main function with record action."""
        context = {
            "customer_id": "TEST-001",
            "input": {
                "action": "record",
                "category": "work_hours",
                "observation": "early bird"
            }
        }
        
        result = main(context)
        
        self.assertEqual(result["status"], "success")
        self.assertEqual(result["result"]["count"], 1)

    def test_main_missing_customer_id(self):
        """Test error handling for missing customer_id."""
        context = {
            "input": {"action": "record", "observation": "test"}
        }
        
        result = main(context)
        
        self.assertEqual(result["status"], "error")

    def test_main_missing_observation(self):
        """Test error handling for missing observation."""
        context = {
            "customer_id": "TEST-001",
            "input": {"action": "record"}
        }
        
        result = main(context)
        
        self.assertEqual(result["status"], "error")

    def test_learning_thresholds(self):
        """Test that thresholds are properly defined."""
        self.assertIn("daily_memory", LEARNING_THRESHOLDS)
        self.assertIn("project_memory", LEARNING_THRESHOLDS)
        self.assertIn("permanent", LEARNING_THRESHOLDS)
        
        self.assertEqual(LEARNING_THRESHOLDS["daily_memory"], 1)
        self.assertEqual(LEARNING_THRESHOLDS["project_memory"], 3)


if __name__ == "__main__":
    unittest.main()
