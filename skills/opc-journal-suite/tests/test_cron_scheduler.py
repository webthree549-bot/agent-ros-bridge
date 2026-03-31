"""Tests for cron scheduler module."""
import unittest
import sys
from pathlib import Path
from datetime import datetime, timedelta

# Add scripts to path
sys.path.insert(0, str(Path(__file__).parent.parent / "scripts"))

from cron_scheduler import main, should_trigger, get_next_trigger, DEFAULT_SCHEDULES


class TestCronScheduler(unittest.TestCase):
    """Test cron scheduler functionality."""

    def test_should_trigger_no_last_run(self):
        """Test trigger check without last run."""
        # If it's past 8 AM, should trigger
        schedule = "0 8 * * *"
        now = datetime.now()
        
        # Test with future time - should not trigger
        if now.hour < 8:
            result = should_trigger(schedule, None)
            self.assertFalse(result)  # Before 8 AM, shouldn't trigger
        else:
            result = should_trigger(schedule, None)
            self.assertTrue(result)  # After 8 AM, should trigger

    def test_should_trigger_already_ran_today(self):
        """Test that task doesn't trigger twice in same day."""
        schedule = "0 8 * * *"
        now = datetime.now()
        last_run = now.replace(hour=9, minute=0).isoformat()
        
        result = should_trigger(schedule, last_run)
        self.assertFalse(result)  # Already ran today

    def test_get_next_trigger_today(self):
        """Test next trigger calculation for today."""
        schedule = "0 8 * * *"  # 8:00 AM
        
        # From 6 AM
        from_time = datetime.now().replace(hour=6, minute=0)
        next_trigger = get_next_trigger(schedule, from_time)
        
        next_dt = datetime.fromisoformat(next_trigger)
        self.assertEqual(next_dt.hour, 8)
        self.assertEqual(next_dt.minute, 0)
        self.assertEqual(next_dt.date(), from_time.date())

    def test_get_next_trigger_tomorrow(self):
        """Test next trigger calculation when time has passed."""
        schedule = "0 8 * * *"  # 8:00 AM
        
        # From 10 AM
        from_time = datetime.now().replace(hour=10, minute=0)
        next_trigger = get_next_trigger(schedule, from_time)
        
        next_dt = datetime.fromisoformat(next_trigger)
        self.assertEqual(next_dt.hour, 8)
        expected_date = from_time.date() + timedelta(days=1)
        self.assertEqual(next_dt.date(), expected_date)

    def test_main_check_triggers(self):
        """Test main function with check_triggers action."""
        context = {
            "customer_id": "TEST-001",
            "input": {"action": "check_triggers"}
        }
        
        result = main(context)
        
        self.assertEqual(result["status"], "success")
        self.assertIn("triggers", result["result"])
        self.assertIn("count", result["result"])
        self.assertIsInstance(result["result"]["triggers"], list)

    def test_main_get_schedule(self):
        """Test main function with get_schedule action."""
        context = {
            "customer_id": "TEST-001",
            "input": {"action": "get_schedule"}
        }
        
        result = main(context)
        
        self.assertEqual(result["status"], "success")
        self.assertIn("schedules", result["result"])
        self.assertIn("daily_summary", result["result"]["schedules"])

    def test_main_update_schedule(self):
        """Test main function with update_schedule action."""
        context = {
            "customer_id": "TEST-001",
            "input": {
                "action": "update_schedule",
                "task_name": "daily_summary",
                "updates": {"enabled": False}
            }
        }
        
        result = main(context)
        
        self.assertEqual(result["status"], "success")
        self.assertEqual(result["result"]["task"], "daily_summary")
        self.assertFalse(result["result"]["updated_config"]["enabled"])

    def test_main_missing_customer_id(self):
        """Test error handling for missing customer_id."""
        context = {
            "input": {"action": "check_triggers"}
        }
        
        result = main(context)
        
        self.assertEqual(result["status"], "error")
        self.assertIn("customer_id", result["message"].lower())

    def test_default_schedules_structure(self):
        """Test that default schedules are properly structured."""
        self.assertIn("daily_summary", DEFAULT_SCHEDULES)
        self.assertIn("weekly_pattern", DEFAULT_SCHEDULES)
        self.assertIn("milestone_check", DEFAULT_SCHEDULES)
        self.assertIn("memory_compaction", DEFAULT_SCHEDULES)
        
        for task_name, config in DEFAULT_SCHEDULES.items():
            self.assertIn("schedule", config)
            self.assertIn("enabled", config)
            self.assertIn("description", config)


if __name__ == "__main__":
    unittest.main()
