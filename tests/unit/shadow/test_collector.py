"""
TDD Tests for ShadowModeCollector

Tests the 200+ hour data collection service.
"""

import json
import tempfile
import time
from datetime import datetime
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest


class TestShadowModeCollectorExists:
    """RED: ShadowModeCollector class should exist"""

    def test_collector_module_exists(self):
        """RED: agent_ros_bridge.shadow.collector module should exist"""
        try:
            from agent_ros_bridge.shadow.collector import ShadowModeCollector

            assert True
        except ImportError:
            pytest.fail("ShadowModeCollector should be importable")

    def test_collector_class_exists(self):
        """RED: ShadowModeCollector class should exist"""
        from agent_ros_bridge.shadow.collector import ShadowModeCollector

        assert ShadowModeCollector is not None

    def test_can_create_collector(self):
        """RED: Should create collector with config"""
        from agent_ros_bridge.shadow.collector import ShadowModeCollector

        with tempfile.TemporaryDirectory() as tmpdir:
            collector = ShadowModeCollector(
                output_dir=tmpdir,
                target_hours=200.0,
            )
            assert collector.target_hours == 200.0
            assert collector.output_dir == Path(tmpdir)


class TestDataCollection:
    """RED: Should collect AI and human decision data"""

    def test_logs_ai_proposal(self):
        """RED: Should log AI proposal"""
        from agent_ros_bridge.shadow.collector import ShadowModeCollector

        with tempfile.TemporaryDirectory() as tmpdir:
            collector = ShadowModeCollector(output_dir=tmpdir)

            collector.on_ai_proposal(
                robot_id="bot1",
                intent_type="NAVIGATE",
                confidence=0.95,
                entities=[{"type": "LOCATION", "value": "kitchen"}],
                reasoning="User wants to navigate",
            )

            assert collector.stats["ai_proposals"] == 1
            assert collector.stats["total_decisions"] == 1
            assert collector.stats["avg_confidence"] == 0.95

    def test_logs_human_decision(self):
        """RED: Should log human decision"""
        from agent_ros_bridge.shadow.collector import ShadowModeCollector

        with tempfile.TemporaryDirectory() as tmpdir:
            collector = ShadowModeCollector(output_dir=tmpdir)

            collector.on_human_decision(
                robot_id="bot1",
                command="navigate_to",
                parameters={"location": "kitchen"},
                matched_ai_proposal=True,
            )

            assert collector.stats["human_decisions"] == 1
            assert collector.stats["agreements"] == 1
            assert collector.stats["agreement_rate"] == 1.0

    def test_logs_rejection(self):
        """RED: Should log rejection"""
        from agent_ros_bridge.shadow.collector import ShadowModeCollector

        with tempfile.TemporaryDirectory() as tmpdir:
            collector = ShadowModeCollector(output_dir=tmpdir)

            collector.on_rejection(
                robot_id="bot1",
                ai_proposal_id="prop_123",
                reason="unsafe",
            )

            assert collector.stats["rejections"] == 1

    def test_logs_modification(self):
        """RED: Should log modification"""
        from agent_ros_bridge.shadow.collector import ShadowModeCollector

        with tempfile.TemporaryDirectory() as tmpdir:
            collector = ShadowModeCollector(output_dir=tmpdir)

            collector.on_modification(
                robot_id="bot1",
                original={"location": "kitchen"},
                modified={"location": "living_room"},
            )

            assert collector.stats["modifications"] == 1


class TestFileLogging:
    """RED: Should write decisions to log files"""

    def test_creates_jsonl_log_file(self):
        """RED: Should create daily JSONL log file"""
        from agent_ros_bridge.shadow.collector import ShadowModeCollector

        with tempfile.TemporaryDirectory() as tmpdir:
            collector = ShadowModeCollector(output_dir=tmpdir)

            collector.on_ai_proposal(
                robot_id="bot1",
                intent_type="NAVIGATE",
                confidence=0.95,
                entities=[],
            )

            date_str = datetime.now().strftime("%Y-%m-%d")
            log_file = Path(tmpdir) / f"decisions_{date_str}.jsonl"

            assert log_file.exists()

    def test_log_entry_format(self):
        """RED: Log entries should have correct format"""
        from agent_ros_bridge.shadow.collector import ShadowModeCollector

        with tempfile.TemporaryDirectory() as tmpdir:
            collector = ShadowModeCollector(output_dir=tmpdir)

            collector.on_ai_proposal(
                robot_id="bot1",
                intent_type="NAVIGATE",
                confidence=0.95,
                entities=[{"type": "LOCATION", "value": "kitchen"}],
            )

            date_str = datetime.now().strftime("%Y-%m-%d")
            log_file = Path(tmpdir) / f"decisions_{date_str}.jsonl"

            with open(log_file) as f:
                entry = json.loads(f.readline())

            assert "timestamp" in entry
            assert entry["type"] == "ai_proposal"
            assert entry["robot_id"] == "bot1"
            assert entry["intent_type"] == "NAVIGATE"


class TestCheckpointing:
    """RED: Should support checkpoint/resume"""

    def test_saves_checkpoint(self):
        """RED: Should save checkpoint periodically"""
        from agent_ros_bridge.shadow.collector import ShadowModeCollector

        with tempfile.TemporaryDirectory() as tmpdir:
            collector = ShadowModeCollector(output_dir=tmpdir)
            collector.stats["total_decisions"] = 100

            collector._save_checkpoint()

            checkpoint_file = Path(tmpdir) / "checkpoint.json"
            assert checkpoint_file.exists()

    def test_loads_checkpoint(self):
        """RED: Should load previous stats from checkpoint"""
        from agent_ros_bridge.shadow.collector import ShadowModeCollector

        with tempfile.TemporaryDirectory() as tmpdir:
            # Create collector and save checkpoint
            collector1 = ShadowModeCollector(output_dir=tmpdir)
            collector1.stats["total_decisions"] = 500
            collector1._save_checkpoint()

            # Create new collector (should load checkpoint)
            collector2 = ShadowModeCollector(output_dir=tmpdir)

            assert collector2.stats["total_decisions"] == 500


class TestDataExport:
    """RED: Should export data for analysis"""

    def test_exports_json(self):
        """RED: Should export all data as JSON"""
        from agent_ros_bridge.shadow.collector import ShadowModeCollector

        with tempfile.TemporaryDirectory() as tmpdir:
            collector = ShadowModeCollector(output_dir=tmpdir)

            collector.on_ai_proposal("bot1", "NAVIGATE", 0.95, [])
            collector.on_human_decision("bot1", "navigate_to", {}, True)

            export_path = collector.export_data("json")

            assert export_path.exists()
            assert export_path.suffix == ".json"

            with open(export_path) as f:
                data = json.load(f)

            assert "metadata" in data
            assert "decisions" in data
            assert len(data["decisions"]) == 2

    def test_exports_csv(self):
        """RED: Should export data as CSV"""
        from agent_ros_bridge.shadow.collector import ShadowModeCollector

        with tempfile.TemporaryDirectory() as tmpdir:
            collector = ShadowModeCollector(output_dir=tmpdir)

            collector.on_ai_proposal("bot1", "NAVIGATE", 0.95, [])

            export_path = collector.export_data("csv")

            assert export_path.exists()
            assert export_path.suffix == ".csv"


class TestCollectionLoop:
    """RED: Should run continuous collection"""

    def test_starts_collection(self):
        """RED: Should start collection loop"""
        from agent_ros_bridge.shadow.collector import ShadowModeCollector

        with tempfile.TemporaryDirectory() as tmpdir:
            collector = ShadowModeCollector(
                output_dir=tmpdir,
                target_hours=0.001,  # Very short for testing
            )

            collector.start_collection()

            assert collector._running is True
            assert collector.stats["start_time"] is not None

            collector.stop_collection()

    def test_stops_gracefully(self):
        """RED: Should stop collection gracefully"""
        from agent_ros_bridge.shadow.collector import ShadowModeCollector

        with tempfile.TemporaryDirectory() as tmpdir:
            collector = ShadowModeCollector(output_dir=tmpdir)

            collector.start_collection()
            time.sleep(0.1)
            collector.stop_collection()

            assert collector._running is False

    def test_gets_status(self):
        """RED: Should return current status"""
        from agent_ros_bridge.shadow.collector import ShadowModeCollector

        with tempfile.TemporaryDirectory() as tmpdir:
            collector = ShadowModeCollector(output_dir=tmpdir)
            collector.stats["total_decisions"] = 100

            status = collector.get_status()

            assert "total_decisions" in status
            assert status["total_decisions"] == 100
            assert "is_running" in status


class TestTargetHours:
    """RED: Should track 200+ hour target"""

    def test_tracks_elapsed_time(self):
        """RED: Should track elapsed collection time"""
        from agent_ros_bridge.shadow.collector import ShadowModeCollector

        with tempfile.TemporaryDirectory() as tmpdir:
            collector = ShadowModeCollector(
                output_dir=tmpdir,
                target_hours=200.0,
            )

            collector.start_collection()
            time.sleep(0.1)

            status = collector.get_status()

            assert "hours_elapsed" in status
            assert status["hours_elapsed"] > 0

            collector.stop_collection()

    def test_calculates_percent_complete(self):
        """RED: Should calculate percentage to target"""
        from agent_ros_bridge.shadow.collector import ShadowModeCollector

        with tempfile.TemporaryDirectory() as tmpdir:
            collector = ShadowModeCollector(
                output_dir=tmpdir,
                target_hours=10.0,  # Short target for testing
            )

            collector.stats["start_time"] = "2026-03-23T10:00:00"
            # Manually set to simulate 5 hours elapsed
            collector.stats["start_time"] = (
                datetime.now() - __import__("datetime").timedelta(hours=5)
            ).isoformat()

            status = collector.get_status()

            assert "percent_complete" in status
            assert 49.9 <= status["percent_complete"] <= 50.1
