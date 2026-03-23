"""
Shadow Mode Data Collection Service

Deploys ShadowModeHooks to collect 200+ hours of AI vs human decision data.
Runs continuously in the background, logging all decisions for analysis.
"""

import json
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path


class ShadowModeCollector:
    """
    Collects shadow mode data over extended periods (200+ hours).

    Features:
    - Continuous background collection
    - Real-time metrics tracking
    - Checkpoint/resume capability
    - Data export for analysis
    - Health monitoring
    """

    def __init__(
        self,
        output_dir: str = "shadow_data",
        target_hours: float = 200.0,
        checkpoint_interval: int = 3600,  # 1 hour
    ):
        """
        Initialize shadow mode collector.

        Args:
            output_dir: Directory to store collected data
            target_hours: Target collection duration (default: 200 hours)
            checkpoint_interval: Seconds between checkpoints
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.target_hours = target_hours
        self.checkpoint_interval = checkpoint_interval

        # Statistics
        self.stats = {
            "start_time": None,
            "total_decisions": 0,
            "ai_proposals": 0,
            "human_decisions": 0,
            "agreements": 0,
            "rejections": 0,
            "modifications": 0,
            "avg_confidence": 0.0,
            "agreement_rate": 0.0,
        }

        # Control
        self._running = False
        self._shutdown_event = threading.Event()
        self._collector_thread: threading.Thread | None = None

        # Load checkpoint if exists
        self._load_checkpoint()

    def _load_checkpoint(self) -> None:
        """Load previous statistics from checkpoint"""
        checkpoint_file = self.output_dir / "checkpoint.json"
        if checkpoint_file.exists():
            with open(checkpoint_file) as f:
                data = json.load(f)
                self.stats.update(data)
                print(f"Loaded checkpoint: {self.stats['total_decisions']} decisions collected")

    def _save_checkpoint(self) -> None:
        """Save current statistics to checkpoint"""
        checkpoint_file = self.output_dir / "checkpoint.json"
        with open(checkpoint_file, "w") as f:
            json.dump(self.stats, f, indent=2, default=str)

    def start_collection(self) -> None:
        """Start continuous data collection"""
        if self._running:
            print("Collection already running")
            return

        self._running = True
        self.stats["start_time"] = datetime.now().isoformat()

        print("Starting shadow mode collection...")
        print(f"Target: {self.target_hours} hours")
        print(f"Output: {self.output_dir}")

        # Start collector thread
        self._collector_thread = threading.Thread(target=self._collection_loop)
        self._collector_thread.daemon = True
        self._collector_thread.start()

        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def stop_collection(self) -> None:
        """Stop data collection gracefully"""
        print("\nStopping collection...")
        self._running = False
        self._shutdown_event.set()

        if self._collector_thread:
            self._collector_thread.join(timeout=5)

        self._save_checkpoint()
        self._print_summary()

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print(f"\nReceived signal {signum}")
        self.stop_collection()
        sys.exit(0)

    def _collection_loop(self) -> None:
        """Main collection loop"""
        last_checkpoint = time.time()

        while self._running and not self._shutdown_event.is_set():
            # Update elapsed time
            if self.stats["start_time"]:
                start = datetime.fromisoformat(self.stats["start_time"])
                elapsed = datetime.now() - start
                hours_elapsed = elapsed.total_seconds() / 3600

                # Check if target reached
                if hours_elapsed >= self.target_hours:
                    print(f"\n✅ Target reached! {hours_elapsed:.1f} hours collected")
                    self.stop_collection()
                    break

                # Print progress every 10 minutes
                if int(time.time()) % 600 == 0:
                    self._print_progress(hours_elapsed)

            # Save checkpoint periodically
            if time.time() - last_checkpoint >= self.checkpoint_interval:
                self._save_checkpoint()
                last_checkpoint = time.time()

            # Sleep briefly
            time.sleep(1)

    def _print_progress(self, hours_elapsed: float) -> None:
        """Print collection progress"""
        pct_complete = (hours_elapsed / self.target_hours) * 100
        print(
            f"Progress: {hours_elapsed:.1f}/{self.target_hours} hours "
            f"({pct_complete:.1f}%) | "
            f"Decisions: {self.stats['total_decisions']} | "
            f"Agreement: {self.stats['agreement_rate']*100:.1f}%"
        )

    def _print_summary(self) -> None:
        """Print collection summary"""
        print("\n" + "=" * 60)
        print("📊 COLLECTION SUMMARY")
        print("=" * 60)
        print(f"Total Decisions: {self.stats['total_decisions']}")
        print(f"AI Proposals: {self.stats['ai_proposals']}")
        print(f"Human Decisions: {self.stats['human_decisions']}")
        print(f"Agreements: {self.stats['agreements']}")
        print(f"Rejections: {self.stats['rejections']}")
        print(f"Modifications: {self.stats['modifications']}")
        print(f"Agreement Rate: {self.stats['agreement_rate']*100:.2f}%")
        print(f"Avg Confidence: {self.stats['avg_confidence']*100:.1f}%")
        if self.stats["start_time"]:
            start = datetime.fromisoformat(self.stats["start_time"])
            elapsed = datetime.now() - start
            print(f"Duration: {elapsed.total_seconds()/3600:.1f} hours")
        print("=" * 60)

    def on_ai_proposal(
        self,
        robot_id: str,
        intent_type: str,
        confidence: float,
        entities: list,
        reasoning: str = "",
    ) -> None:
        """
        Log AI proposal.

        Args:
            robot_id: Robot identifier
            intent_type: Type of intent
            confidence: AI confidence (0-1)
            entities: Extracted entities
            reasoning: AI reasoning
        """
        self.stats["ai_proposals"] += 1
        self.stats["total_decisions"] += 1

        # Update running average of confidence
        n = self.stats["ai_proposals"]
        self.stats["avg_confidence"] = (self.stats["avg_confidence"] * (n - 1) + confidence) / n

        # Log to file
        timestamp = datetime.now().isoformat()
        log_entry = {
            "timestamp": timestamp,
            "type": "ai_proposal",
            "robot_id": robot_id,
            "intent_type": intent_type,
            "confidence": confidence,
            "entities": entities,
            "reasoning": reasoning,
        }

        self._append_log(log_entry)

    def on_human_decision(
        self,
        robot_id: str,
        command: str,
        parameters: dict,
        matched_ai_proposal: bool = False,
    ) -> None:
        """
        Log human decision.

        Args:
            robot_id: Robot identifier
            command: Command executed
            parameters: Command parameters
            matched_ai_proposal: Whether it matched AI proposal
        """
        self.stats["human_decisions"] += 1
        self.stats["total_decisions"] += 1

        if matched_ai_proposal:
            self.stats["agreements"] += 1

        # Recalculate agreement rate
        if self.stats["human_decisions"] > 0:
            self.stats["agreement_rate"] = self.stats["agreements"] / self.stats["human_decisions"]

        # Log to file
        timestamp = datetime.now().isoformat()
        log_entry = {
            "timestamp": timestamp,
            "type": "human_decision",
            "robot_id": robot_id,
            "command": command,
            "parameters": parameters,
            "matched_ai_proposal": matched_ai_proposal,
        }

        self._append_log(log_entry)

    def on_rejection(
        self,
        robot_id: str,
        ai_proposal_id: str,
        reason: str,
    ) -> None:
        """Log human rejection of AI proposal"""
        self.stats["rejections"] += 1

        timestamp = datetime.now().isoformat()
        log_entry = {
            "timestamp": timestamp,
            "type": "rejection",
            "robot_id": robot_id,
            "ai_proposal_id": ai_proposal_id,
            "reason": reason,
        }

        self._append_log(log_entry)

    def on_modification(
        self,
        robot_id: str,
        original: dict,
        modified: dict,
    ) -> None:
        """Log human modification of AI proposal"""
        self.stats["modifications"] += 1

        timestamp = datetime.now().isoformat()
        log_entry = {
            "timestamp": timestamp,
            "type": "modification",
            "robot_id": robot_id,
            "original": original,
            "modified": modified,
        }

        self._append_log(log_entry)

    def _append_log(self, entry: dict) -> None:
        """Append entry to daily log file"""
        date_str = datetime.now().strftime("%Y-%m-%d")
        log_file = self.output_dir / f"decisions_{date_str}.jsonl"

        with open(log_file, "a") as f:
            f.write(json.dumps(entry) + "\n")

    def export_data(self, format: str = "json") -> Path:
        """
        Export collected data for analysis.

        Args:
            format: Export format ('json', 'csv')

        Returns:
            Path to exported file
        """
        if format == "json":
            return self._export_json()
        elif format == "csv":
            return self._export_csv()
        else:
            raise ValueError(f"Unknown format: {format}")

    def _export_json(self) -> Path:
        """Export all data as single JSON file"""
        export_file = self.output_dir / "export_all.json"

        all_decisions = []
        for log_file in sorted(self.output_dir.glob("decisions_*.jsonl")):
            with open(log_file) as f:
                for line in f:
                    all_decisions.append(json.loads(line))

        export_data = {
            "metadata": {
                "export_date": datetime.now().isoformat(),
                "total_decisions": len(all_decisions),
                "collection_stats": self.stats,
            },
            "decisions": all_decisions,
        }

        with open(export_file, "w") as f:
            json.dump(export_data, f, indent=2, default=str)

        print(f"Exported {len(all_decisions)} decisions to {export_file}")
        return export_file

    def _export_csv(self) -> Path:
        """Export data as CSV"""
        import csv

        export_file = self.output_dir / "export_all.csv"

        with open(export_file, "w", newline="") as csvfile:
            fieldnames = [
                "timestamp",
                "type",
                "robot_id",
                "intent_type",
                "confidence",
                "matched",
                "reason",
            ]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            for log_file in sorted(self.output_dir.glob("decisions_*.jsonl")):
                with open(log_file) as f:
                    for line in f:
                        entry = json.loads(line)
                        writer.writerow(
                            {
                                "timestamp": entry.get("timestamp"),
                                "type": entry.get("type"),
                                "robot_id": entry.get("robot_id"),
                                "intent_type": entry.get("intent_type", ""),
                                "confidence": entry.get("confidence", ""),
                                "matched": entry.get("matched_ai_proposal", ""),
                                "reason": entry.get("reason", ""),
                            }
                        )

        print(f"Exported to {export_file}")
        return export_file

    def get_status(self) -> dict:
        """Get current collection status"""
        status = self.stats.copy()

        if self.stats["start_time"]:
            start = datetime.fromisoformat(self.stats["start_time"])
            elapsed = datetime.now() - start
            status["hours_elapsed"] = elapsed.total_seconds() / 3600
            status["hours_remaining"] = max(0, self.target_hours - status["hours_elapsed"])
            status["percent_complete"] = min(
                100, (status["hours_elapsed"] / self.target_hours) * 100
            )

        status["is_running"] = self._running
        return status


# Convenience function
def start_shadow_collection(
    output_dir: str = "shadow_data",
    target_hours: float = 200.0,
) -> ShadowModeCollector:
    """
    Start shadow mode data collection.

    Args:
        output_dir: Output directory
        target_hours: Target collection duration

    Returns:
        ShadowModeCollector instance
    """
    collector = ShadowModeCollector(
        output_dir=output_dir,
        target_hours=target_hours,
    )
    collector.start_collection()
    return collector


if __name__ == "__main__":
    # Example usage
    print("Shadow Mode Data Collection Service")
    print("=" * 60)

    collector = start_shadow_collection(target_hours=0.1)  # 6 min for testing

    # Simulate some data
    for i in range(10):
        collector.on_ai_proposal(
            robot_id="bot1",
            intent_type="NAVIGATE",
            confidence=0.95,
            entities=[{"type": "LOCATION", "value": "kitchen"}],
            reasoning="User wants to go to kitchen",
        )

        collector.on_human_decision(
            robot_id="bot1",
            command="navigate_to",
            parameters={"location": "kitchen"},
            matched_ai_proposal=True,
        )

        time.sleep(1)

    # Wait for target duration
    try:
        while collector._running:
            time.sleep(1)
    except KeyboardInterrupt:
        collector.stop_collection()

    # Export data
    collector.export_data("json")
    collector.export_data("csv")
