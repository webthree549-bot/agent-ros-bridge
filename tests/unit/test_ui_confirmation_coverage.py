"""UI confirmation module coverage tests."""

import pytest
from unittest.mock import Mock, patch, MagicMock, AsyncMock
import asyncio


class TestConfirmationUI:
    """Test ConfirmationUI class comprehensively."""

    def test_ui_initialization(self):
        """Test UI initialization with config."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI, UIConfig

        ui = ConfirmationUI(
            port=8080,
            auto_approve_threshold=0.95,
            require_confirmation=True,
            timeout_seconds=60.0,
        )

        assert ui.port == 8080
        assert ui.config.auto_approve_threshold == 0.95
        assert ui.config.require_confirmation is True
        assert ui.config.timeout_seconds == 60.0

    def test_proposal_creation(self):
        """Test creating and storing proposals."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI, Proposal

        ui = ConfirmationUI()

        proposal_data = {
            "proposal_id": "prop_001",
            "robot_id": "bot1",
            "intent_type": "NAVIGATE",
            "confidence": 0.9,
            "entities": [{"type": "LOCATION", "value": "kitchen"}],
            "reasoning": "User wants to go to kitchen",
        }

        proposal_id = ui.receive_proposal(proposal_data)
        assert proposal_id == "prop_001"
        assert "prop_001" in ui._proposals
        assert ui._proposals["prop_001"].status == "pending"

    def test_auto_approve_high_confidence(self):
        """Test auto-approval for high confidence proposals."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI

        ui = ConfirmationUI(
            auto_approve_threshold=0.9,
            require_confirmation=False,
        )

        proposal_data = {
            "proposal_id": "prop_002",
            "robot_id": "bot1",
            "intent_type": "NAVIGATE",
            "confidence": 0.95,  # Above threshold
            "entities": [],
        }

        proposal_id = ui.receive_proposal(proposal_data)
        # Should be auto-approved
        assert ui._proposals[proposal_id].status == "approved"

    def test_proposal_approval(self):
        """Test manual proposal approval."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI

        ui = ConfirmationUI()

        proposal_data = {
            "proposal_id": "prop_003",
            "robot_id": "bot1",
            "intent_type": "PICKUP",
            "confidence": 0.8,
            "entities": [{"type": "OBJECT", "value": "cup"}],
        }

        ui.receive_proposal(proposal_data)
        result = ui.approve_proposal("prop_003")

        assert result["success"] is True
        assert ui._proposals["prop_003"].status == "approved"

    def test_proposal_rejection(self):
        """Test proposal rejection."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI

        ui = ConfirmationUI()

        proposal_data = {
            "proposal_id": "prop_004",
            "robot_id": "bot1",
            "intent_type": "NAVIGATE",
            "confidence": 0.6,
            "entities": [],
        }

        ui.receive_proposal(proposal_data)
        result = ui.reject_proposal("prop_004", reason="unsafe_path")

        assert result["success"] is True
        assert ui._proposals["prop_004"].status == "rejected"

    def test_proposal_modification(self):
        """Test proposal modification."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI

        ui = ConfirmationUI()

        proposal_data = {
            "proposal_id": "prop_005",
            "robot_id": "bot1",
            "intent_type": "NAVIGATE",
            "confidence": 0.8,
            "entities": [{"type": "LOCATION", "value": "kitchen"}],
        }

        ui.receive_proposal(proposal_data)

        original = {"location": "kitchen"}
        modified = {"location": "living_room"}

        result = ui.modify_proposal("prop_005", original, modified)

        assert result["success"] is True
        assert ui._proposals["prop_005"].status == "modified"

    def test_risk_level_calculation(self):
        """Test risk level calculation."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI

        ui = ConfirmationUI()

        # High risk: emergency stop
        assert ui._calculate_risk_level(0.9, "EMERGENCY_STOP") == "HIGH"

        # High risk: low confidence
        assert ui._calculate_risk_level(0.4, "NAVIGATE") == "HIGH"

        # Medium risk
        assert ui._calculate_risk_level(0.6, "NAVIGATE") == "MEDIUM"

        # Low risk
        assert ui._calculate_risk_level(0.8, "NAVIGATE") == "LOW"

    def test_get_pending_proposals(self):
        """Test retrieving pending proposals."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI

        ui = ConfirmationUI()

        # Add proposals
        for i in range(3):
            ui.receive_proposal({
                "proposal_id": f"prop_{i}",
                "robot_id": "bot1",
                "intent_type": "NAVIGATE",
                "confidence": 0.8,
                "entities": [],
            })

        # Approve one
        ui.approve_proposal("prop_0")

        pending = ui.get_pending_proposals()
        assert len(pending) == 2

    def test_get_metrics(self):
        """Test metrics retrieval."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI

        ui = ConfirmationUI()

        # Add and process proposals
        ui.receive_proposal({
            "proposal_id": "p1",
            "robot_id": "bot1",
            "intent_type": "NAVIGATE",
            "confidence": 0.8,
            "entities": [],
        })
        ui.approve_proposal("p1")

        ui.receive_proposal({
            "proposal_id": "p2",
            "robot_id": "bot1",
            "intent_type": "PICKUP",
            "confidence": 0.7,
            "entities": [],
        })
        ui.reject_proposal("p2", reason="unsafe")

        metrics = ui.get_metrics()
        assert metrics["total_decisions"] >= 2
        assert metrics["approved"] >= 1
        assert metrics["rejected"] >= 1

    def test_proposal_not_found(self):
        """Test handling non-existent proposal."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI

        ui = ConfirmationUI()

        result = ui.approve_proposal("nonexistent")
        assert result["success"] is False
        assert "error" in result

    def test_rejection_reason_required(self):
        """Test that rejection requires reason."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI

        ui = ConfirmationUI()

        ui.receive_proposal({
            "proposal_id": "p1",
            "robot_id": "bot1",
            "intent_type": "NAVIGATE",
            "confidence": 0.5,
            "entities": [],
        })

        with pytest.raises(ValueError, match="Rejection reason is required"):
            ui.reject_proposal("p1", reason="")


class TestConfirmationDialog:
    """Test ConfirmationDialog class."""

    def test_dialog_initialization(self):
        """Test dialog initialization."""
        from agent_ros_bridge.ui.confirmation import ConfirmationDialog

        dialog = ConfirmationDialog(
            title="Confirm Navigation",
            message="Navigate to kitchen?",
            timeout_sec=30.0,
        )

        assert dialog.title == "Confirm Navigation"
        assert dialog.message == "Navigate to kitchen?"
        assert dialog.timeout_sec == 30.0
        assert dialog.status == "pending"

    def test_dialog_approval(self):
        """Test dialog approval."""
        from agent_ros_bridge.ui.confirmation import ConfirmationDialog

        dialog = ConfirmationDialog(
            title="Confirm",
            message="Proceed?",
        )

        dialog.approve()
        assert dialog.status == "approved"

    def test_dialog_rejection(self):
        """Test dialog rejection."""
        from agent_ros_bridge.ui.confirmation import ConfirmationDialog

        dialog = ConfirmationDialog(
            title="Confirm",
            message="Proceed?",
        )

        dialog.reject()
        assert dialog.status == "rejected"

    def test_dialog_timeout(self):
        """Test dialog timeout."""
        from agent_ros_bridge.ui.confirmation import ConfirmationDialog

        dialog = ConfirmationDialog(
            title="Confirm",
            message="Proceed?",
            timeout_sec=1.0,
        )

        dialog.on_timeout()
        assert dialog.status == "timeout"


class TestProposalDataclass:
    """Test Proposal dataclass."""

    def test_proposal_defaults(self):
        """Test Proposal default values."""
        from agent_ros_bridge.ui.confirmation import Proposal

        proposal = Proposal(
            proposal_id="p1",
            robot_id="bot1",
            intent_type="NAVIGATE",
            confidence=0.9,
        )

        assert proposal.status == "pending"
        assert proposal.entities == []
        assert proposal.reasoning == ""

    def test_proposal_status_transitions(self):
        """Test proposal status transitions."""
        from agent_ros_bridge.ui.confirmation import Proposal

        proposal = Proposal(
            proposal_id="p1",
            robot_id="bot1",
            intent_type="NAVIGATE",
            confidence=0.9,
        )

        assert proposal.status == "pending"

        proposal.status = "approved"
        assert proposal.status == "approved"

        proposal.status = "rejected"
        assert proposal.status == "rejected"


class TestUIConfig:
    """Test UIConfig dataclass."""

    def test_config_defaults(self):
        """Test UIConfig default values."""
        from agent_ros_bridge.ui.confirmation import UIConfig

        config = UIConfig()

        assert config.port == 8080
        assert config.auto_approve_threshold == 1.0
        assert config.require_confirmation is True
        assert config.timeout_seconds == 60.0
        assert config.enable_websocket is True
        assert config.show_reasoning is True
        assert config.show_confidence is True

    def test_config_custom_values(self):
        """Test UIConfig with custom values."""
        from agent_ros_bridge.ui.confirmation import UIConfig

        config = UIConfig(
            port=9090,
            auto_approve_threshold=0.95,
            require_confirmation=False,
            timeout_seconds=30.0,
        )

        assert config.port == 9090
        assert config.auto_approve_threshold == 0.95
        assert config.require_confirmation is False
        assert config.timeout_seconds == 30.0


class TestUIDisplay:
    """Test UI display methods."""

    def test_display_proposal_html(self):
        """Test HTML generation for proposal display."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI

        ui = ConfirmationUI()

        proposal = {
            "proposal_id": "p1",
            "robot_id": "bot1",
            "intent_type": "NAVIGATE",
            "confidence": 0.85,
            "entities": [{"type": "LOCATION", "value": "kitchen"}],
            "reasoning": "Navigate to kitchen",
        }

        html = ui.display_proposal(proposal)

        assert "NAVIGATE" in html
        assert "bot1" in html
        assert "85.0%" in html or "85" in html
        assert "kitchen" in html
        assert "btn-approve" in html
        assert "btn-reject" in html

    def test_display_low_confidence_warning(self):
        """Test warning display for low confidence."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI

        ui = ConfirmationUI()

        proposal = {
            "proposal_id": "p1",
            "robot_id": "bot1",
            "intent_type": "NAVIGATE",
            "confidence": 0.5,  # Low confidence
            "entities": [],
            "reasoning": "",
        }

        html = ui.display_proposal(proposal)

        assert "warning" in html.lower() or "⚠️" in html

    def test_render_interface_html(self):
        """Test full interface HTML rendering."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI

        ui = ConfirmationUI()
        html = ui.render_interface()

        assert "<!DOCTYPE html>" in html
        assert "AI Proposal Confirmation" in html
        assert "btn-approve" in html
        assert "btn-reject" in html
        assert "btn-modify" in html


class TestUIAPI:
    """Test UI API methods."""

    def test_api_get_pending(self):
        """Test API get pending proposals."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI

        ui = ConfirmationUI()

        ui.receive_proposal({
            "proposal_id": "p1",
            "robot_id": "bot1",
            "intent_type": "NAVIGATE",
            "confidence": 0.8,
            "entities": [],
        })

        pending = ui.api_get_pending()
        assert len(pending) == 1
        assert pending[0]["proposal_id"] == "p1"

    def test_api_approve(self):
        """Test API approve endpoint."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI

        ui = ConfirmationUI()

        ui.receive_proposal({
            "proposal_id": "p1",
            "robot_id": "bot1",
            "intent_type": "NAVIGATE",
            "confidence": 0.8,
            "entities": [],
        })

        result = ui.api_approve("p1")
        assert result["success"] is True

    def test_api_reject(self):
        """Test API reject endpoint."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI

        ui = ConfirmationUI()

        ui.receive_proposal({
            "proposal_id": "p1",
            "robot_id": "bot1",
            "intent_type": "NAVIGATE",
            "confidence": 0.5,
            "entities": [],
        })

        result = ui.api_reject("p1", reason="unsafe")
        assert result["success"] is True

    def test_api_get_metrics(self):
        """Test API get metrics endpoint."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI

        ui = ConfirmationUI()

        metrics = ui.api_get_metrics()
        assert "total_decisions" in metrics


class TestUIShadowHooks:
    """Test UI integration with shadow hooks."""

    def test_shadow_hooks_logging_on_approve(self):
        """Test that approval logs to shadow hooks."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks

        hooks = Mock(spec=ShadowModeHooks)

        ui = ConfirmationUI(shadow_hooks=hooks)

        ui.receive_proposal({
            "proposal_id": "p1",
            "robot_id": "bot1",
            "intent_type": "NAVIGATE",
            "confidence": 0.8,
            "entities": [],
        })

        ui.approve_proposal("p1")

        # Should log to shadow hooks
        hooks.on_human_command.assert_called_once()

    def test_shadow_hooks_logging_on_reject(self):
        """Test that rejection logs to shadow hooks."""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks

        hooks = Mock(spec=ShadowModeHooks)

        ui = ConfirmationUI(shadow_hooks=hooks)

        ui.receive_proposal({
            "proposal_id": "p1",
            "robot_id": "bot1",
            "intent_type": "NAVIGATE",
            "confidence": 0.5,
            "entities": [],
        })

        ui.reject_proposal("p1", reason="unsafe")

        # Should log rejection
        hooks.on_human_rejected.assert_called_once()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
