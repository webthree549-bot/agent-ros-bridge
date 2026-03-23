"""
TDD Tests for Human Confirmation UI

Web interface for operators to approve/reject AI suggestions.
Integrates with ShadowModeHooks for real-time decision tracking.
"""

import json
from unittest.mock import MagicMock, Mock, patch

import pytest

# =============================================================================
# Phase 1: RED - Write failing tests
# =============================================================================


class TestConfirmationUIExists:
    """RED: ConfirmationUI class should exist"""

    def test_confirmation_ui_module_exists(self):
        """RED: agent_ros_bridge.ui.confirmation module should exist"""
        try:
            from agent_ros_bridge.ui.confirmation import ConfirmationUI
            assert True
        except ImportError:
            pytest.fail("ConfirmationUI should be importable")

    def test_confirmation_ui_class_exists(self):
        """RED: ConfirmationUI class should exist"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        assert ConfirmationUI is not None

    def test_can_create_ui(self):
        """RED: Should create UI with config"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI(port=8080)
        assert ui is not None
        assert ui.port == 8080


class TestProposalDisplay:
    """RED: Should display AI proposals to operators"""

    def test_display_proposal_method_exists(self):
        """RED: Should have method to display proposals"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()
        assert hasattr(ui, 'display_proposal')

    def test_displays_intent_type(self):
        """RED: Should show intent type"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()

        proposal = {
            'proposal_id': 'prop_123',
            'robot_id': 'bot1',
            'intent_type': 'NAVIGATE',
            'confidence': 0.95,
            'entities': [{'type': 'LOCATION', 'value': 'kitchen'}],
            'reasoning': 'User wants to go to kitchen',
        }

        html = ui.display_proposal(proposal)

        assert 'NAVIGATE' in html
        assert 'bot1' in html

    def test_displays_confidence(self):
        """RED: Should show confidence score"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()

        proposal = {
            'proposal_id': 'prop_123',
            'robot_id': 'bot1',
            'intent_type': 'NAVIGATE',
            'confidence': 0.95,
        }

        html = ui.display_proposal(proposal)

        # Accept various formats: 95.0%, 95%, 0.95, etc.
        assert '95' in html or '0.95' in html

    def test_displays_entities(self):
        """RED: Should show extracted entities"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()

        proposal = {
            'proposal_id': 'prop_123',
            'robot_id': 'bot1',
            'intent_type': 'NAVIGATE',
            'confidence': 0.95,
            'entities': [
                {'type': 'LOCATION', 'value': 'kitchen'},
                {'type': 'OBJECT', 'value': 'cup'},
            ],
        }

        html = ui.display_proposal(proposal)

        assert 'kitchen' in html
        assert 'LOCATION' in html

    def test_displays_reasoning(self):
        """RED: Should show AI reasoning"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()

        proposal = {
            'proposal_id': 'prop_123',
            'robot_id': 'bot1',
            'intent_type': 'NAVIGATE',
            'confidence': 0.95,
            'reasoning': 'Parsed "go to kitchen" as navigation command',
        }

        html = ui.display_proposal(proposal)

        assert 'Parsed' in html or 'reasoning' in html.lower()


class TestOperatorActions:
    """RED: Should handle operator approve/reject/modify actions"""

    def test_approve_method_exists(self):
        """RED: Should have approve method"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()
        assert hasattr(ui, 'approve_proposal')

    def test_approve_executes_command(self):
        """RED: Approve should execute the AI proposal"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI, Proposal
        ui = ConfirmationUI()

        # Pre-populate a proposal
        ui._proposals['prop_123'] = Proposal(
            proposal_id='prop_123',
            robot_id='bot1',
            intent_type='NAVIGATE',
            confidence=0.95,
        )

        exec_called = [False]
        original_exec = ui._execute_command

        def tracking_exec(proposal, modified_params=None):
            exec_called[0] = True
            return {'executed': True}

        ui._execute_command = tracking_exec

        ui.approve_proposal('prop_123')

        assert exec_called[0], "_execute_command should be called on approve"

    def test_reject_method_exists(self):
        """RED: Should have reject method"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()
        assert hasattr(ui, 'reject_proposal')

    def test_reject_logs_reason(self):
        """RED: Reject should log rejection reason"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI, Proposal
        ui = ConfirmationUI()

        # Pre-populate a proposal
        ui._proposals['prop_123'] = Proposal(
            proposal_id='prop_123',
            robot_id='bot1',
            intent_type='NAVIGATE',
            confidence=0.95,
        )

        # Create mock shadow hooks
        mock_hooks = Mock()
        ui._shadow_hooks = mock_hooks

        ui.reject_proposal('prop_123', reason='unsafe')

        mock_hooks.on_human_rejected.assert_called_once()
        call_args = mock_hooks.on_human_rejected.call_args[1]
        assert call_args['rejection_reason'] == 'unsafe'

    def test_modify_method_exists(self):
        """RED: Should have modify method"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()
        assert hasattr(ui, 'modify_proposal')

    def test_modify_executes_modified_command(self):
        """RED: Modify should execute with changed parameters"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI, Proposal
        ui = ConfirmationUI()

        # Pre-populate a proposal
        ui._proposals['prop_123'] = Proposal(
            proposal_id='prop_123',
            robot_id='bot1',
            intent_type='NAVIGATE',
            confidence=0.95,
        )

        original = {'location': 'kitchen'}
        modified = {'location': 'living_room'}

        exec_called = [False]

        def tracking_exec(proposal, modified_params=None):
            exec_called[0] = True
            return {'executed': True}

        ui._execute_command = tracking_exec

        with patch.object(ui, '_shadow_hooks') as mock_hooks:
            ui.modify_proposal('prop_123', original, modified)

            assert exec_called[0], "_execute_command should be called on modify"
            mock_hooks.on_human_modified.assert_called_once()


class TestWebServer:
    """RED: Should serve web interface"""

    def test_start_server_method_exists(self):
        """RED: Should have method to start web server"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()
        assert hasattr(ui, 'start_server')

    def test_serves_html_interface(self):
        """RED: Should serve HTML confirmation interface"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()

        html = ui.render_interface()

        assert '<html>' in html.lower()
        assert 'confirm' in html.lower() or 'approve' in html.lower()

    def test_has_approve_button(self):
        """RED: Interface should have approve button"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()

        html = ui.render_interface()

        assert 'approve' in html.lower() or 'confirm' in html.lower()

    def test_has_reject_button(self):
        """RED: Interface should have reject button"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()

        html = ui.render_interface()

        assert 'reject' in html.lower() or 'deny' in html.lower()

    def test_has_modify_form(self):
        """RED: Interface should have modify form"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()

        html = ui.render_interface()

        assert 'modify' in html.lower() or 'edit' in html.lower() or 'change' in html.lower()


class TestAPIEndpoints:
    """RED: Should expose REST API for frontend"""

    def test_get_pending_proposals_endpoint(self):
        """RED: Should have endpoint to get pending proposals"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()

        with patch.object(ui, 'get_pending_proposals') as mock_get:
            mock_get.return_value = [
                {'proposal_id': 'prop_1', 'intent_type': 'NAVIGATE'},
            ]

            proposals = ui.api_get_pending()

            assert len(proposals) == 1
            assert proposals[0]['proposal_id'] == 'prop_1'

    def test_post_approve_endpoint(self):
        """RED: Should have endpoint to approve"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI, Proposal
        ui = ConfirmationUI()

        # Pre-populate a proposal
        ui._proposals['prop_123'] = Proposal(
            proposal_id='prop_123',
            robot_id='bot1',
            intent_type='NAVIGATE',
            confidence=0.95,
        )

        result = ui.api_approve('prop_123')

        assert result['success'] is True

    def test_post_reject_endpoint(self):
        """RED: Should have endpoint to reject"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI, Proposal
        ui = ConfirmationUI()

        # Pre-populate a proposal
        ui._proposals['prop_123'] = Proposal(
            proposal_id='prop_123',
            robot_id='bot1',
            intent_type='NAVIGATE',
            confidence=0.95,
        )

        result = ui.api_reject('prop_123', reason='unsafe')

        assert result['success'] is True

    def test_get_metrics_endpoint(self):
        """RED: Should have endpoint to get metrics"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()

        # Add some proposals to get metrics
        from agent_ros_bridge.ui.confirmation import Proposal
        ui._proposals['prop_1'] = Proposal(
            proposal_id='prop_1', robot_id='bot1', 
            intent_type='NAVIGATE', confidence=0.95, status='approved'
        )

        metrics = ui.api_get_metrics()

        assert 'total_decisions' in metrics


class TestIntegrationWithShadowMode:
    """RED: Should integrate with ShadowModeHooks"""

    def test_receives_proposals_from_shadow_hooks(self):
        """RED: Should auto-display proposals from shadow mode"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI

        ui = ConfirmationUI()

        # Simulate receiving a proposal directly
        ui.receive_proposal({
            'proposal_id': 'prop_001',
            'robot_id': 'bot1',
            'intent_type': 'NAVIGATE',
            'confidence': 0.95,
            'entities': [{'type': 'LOCATION', 'value': 'kitchen'}],
        })

        # UI should have the proposal
        pending = ui.get_pending_proposals()
        assert len(pending) == 1
        assert pending[0]['intent_type'] == 'NAVIGATE'

    def test_logs_human_decisions_to_shadow_hooks(self):
        """RED: Should log operator decisions via shadow hooks"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI, Proposal
        ui = ConfirmationUI()

        # Pre-populate a proposal
        ui._proposals['prop_123'] = Proposal(
            proposal_id='prop_123',
            robot_id='bot1',
            intent_type='NAVIGATE',
            confidence=0.95,
        )

        # Create mock shadow hooks
        mock_hooks = Mock()
        ui._shadow_hooks = mock_hooks

        ui.approve_proposal('prop_123')

        # Should log the human decision
        mock_hooks.on_human_command.assert_called_once()

    def test_updates_metrics_via_shadow_hooks(self):
        """RED: Should get metrics from shadow hooks"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()

        with patch.object(ui, '_shadow_hooks') as mock_hooks:
            mock_hooks.get_stats.return_value = {
                'total_decisions': 50,
                'agreement_rate': 0.90,
            }

            metrics = ui.get_metrics()

            assert metrics['total_decisions'] == 50
            assert metrics['agreement_rate'] == 0.90


class TestConfiguration:
    """RED: Should be configurable"""

    def test_can_set_auto_approve_threshold(self):
        """RED: Should auto-approve high confidence proposals"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        
        # Create UI with auto-approve threshold
        ui = ConfirmationUI(auto_approve_threshold=0.95, require_confirmation=False)

        # Track if approve was called
        approve_called = [False]
        original_approve = ui.approve_proposal
        
        def tracking_approve(proposal_id):
            approve_called[0] = True
            return {'success': True}
        
        ui.approve_proposal = tracking_approve

        # High confidence - should auto-approve
        proposal = {'proposal_id': 'prop_1', 'confidence': 0.98, 'robot_id': 'bot1', 'intent_type': 'NAVIGATE'}
        ui.receive_proposal(proposal)

        assert approve_called[0], "High confidence proposal should be auto-approved"

    def test_can_require_confirmation(self):
        """RED: Should require confirmation for all proposals"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI(require_confirmation=True)

        # Even high confidence should wait for approval
        with patch.object(ui, 'approve_proposal') as mock_approve:
            proposal = {'proposal_id': 'prop_1', 'confidence': 0.98}
            ui.receive_proposal(proposal)

            mock_approve.assert_not_called()

    def test_can_set_timeout(self):
        """RED: Should timeout proposals after delay"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI(timeout_seconds=30)

        assert ui.timeout_seconds == 30


class TestSafetyFeatures:
    """RED: Should have safety safeguards"""

    def test_shows_safety_warnings(self):
        """RED: Should show warnings for low confidence"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()

        proposal = {
            'proposal_id': 'prop_1',
            'confidence': 0.45,
            'intent_type': 'NAVIGATE',
        }

        html = ui.display_proposal(proposal)

        assert 'warning' in html.lower() or 'low confidence' in html.lower()

    def test_requires_reason_for_reject(self):
        """RED: Should require reason when rejecting"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()

        # Reject without reason should fail
        with pytest.raises(ValueError):
            ui.reject_proposal('prop_123', reason='')

    def test_shows_risk_level(self):
        """RED: Should display risk level"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()

        proposal = {
            'proposal_id': 'prop_1',
            'confidence': 0.95,
            'intent_type': 'EMERGENCY_STOP',
        }

        html = ui.display_proposal(proposal)

        assert 'risk' in html.lower() or 'safety' in html.lower()


class TestRealTimeUpdates:
    """RED: Should support real-time updates"""

    def test_websocket_updates(self):
        """RED: Should push updates via WebSocket"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI
        ui = ConfirmationUI()

        broadcast_called = [False]
        original_broadcast = ui._broadcast_update

        def tracking_broadcast(message):
            broadcast_called[0] = True

        ui._broadcast_update = tracking_broadcast

        ui.receive_proposal({'proposal_id': 'prop_1', 'intent_type': 'NAVIGATE', 'robot_id': 'bot1', 'confidence': 0.95})

        assert broadcast_called[0], "_broadcast_update should be called when receiving proposal"

    def test_live_metrics_update(self):
        """RED: Should update metrics in real-time"""
        from agent_ros_bridge.ui.confirmation import ConfirmationUI, Proposal
        ui = ConfirmationUI()

        # Pre-populate a proposal
        ui._proposals['prop_123'] = Proposal(
            proposal_id='prop_123',
            robot_id='bot1',
            intent_type='NAVIGATE',
            confidence=0.95,
        )

        broadcast_called = [False]
        original_broadcast = ui._broadcast_metrics

        def tracking_broadcast():
            broadcast_called[0] = True

        ui._broadcast_metrics = tracking_broadcast

        ui.approve_proposal('prop_123')

        assert broadcast_called[0], "_broadcast_metrics should be called after approve"


# =============================================================================
# Fixtures
# =============================================================================

@pytest.fixture
def mock_shadow_hooks():
    """Create mock shadow hooks"""
    hooks = Mock()
    hooks.on_intent_parsed = Mock(return_value='decision_id_123')
    hooks.on_human_command = Mock()
    hooks.on_human_rejected = Mock()
    hooks.on_human_modified = Mock()
    hooks.get_stats = Mock(return_value={
        'total_decisions': 0,
        'agreement_rate': 0.0,
    })
    return hooks


@pytest.fixture
def sample_proposal():
    """Create sample proposal for testing"""
    return {
        'proposal_id': 'prop_test_123',
        'robot_id': 'bot1',
        'intent_type': 'NAVIGATE',
        'confidence': 0.95,
        'entities': [{'type': 'LOCATION', 'value': 'kitchen'}],
        'reasoning': 'User wants to navigate to kitchen',
        'timestamp': '2026-03-23T10:00:00Z',
    }
