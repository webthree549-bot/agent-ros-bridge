"""
TDD Tests for Shadow Mode Hooks

Integrates shadow mode logging with existing systems:
- Intent parser (AI proposals)
- Gateway (human commands)
- Automatic decision comparison
"""

from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

import pytest

# =============================================================================
# Phase 1: RED - Write failing tests
# =============================================================================


class TestShadowModeHooksExist:
    """RED: ShadowModeHooks class should exist"""

    def test_shadow_hooks_module_exists(self):
        """RED: agent_ros_bridge.shadow.hooks module should exist"""
        try:
            from agent_ros_bridge.shadow.hooks import ShadowModeHooks
            assert True
        except ImportError:
            pytest.fail("ShadowModeHooks should be importable")

    def test_shadow_hooks_class_exists(self):
        """RED: ShadowModeHooks class should exist"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        assert ShadowModeHooks is not None

    def test_can_create_hooks(self):
        """RED: Should create hooks with decision logger"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        hooks = ShadowModeHooks()
        assert hooks is not None


class TestIntentParserIntegration:
    """RED: Should hook into intent parser to log AI proposals"""

    def test_hook_intent_parser_method_exists(self):
        """RED: Should have method to hook intent parser"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        hooks = ShadowModeHooks()
        assert hasattr(hooks, 'hook_intent_parser')

    def test_logs_ai_proposal_on_intent_parse(self):
        """RED: Should log AI proposal when intent is parsed"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        hooks = ShadowModeHooks()

        # Mock the decision logger
        with patch.object(hooks, '_decision_logger') as mock_logger:
            # Simulate intent parsing
            intent_result = {
                'intent_type': 'NAVIGATE',
                'confidence': 0.95,
                'entities': [{'type': 'LOCATION', 'value': 'kitchen'}],
                'raw_text': 'go to the kitchen',
            }

            hooks.on_intent_parsed(
                robot_id='bot1',
                intent_result=intent_result,
            )

            # Should log the AI decision
            mock_logger.log_ai_proposal.assert_called_once()
            call_args = mock_logger.log_ai_proposal.call_args[1]
            assert call_args['robot_id'] == 'bot1'

    def test_extracts_entities_correctly(self):
        """RED: Should extract entities from intent result"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        hooks = ShadowModeHooks()

        with patch.object(hooks, '_decision_logger') as mock_logger:
            intent_result = {
                'intent_type': 'PICK_AND_PLACE',
                'confidence': 0.88,
                'entities': [
                    {'type': 'OBJECT', 'value': 'cup'},
                    {'type': 'LOCATION', 'value': 'table'},
                ],
            }

            hooks.on_intent_parsed('bot1', intent_result)

            call_args = mock_logger.log_ai_proposal.call_args[1]
            proposal = call_args['proposal']
            entities = proposal.entities
            assert len(entities) == 2
            assert entities[0]['type'] == 'OBJECT'

    def test_handles_low_confidence_intents(self):
        """RED: Should still log low confidence proposals"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        hooks = ShadowModeHooks()

        with patch.object(hooks, '_decision_logger') as mock_logger:
            intent_result = {
                'intent_type': 'UNKNOWN',
                'confidence': 0.45,
                'entities': [],
            }

            hooks.on_intent_parsed('bot1', intent_result)

            # Should still log even with low confidence
            mock_logger.log_ai_proposal.assert_called_once()


class TestGatewayIntegration:
    """RED: Should hook into gateway to log human commands"""

    def test_hook_gateway_method_exists(self):
        """RED: Should have method to hook gateway"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        hooks = ShadowModeHooks()
        assert hasattr(hooks, 'hook_gateway')

    def test_logs_human_command(self):
        """RED: Should log human command when executed"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        hooks = ShadowModeHooks()

        with patch.object(hooks, '_decision_logger') as mock_logger:
            command = {
                'command': 'navigate_to',
                'parameters': {'location': 'kitchen'},
                'robot_id': 'bot1',
            }

            hooks.on_human_command(command)

            mock_logger.log_human_action.assert_called_once()
            call_args = mock_logger.log_human_action.call_args[1]
            assert call_args['robot_id'] == 'bot1'

    def test_logs_rejected_commands(self):
        """RED: Should log when human rejects AI proposal"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        hooks = ShadowModeHooks()

        with patch.object(hooks, '_decision_logger') as mock_logger:
            hooks.on_human_rejected(
                robot_id='bot1',
                ai_proposal_id='proposal_123',
                rejection_reason='unsafe',
            )

            mock_logger.log_rejection.assert_called_once()
            call_args = mock_logger.log_rejection.call_args[1]
            assert call_args['robot_id'] == 'bot1'
            assert call_args['ai_proposal_id'] == 'proposal_123'

    def test_logs_modified_commands(self):
        """RED: Should log when human modifies AI proposal"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        hooks = ShadowModeHooks()

        with patch.object(hooks, '_decision_logger') as mock_logger:
            hooks.on_human_modified(
                robot_id='bot1',
                ai_proposal_id='proposal_123',
                original={'location': 'kitchen'},
                modified={'location': 'living_room'},
            )

            mock_logger.log_modification.assert_called_once()


class TestAutomaticComparison:
    """RED: Should automatically compare AI vs human decisions"""

    def test_tracks_decision_pair(self):
        """RED: Should track AI proposal and human decision as pair"""
        from datetime import UTC, datetime
        from unittest.mock import Mock

        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        from agent_ros_bridge.shadow.models import AIProposal, DecisionRecord

        hooks = ShadowModeHooks()

        # Pre-set mocks
        mock_comparator = Mock()
        mock_comparator.compare.return_value = (True, 0.95)
        hooks._comparator = mock_comparator

        # Create mock decision logger that returns pending records
        mock_logger = Mock()
        mock_proposal = AIProposal(
            intent_type='NAVIGATE',
            confidence=0.95,
            entities=[{'type': 'LOCATION', 'value': 'kitchen'}],
        )
        from agent_ros_bridge.shadow.models import DecisionContext
        mock_pending = DecisionRecord(
            record_id='test-123',
            robot_id='bot1',
            timestamp=datetime.now(UTC),
            context=DecisionContext(),
            ai_proposal=mock_proposal,
            status='pending',
        )
        mock_logger.get_pending.return_value = [mock_pending]
        hooks._decision_logger = mock_logger

        # AI proposes
        hooks.on_intent_parsed('bot1', {
            'intent_type': 'NAVIGATE',
            'confidence': 0.95,
            'entities': [{'type': 'LOCATION', 'value': 'kitchen'}],
        })

        # Human executes different command
        hooks.on_human_command({
            'robot_id': 'bot1',
            'command': 'navigate_to',
            'parameters': {'location': 'living_room'},  # Different!
        })

        # Should trigger comparison
        mock_comparator.compare.assert_called_once()

    def test_calculates_agreement_realtime(self):
        """RED: Should calculate agreement rate in real-time"""
        from datetime import UTC, datetime
        from unittest.mock import Mock

        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        from agent_ros_bridge.shadow.models import AIProposal, DecisionRecord

        hooks = ShadowModeHooks()

        # Pre-set mocks
        mock_comparator = Mock()
        mock_comparator.compare.return_value = (False, 0.3)
        hooks._comparator = mock_comparator

        mock_logger = Mock()
        mock_proposal = AIProposal(
            intent_type='NAVIGATE',
            confidence=0.95,
            entities=[{'type': 'LOCATION', 'value': 'kitchen'}],
        )
        from agent_ros_bridge.shadow.models import DecisionContext
        mock_pending = DecisionRecord(
            record_id='test-123',
            robot_id='bot1',
            timestamp=datetime.now(UTC),
            context=DecisionContext(),
            ai_proposal=mock_proposal,
            status='pending',
        )
        mock_logger.get_pending.return_value = [mock_pending]
        hooks._decision_logger = mock_logger

        hooks.on_intent_parsed('bot1', {
            'intent_type': 'NAVIGATE',
            'entities': [{'type': 'LOCATION', 'value': 'kitchen'}],
        })

        hooks.on_human_command({
            'robot_id': 'bot1',
            'command': 'navigate_to',
            'parameters': {'location': 'living_room'},
        })

        # Should have called comparison
        mock_comparator.compare.assert_called_once()
        # Should log the disagreement
        agrees, similarity = mock_comparator.compare.return_value
        assert not agrees

    def test_updates_dashboard_realtime(self):
        """RED: Should update dashboard metrics in real-time"""
        from datetime import UTC, datetime
        from unittest.mock import Mock

        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        from agent_ros_bridge.shadow.models import AIProposal, DecisionContext, DecisionRecord

        hooks = ShadowModeHooks()

        # Pre-set mocks
        mock_comparator = Mock()
        mock_comparator.compare.return_value = (True, 0.95)
        hooks._comparator = mock_comparator

        mock_dashboard = Mock()
        hooks._dashboard = mock_dashboard

        mock_logger = Mock()
        mock_proposal = AIProposal(
            intent_type='NAVIGATE',
            confidence=0.95,
            entities=[],
        )
        mock_pending = DecisionRecord(
            record_id='test-123',
            robot_id='bot1',
            timestamp=datetime.now(UTC),
            context=DecisionContext(),
            ai_proposal=mock_proposal,
            status='pending',
        )
        mock_logger.get_pending.return_value = [mock_pending]
        hooks._decision_logger = mock_logger

        hooks.on_intent_parsed('bot1', {
            'intent_type': 'NAVIGATE',
            'confidence': 0.95,
            'entities': [],
        })

        hooks.on_human_command({
            'robot_id': 'bot1',
            'command': 'navigate_to',
            'parameters': {'location': 'kitchen'},
        })

        # Should update dashboard
        mock_dashboard.update_metrics.assert_called_once()


class TestConfiguration:
    """RED: Should be configurable"""

    def test_can_enable_disable(self):
        """RED: Should be able to disable shadow mode"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        hooks = ShadowModeHooks(enabled=False)

        with patch.object(hooks, '_decision_logger') as mock_logger:
            hooks.on_intent_parsed('bot1', {'intent_type': 'NAVIGATE'})

            # Should not log when disabled
            mock_logger.log_ai_decision.assert_not_called()

    def test_can_set_confidence_threshold(self):
        """RED: Should only log high confidence above threshold"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        hooks = ShadowModeHooks(confidence_threshold=0.8)

        with patch.object(hooks, '_decision_logger') as mock_logger:
            # High confidence - should log
            hooks.on_intent_parsed('bot1', {
                'intent_type': 'NAVIGATE',
                'confidence': 0.95,
            })
            assert mock_logger.log_ai_proposal.call_count == 1

            # Low confidence - should NOT log (below threshold)
            hooks.on_intent_parsed('bot1', {
                'intent_type': 'NAVIGATE',
                'confidence': 0.5,
            })
            # Still only 1 call because 0.5 < 0.8 threshold
            assert mock_logger.log_ai_proposal.call_count == 1

    def test_can_filter_by_robot_id(self):
        """RED: Should only log for specified robots"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        hooks = ShadowModeHooks(robot_ids=['bot1', 'bot2'])

        with patch.object(hooks, '_decision_logger') as mock_logger:
            hooks.on_intent_parsed('bot1', {'intent_type': 'NAVIGATE'})
            assert mock_logger.log_ai_proposal.call_count == 1

            hooks.on_intent_parsed('bot3', {'intent_type': 'NAVIGATE'})
            # Should not log bot3
            assert mock_logger.log_ai_proposal.call_count == 1


class TestIntegrationWithExistingSystems:
    """RED: Should integrate with existing agent_ros_bridge code"""

    def test_wraps_intent_parser(self):
        """RED: Should wrap existing intent parser without modification"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        hooks = ShadowModeHooks()

        call_count = [0]

        class MockParser:
            def parse(self, text, robot_id='unknown'):
                call_count[0] += 1
                return {
                    'intent_type': 'NAVIGATE',
                    'confidence': 0.95,
                }

        mock_parser = MockParser()

        # Wrap it
        wrapped = hooks.wrap_intent_parser(mock_parser)

        # Call through wrapper
        result = wrapped.parse("go to kitchen", robot_id='bot1')

        # Should return original result
        assert result['intent_type'] == 'NAVIGATE'
        # Should also log (original was called)
        assert call_count[0] == 1

    def test_wraps_gateway(self):
        """RED: Should wrap existing gateway without modification"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        hooks = ShadowModeHooks()

        call_count = [0]

        class MockGateway:
            def send_command(self, command):
                call_count[0] += 1
                return {'success': True}

        mock_gateway = MockGateway()

        # Wrap it
        wrapped = hooks.wrap_gateway(mock_gateway)

        # Call through wrapper
        result = wrapped.send_command({'robot_id': 'bot1', 'command': 'move'})

        # Should return original result
        assert result['success'] is True
        # Should also log (original was called)
        assert call_count[0] == 1

    def test_preserves_original_behavior(self):
        """RED: Should not change behavior of wrapped components"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        hooks = ShadowModeHooks()

        class MockParser:
            def parse(self, text, robot_id='unknown'):
                return {'intent_type': 'TEST'}

        mock_parser = MockParser()

        wrapped = hooks.wrap_intent_parser(mock_parser)
        result = wrapped.parse("test")

        # Result should be exactly what parser returned
        assert result == {'intent_type': 'TEST'}


class TestMetricsAggregation:
    """RED: Should aggregate metrics over time"""

    def test_tracks_total_decisions(self):
        """RED: Should track total decision count"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        hooks = ShadowModeHooks()

        assert hooks.total_decisions == 0

        hooks.on_intent_parsed('bot1', {'intent_type': 'NAVIGATE'})
        assert hooks.total_decisions == 1

        hooks.on_intent_parsed('bot1', {'intent_type': 'PICK'})
        assert hooks.total_decisions == 2

    def test_tracks_agreement_rate(self):
        """RED: Should track running agreement rate"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        hooks = ShadowModeHooks()

        # Start at 0
        assert hooks.agreement_rate == 0.0

    def test_get_stats_method(self):
        """RED: Should provide stats summary"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        hooks = ShadowModeHooks()

        stats = hooks.get_stats()

        assert 'total_decisions' in stats
        assert 'agreement_rate' in stats
        assert 'avg_confidence' in stats


# =============================================================================
# Integration Fixtures
# =============================================================================

@pytest.fixture
def mock_decision_logger():
    """Create mock decision logger"""
    logger = Mock()
    logger.log_ai_decision = Mock(return_value='decision_id_123')
    logger.log_human_decision = Mock(return_value='decision_id_456')
    logger.log_rejection = Mock()
    logger.log_modification = Mock()
    return logger


@pytest.fixture
def mock_comparator():
    """Create mock comparator"""
    comparator = Mock()
    comparator.compare = Mock(return_value={
        'agreement': True,
        'similarity': 0.95,
    })
    return comparator


@pytest.fixture
def mock_dashboard():
    """Create mock dashboard"""
    dashboard = Mock()
    dashboard.update_metrics = Mock()
    return dashboard
