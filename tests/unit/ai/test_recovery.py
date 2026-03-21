"""Tests for recovery strategies."""

import asyncio

import pytest

from agent_ros_bridge.ai.recovery import (
    DeviationRecovery,
    ObstacleRecovery,
    RecoveryAction,
    RecoveryStrategy,
    RecoveryStrategyFactory,
    RecoveryStrategyType,
    StuckRecovery,
    TimeoutRecovery,
    recover_from_deviation,
    recover_from_obstacle,
    recover_from_stuck,
    recover_from_timeout,
)


class TestRecoveryStrategyType:
    """Test RecoveryStrategyType enum."""

    def test_enum_values(self):
        """Test that all enum values exist."""
        assert RecoveryStrategyType.BACKUP_AND_REPLAN.value == "backup_and_replan"
        assert RecoveryStrategyType.RELOCALIZE.value == "relocalize"
        assert RecoveryStrategyType.WAIT_AND_REPLAN.value == "wait_and_replan"
        assert RecoveryStrategyType.ABORT.value == "abort"
        assert RecoveryStrategyType.RETRY.value == "retry"
        assert RecoveryStrategyType.EMERGENCY_STOP.value == "emergency_stop"


class TestRecoveryStrategy:
    """Test RecoveryStrategy dataclass."""

    def test_default_creation(self):
        """Test creating RecoveryStrategy with defaults."""
        strategy = RecoveryStrategy(
            strategy_type=RecoveryStrategyType.BACKUP_AND_REPLAN,
            description="Test strategy",
        )
        assert strategy.strategy_type == RecoveryStrategyType.BACKUP_AND_REPLAN
        assert strategy.description == "Test strategy"
        assert strategy.max_attempts == 3
        assert strategy.timeout == 30.0
        assert strategy.parameters == {}

    def test_custom_creation(self):
        """Test creating RecoveryStrategy with custom values."""
        strategy = RecoveryStrategy(
            strategy_type=RecoveryStrategyType.ABORT,
            description="Custom abort",
            max_attempts=5,
            timeout=60.0,
            parameters={"notify": True},
        )
        assert strategy.max_attempts == 5
        assert strategy.timeout == 60.0
        assert strategy.parameters == {"notify": True}


class TestRecoveryAction:
    """Test RecoveryAction dataclass."""

    def test_default_creation(self):
        """Test creating RecoveryAction with defaults."""
        action = RecoveryAction(
            action_type="backup",
            description="Back up 0.5m",
        )
        assert action.action_type == "backup"
        assert action.description == "Back up 0.5m"
        assert action.parameters == {}

    def test_with_parameters(self):
        """Test creating RecoveryAction with parameters."""
        action = RecoveryAction(
            action_type="replan",
            description="Replan path",
            parameters={"algorithm": "astar", "timeout": 10.0},
        )
        assert action.parameters == {"algorithm": "astar", "timeout": 10.0}


class TestStuckRecovery:
    """Test StuckRecovery class."""

    def test_default_initialization(self):
        """Test default initialization."""
        recovery = StuckRecovery()
        assert recovery.backup_distance == 0.5

    def test_custom_initialization(self):
        """Test initialization with custom distance."""
        recovery = StuckRecovery(backup_distance=1.0)
        assert recovery.backup_distance == 1.0

    @pytest.mark.asyncio
    async def test_execute_success(self):
        """Test successful stuck recovery execution."""
        recovery = StuckRecovery(backup_distance=0.5)
        result = await recovery.execute()

        assert result["success"] is True
        assert "backed_up" in result["action"]
        assert result["backup_distance"] == 0.5

    @pytest.mark.asyncio
    async def test_stop_motion(self):
        """Test _stop_motion method."""
        recovery = StuckRecovery()
        await recovery._stop_motion()  # Should complete without error

    @pytest.mark.asyncio
    async def test_backup(self):
        """Test _backup method."""
        recovery = StuckRecovery()
        result = await recovery._backup()
        assert result is True

    @pytest.mark.asyncio
    async def test_replan(self):
        """Test _replan method."""
        recovery = StuckRecovery()
        result = await recovery._replan()
        assert result is True


class TestDeviationRecovery:
    """Test DeviationRecovery class."""

    def test_default_initialization(self):
        """Test default initialization."""
        recovery = DeviationRecovery()
        assert recovery.relocalization_method == "amcl"

    def test_custom_initialization(self):
        """Test initialization with custom method."""
        recovery = DeviationRecovery(relocalization_method="gps")
        assert recovery.relocalization_method == "gps"

    @pytest.mark.asyncio
    async def test_execute_success(self):
        """Test successful deviation recovery execution."""
        recovery = DeviationRecovery(relocalization_method="amcl")
        result = await recovery.execute()

        assert result["success"] is True
        assert result["action"] == "relocalized_and_resumed"
        assert result["method"] == "amcl"

    @pytest.mark.asyncio
    async def test_relocalize(self):
        """Test _relocalize method."""
        recovery = DeviationRecovery()
        result = await recovery._relocalize()
        assert result is True

    @pytest.mark.asyncio
    async def test_resume(self):
        """Test _resume method."""
        recovery = DeviationRecovery()
        result = await recovery._resume()
        assert result is True


class TestObstacleRecovery:
    """Test ObstacleRecovery class."""

    def test_default_initialization(self):
        """Test default initialization."""
        recovery = ObstacleRecovery()
        assert recovery.wait_timeout == 10.0

    def test_custom_initialization(self):
        """Test initialization with custom timeout."""
        recovery = ObstacleRecovery(wait_timeout=5.0)
        assert recovery.wait_timeout == 5.0

    @pytest.mark.asyncio
    async def test_execute_replans_around(self):
        """Test obstacle recovery when obstacle doesn't clear."""
        recovery = ObstacleRecovery(wait_timeout=0.01)
        result = await recovery.execute()

        # Obstacle doesn't clear in test, so it should replan
        assert result["success"] is True
        assert "replanned" in result["action"]

    @pytest.mark.asyncio
    async def test_wait_for_clearance(self):
        """Test _wait_for_clearance method."""
        recovery = ObstacleRecovery()
        result = await recovery._wait_for_clearance()
        # In test simulation, obstacle doesn't clear
        assert result is False

    @pytest.mark.asyncio
    async def test_replan_around(self):
        """Test _replan_around method."""
        recovery = ObstacleRecovery()
        result = await recovery._replan_around()
        assert result is True


class TestTimeoutRecovery:
    """Test TimeoutRecovery class."""

    def test_default_initialization(self):
        """Test default initialization."""
        recovery = TimeoutRecovery()
        assert recovery.notify_operator is True

    def test_custom_initialization(self):
        """Test initialization with custom notify setting."""
        recovery = TimeoutRecovery(notify_operator=False)
        assert recovery.notify_operator is False

    @pytest.mark.asyncio
    async def test_execute_with_notification(self):
        """Test timeout recovery with operator notification."""
        recovery = TimeoutRecovery(notify_operator=True)
        result = await recovery.execute()

        assert result["success"] is True
        assert result["action"] == "aborted_and_notified"
        assert result["notified"] is True

    @pytest.mark.asyncio
    async def test_execute_without_notification(self):
        """Test timeout recovery without operator notification."""
        recovery = TimeoutRecovery(notify_operator=False)
        result = await recovery.execute()

        assert result["success"] is True
        assert result["action"] == "aborted_and_notified"
        assert result["notified"] is False

    @pytest.mark.asyncio
    async def test_abort(self):
        """Test _abort method."""
        recovery = TimeoutRecovery()
        await recovery._abort()  # Should complete without error

    @pytest.mark.asyncio
    async def test_notify(self):
        """Test _notify method."""
        recovery = TimeoutRecovery()
        await recovery._notify()  # Should complete without error


class TestRecoveryStrategyFactory:
    """Test RecoveryStrategyFactory class."""

    def test_create_stuck_recovery(self):
        """Test creating stuck recovery."""
        recovery = RecoveryStrategyFactory.create_stuck_recovery()
        assert isinstance(recovery, StuckRecovery)
        assert recovery.backup_distance == 0.5

        custom = RecoveryStrategyFactory.create_stuck_recovery(backup_distance=1.0)
        assert custom.backup_distance == 1.0

    def test_create_deviation_recovery(self):
        """Test creating deviation recovery."""
        recovery = RecoveryStrategyFactory.create_deviation_recovery()
        assert isinstance(recovery, DeviationRecovery)
        assert recovery.relocalization_method == "amcl"

        custom = RecoveryStrategyFactory.create_deviation_recovery(relocalization_method="gps")
        assert custom.relocalization_method == "gps"

    def test_create_obstacle_recovery(self):
        """Test creating obstacle recovery."""
        recovery = RecoveryStrategyFactory.create_obstacle_recovery()
        assert isinstance(recovery, ObstacleRecovery)
        assert recovery.wait_timeout == 10.0

        custom = RecoveryStrategyFactory.create_obstacle_recovery(wait_timeout=5.0)
        assert custom.wait_timeout == 5.0

    def test_create_timeout_recovery(self):
        """Test creating timeout recovery."""
        recovery = RecoveryStrategyFactory.create_timeout_recovery()
        assert isinstance(recovery, TimeoutRecovery)
        assert recovery.notify_operator is True

        custom = RecoveryStrategyFactory.create_timeout_recovery(notify_operator=False)
        assert custom.notify_operator is False


class TestConvenienceFunctions:
    """Test convenience recovery functions."""

    @pytest.mark.asyncio
    async def test_recover_from_stuck(self):
        """Test recover_from_stuck convenience function."""
        result = await recover_from_stuck(backup_distance=0.5)
        assert result["success"] is True
        assert "backed_up" in result["action"]

    @pytest.mark.asyncio
    async def test_recover_from_deviation(self):
        """Test recover_from_deviation convenience function."""
        result = await recover_from_deviation(relocalization_method="amcl")
        assert result["success"] is True
        assert result["action"] == "relocalized_and_resumed"

    @pytest.mark.asyncio
    async def test_recover_from_obstacle(self):
        """Test recover_from_obstacle convenience function."""
        result = await recover_from_obstacle(wait_timeout=0.01)
        assert result["success"] is True

    @pytest.mark.asyncio
    async def test_recover_from_timeout(self):
        """Test recover_from_timeout convenience function."""
        result = await recover_from_timeout(notify_operator=True)
        assert result["success"] is True
        assert result["notified"] is True
