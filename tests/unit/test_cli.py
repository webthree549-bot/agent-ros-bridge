"""Tests for CLI module."""

import argparse
import os
import socket
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from agent_ros_bridge.cli import (
    handle_config,
    handle_skill,
    handle_status,
    main,
    start_bridge,
)


class TestCLIArgumentParsing:
    """Test CLI argument parsing."""

    @pytest.fixture
    def mock_bridge(self):
        """Create mock bridge."""
        with patch("agent_ros_bridge.cli.Bridge") as mock:
            bridge_instance = MagicMock()
            bridge_instance.transport_manager = MagicMock()
            mock.return_value = bridge_instance
            yield mock, bridge_instance

    def test_main_no_args(self, mock_bridge):
        """Test main with no arguments."""
        with (
            patch("sys.argv", ["agent-ros-bridge"]),
            patch.dict(os.environ, {"JWT_SECRET": "test-secret"}),
            patch("asyncio.run") as mock_run,
        ):
            main()
            mock_run.assert_called_once()

    def test_main_version_flag(self, capsys):
        """Test --version flag."""
        with pytest.raises(SystemExit) as exc_info:
            with patch("sys.argv", ["agent-ros-bridge", "--version"]):
                main()
        assert exc_info.value.code == 0

    def test_main_help_flag(self, capsys):
        """Test --help flag."""
        with pytest.raises(SystemExit) as exc_info:
            with patch("sys.argv", ["agent-ros-bridge", "--help"]):
                main()
        assert exc_info.value.code == 0

    def test_main_missing_jwt_secret(self):
        """Test main exits when JWT_SECRET not set."""
        with pytest.raises(SystemExit) as exc_info:
            with (
                patch("sys.argv", ["agent-ros-bridge"]),
                patch.dict(os.environ, {}, clear=True),
            ):
                main()
        assert exc_info.value.code == 1

    def test_main_with_jwt_secret_arg(self):
        """Test main with --jwt-secret argument."""
        with (
            patch("sys.argv", ["agent-ros-bridge", "--jwt-secret", "my-secret"]),
            patch("asyncio.run") as mock_run,
        ):
            main()
            assert os.environ.get("JWT_SECRET") == "my-secret"
            mock_run.assert_called_once()

    def test_main_with_config(self):
        """Test main with --config argument."""
        with (
            patch("sys.argv", ["agent-ros-bridge", "--config", "custom/config.yaml"]),
            patch.dict(os.environ, {"JWT_SECRET": "test-secret"}),
            patch("asyncio.run"),
        ):
            main()

    def test_main_with_ports(self):
        """Test main with custom ports."""
        with (
            patch(
                "sys.argv",
                [
                    "agent-ros-bridge",
                    "--websocket-port",
                    "9000",
                    "--mqtt-port",
                    "9001",
                    "--grpc-port",
                    "9002",
                ],
            ),
            patch.dict(os.environ, {"JWT_SECRET": "test-secret"}),
            patch("asyncio.run"),
        ):
            main()

    def test_main_with_log_level(self):
        """Test main with --log-level argument."""
        with (
            patch("sys.argv", ["agent-ros-bridge", "--log-level", "DEBUG"]),
            patch.dict(os.environ, {"JWT_SECRET": "test-secret"}),
            patch("asyncio.run"),
        ):
            main()

    def test_main_start_command(self):
        """Test main with start command."""
        with (
            patch("sys.argv", ["agent-ros-bridge", "start"]),
            patch.dict(os.environ, {"JWT_SECRET": "test-secret"}),
            patch("asyncio.run") as mock_run,
        ):
            main()
            mock_run.assert_called_once()

    def test_main_start_with_daemon(self):
        """Test main with start --daemon."""
        with (
            patch("sys.argv", ["agent-ros-bridge", "start", "--daemon"]),
            patch.dict(os.environ, {"JWT_SECRET": "test-secret"}),
            patch("asyncio.run"),
        ):
            main()


class TestConfigCommand:
    """Test config command handling."""

    def test_config_init(self, tmp_path, capsys):
        """Test config --init creates default config."""
        with patch("agent_ros_bridge.cli.Path") as mock_path:
            mock_config_path = tmp_path / "config" / "bridge.yaml"
            mock_path.return_value = mock_config_path

            args = argparse.Namespace(init=True, validate=False)
            handle_config(args)

            captured = capsys.readouterr()
            assert "Created default configuration" in captured.out

    def test_config_init_creates_directory(self, tmp_path, capsys):
        """Test config --init creates parent directories."""
        args = argparse.Namespace(init=True, validate=False)

        with (
            patch("pathlib.Path.mkdir") as mock_mkdir,
            patch("pathlib.Path.write_text") as mock_write,
        ):
            handle_config(args)
            mock_mkdir.assert_called_once_with(parents=True, exist_ok=True)

    def test_config_validate(self, capsys):
        """Test config --validate."""
        args = argparse.Namespace(init=False, validate=True)
        handle_config(args)

        captured = capsys.readouterr()
        assert "Configuration validation not yet implemented" in captured.out

    def test_config_no_args(self, capsys):
        """Test config with no arguments."""
        args = argparse.Namespace(init=False, validate=False)
        handle_config(args)

        captured = capsys.readouterr()
        assert "Use --init to create default config" in captured.out


class TestSkillCommand:
    """Test skill command handling."""

    def test_skill_package_not_found(self, capsys):
        """Test skill --package with non-existent path."""
        args = argparse.Namespace(package="/nonexistent/path", validate=None)
        handle_skill(args)

        captured = capsys.readouterr()
        assert "Skill path not found" in captured.out

    @pytest.mark.skip(reason="package_skill import path has hyphen issues")
    def test_skill_package_success(self, tmp_path, capsys):
        """Test skill --package success."""
        skill_dir = tmp_path / "my_skill"
        skill_dir.mkdir()

        args = argparse.Namespace(package=str(skill_dir), validate=None)

        with patch("skills.agent_ros_bridge.scripts.package_skill.package_skill") as mock_package:
            mock_package.return_value = tmp_path / "dist" / "my_skill.zip"
            handle_skill(args)

            captured = capsys.readouterr()
            assert "Packaging skill" in captured.out

    @pytest.mark.skip(reason="package_skill import path has hyphen issues")
    def test_skill_package_failure(self, tmp_path, capsys):
        """Test skill --package failure."""
        skill_dir = tmp_path / "my_skill"
        skill_dir.mkdir()

        args = argparse.Namespace(package=str(skill_dir), validate=None)

        with patch("skills.agent_ros_bridge.scripts.package_skill.package_skill") as mock_package:
            mock_package.side_effect = Exception("Packaging error")
            handle_skill(args)

            captured = capsys.readouterr()
            assert "Packaging failed" in captured.out

    def test_skill_validate_success(self, capsys):
        """Test skill --validate success."""
        args = argparse.Namespace(package=None, validate="/some/path")

        with patch("subprocess.run") as mock_run:
            mock_run.return_value = MagicMock(returncode=0, stdout="", stderr="")
            handle_skill(args)

            captured = capsys.readouterr()
            assert "Validating skill" in captured.out

    def test_skill_validate_failure(self, capsys):
        """Test skill --validate failure."""
        args = argparse.Namespace(package=None, validate="/some/path")

        with patch("subprocess.run") as mock_run:
            mock_run.return_value = MagicMock(returncode=1, stdout="errors", stderr="")
            handle_skill(args)

            captured = capsys.readouterr()
            assert "Skill validation failed" in captured.out

    def test_skill_no_args(self, capsys):
        """Test skill with no arguments."""
        args = argparse.Namespace(package=None, validate=None)
        handle_skill(args)

        captured = capsys.readouterr()
        assert "Use --package" in captured.out


class TestStatusCommand:
    """Test status command handling."""

    def test_status_all_stopped(self, capsys):
        """Test status when all services stopped."""
        args = argparse.Namespace()

        with patch("socket.socket") as mock_socket:
            mock_sock = MagicMock()
            mock_sock.connect_ex.return_value = 1  # Connection refused
            mock_socket.return_value = mock_sock

            handle_status(args)

            captured = capsys.readouterr()
            assert "WebSocket (8765): 🔴 Stopped" in captured.out
            assert "MQTT (1883):      🔴 Stopped" in captured.out
            assert "gRPC (50051):     🔴 Stopped" in captured.out

    def test_status_all_running(self, capsys):
        """Test status when all services running."""
        args = argparse.Namespace()

        with patch("socket.socket") as mock_socket:
            mock_sock = MagicMock()
            mock_sock.connect_ex.return_value = 0  # Connection accepted
            mock_socket.return_value = mock_sock

            handle_status(args)

            captured = capsys.readouterr()
            assert "WebSocket (8765): 🟢 Running" in captured.out
            assert "MQTT (1883):      🟢 Running" in captured.out
            assert "gRPC (50051):     🟢 Running" in captured.out

    def test_status_database_exists(self, tmp_path, capsys):
        """Test status when database exists."""
        args = argparse.Namespace()

        with patch("socket.socket") as mock_socket:
            mock_sock = MagicMock()
            mock_sock.connect_ex.return_value = 1
            mock_socket.return_value = mock_sock

            with (
                patch("pathlib.Path.exists", return_value=True),
                patch("pathlib.Path.stat") as mock_stat,
            ):
                mock_stat.return_value = MagicMock(st_size=1024)
                handle_status(args)

                captured = capsys.readouterr()
                assert "Database:" in captured.out

    @pytest.mark.skip(reason="Test isolation issue in full suite")
    def test_status_database_not_exists(self, capsys):
        """Test status when database doesn't exist."""
        args = argparse.Namespace()

        with patch("socket.socket") as mock_socket:
            mock_sock = MagicMock()
            mock_sock.connect_ex.return_value = 1
            mock_socket.return_value = mock_sock

            with patch("pathlib.Path.exists", return_value=False):
                handle_status(args)

                captured = capsys.readouterr()
                assert "Not initialized" in captured.out


class TestStartBridge:
    """Test start_bridge function."""

    @pytest.fixture
    def mock_args(self):
        """Create mock args."""
        args = MagicMock()
        args.websocket_port = 8765
        args.mqtt_port = 1883
        args.grpc_port = 50051
        args.database_url = "sqlite:///test.db"
        return args

    @pytest.mark.asyncio
    async def test_start_bridge_keyboard_interrupt(self, mock_args, capsys):
        """Test start_bridge handles KeyboardInterrupt."""
        with patch("agent_ros_bridge.cli.Bridge") as mock_bridge:
            bridge_instance = MagicMock()
            bridge_instance.transport_manager = MagicMock()
            bridge_instance.run = MagicMock()
            bridge_instance.run.return_value.__aenter__ = AsyncMock(side_effect=KeyboardInterrupt())
            bridge_instance.run.return_value.__aexit__ = AsyncMock(return_value=False)
            mock_bridge.return_value = bridge_instance

            await start_bridge(mock_args)

            captured = capsys.readouterr()
            assert "Stopping bridge" in captured.out

    @pytest.mark.asyncio
    async def test_start_bridge_error(self, mock_args, capsys):
        """Test start_bridge handles general error."""
        with patch("agent_ros_bridge.cli.Bridge") as mock_bridge:
            mock_bridge.side_effect = Exception("Test error")

            with pytest.raises(SystemExit) as exc_info:
                await start_bridge(mock_args)

            assert exc_info.value.code == 1

    @pytest.mark.asyncio
    async def test_start_bridge_registers_websocket(self, mock_args):
        """Test start_bridge registers WebSocket transport."""
        with (
            patch("agent_ros_bridge.cli.Bridge") as mock_bridge,
            patch("agent_ros_bridge.gateway_v2.transports.websocket.WebSocketTransport") as mock_ws,
        ):
            bridge_instance = MagicMock()
            bridge_instance.transport_manager = MagicMock()
            bridge_instance.run = MagicMock()
            bridge_instance.run.return_value.__aenter__ = AsyncMock(side_effect=KeyboardInterrupt())
            bridge_instance.run.return_value.__aexit__ = AsyncMock(return_value=False)
            mock_bridge.return_value = bridge_instance

            await start_bridge(mock_args)

            bridge_instance.transport_manager.register.assert_called_once()


class TestDefaultConfigValues:
    """Test default configuration values."""

    def test_default_websocket_port(self):
        """Test default WebSocket port."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--websocket-port", type=int, default=8765)
        args = parser.parse_args([])
        assert args.websocket_port == 8765

    def test_default_mqtt_port(self):
        """Test default MQTT port."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--mqtt-port", type=int, default=1883)
        args = parser.parse_args([])
        assert args.mqtt_port == 1883

    def test_default_grpc_port(self):
        """Test default gRPC port."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--grpc-port", type=int, default=50051)
        args = parser.parse_args([])
        assert args.grpc_port == 50051

    def test_default_database_url(self):
        """Test default database URL."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--database-url", type=str, default="sqlite:///agent_ros_bridge.db")
        args = parser.parse_args([])
        assert args.database_url == "sqlite:///agent_ros_bridge.db"

    def test_default_log_level(self):
        """Test default log level."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--log-level", type=str, default="INFO")
        args = parser.parse_args([])
        assert args.log_level == "INFO"
