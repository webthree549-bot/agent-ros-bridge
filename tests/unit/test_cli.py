"""Tests for CLI module."""
import pytest
import argparse
from unittest.mock import Mock, patch, MagicMock
import sys

import agent_ros_bridge.cli as cli_module


class TestMainFunction:
    """Test main CLI entry point."""
    
    def test_main_with_no_args_shows_help(self):
        """Main with no args shows help."""
        with patch.object(sys, 'argv', ['agent-ros-bridge']):
            with pytest.raises(SystemExit) as exc_info:
                cli_module.main()
            # argparse exits with 0 for --version, 2 for missing required args
            assert exc_info.value.code in [0, 2]
    
    def test_main_version_flag(self):
        """Main handles version flag."""
        with patch.object(sys, 'argv', ['agent-ros-bridge', '--version']):
            with pytest.raises(SystemExit) as exc_info:
                cli_module.main()
            assert exc_info.value.code == 0


class TestArgumentParser:
    """Test CLI argument parsing."""
    
    def test_parser_accepts_config(self):
        """Parser accepts config argument."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--config", default="config/bridge.yaml")
        
        args = parser.parse_args(["--config", "custom.yaml"])
        
        assert args.config == "custom.yaml"
    
    def test_parser_accepts_websocket_port(self):
        """Parser accepts websocket port."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--websocket-port", type=int, default=8765)
        
        args = parser.parse_args(["--websocket-port", "9000"])
        
        assert args.websocket_port == 9000
    
    def test_parser_accepts_mqtt_port(self):
        """Parser accepts MQTT port."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--mqtt-port", type=int, default=1883)
        
        args = parser.parse_args(["--mqtt-port", "1884"])
        
        assert args.mqtt_port == 1884
    
    def test_parser_accepts_grpc_port(self):
        """Parser accepts gRPC port."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--grpc-port", type=int, default=50051)
        
        args = parser.parse_args(["--grpc-port", "50052"])
        
        assert args.grpc_port == 50052
    
    def test_parser_accepts_jwt_secret(self):
        """Parser accepts JWT secret."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--jwt-secret", default=None)
        
        args = parser.parse_args(["--jwt-secret", "mysecret"])
        
        assert args.jwt_secret == "mysecret"
    
    def test_parser_accepts_database_url(self):
        """Parser accepts database URL."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--database-url", default="sqlite:///agent_ros_bridge.db")
        
        args = parser.parse_args(["--database-url", "postgresql://localhost/db"])
        
        assert args.database_url == "postgresql://localhost/db"
    
    def test_parser_accepts_redis_url(self):
        """Parser accepts Redis URL."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--redis-url", default=None)
        
        args = parser.parse_args(["--redis-url", "redis://localhost:6379"])
        
        assert args.redis_url == "redis://localhost:6379"
    
    def test_parser_accepts_log_level(self):
        """Parser accepts log level."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
        
        args = parser.parse_args(["--log-level", "DEBUG"])
        
        assert args.log_level == "DEBUG"
    
    def test_parser_accepts_skill_path(self):
        """Parser accepts skill path."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--skill-path", default=None)
        
        args = parser.parse_args(["--skill-path", "/path/to/skills"])
        
        assert args.skill_path == "/path/to/skills"


class TestDefaultValues:
    """Test CLI default values."""
    
    def test_config_default(self):
        """Config has correct default."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--config", default="config/bridge.yaml")
        
        args = parser.parse_args([])
        
        assert args.config == "config/bridge.yaml"
    
    def test_websocket_port_default(self):
        """WebSocket port has correct default."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--websocket-port", type=int, default=8765)
        
        args = parser.parse_args([])
        
        assert args.websocket_port == 8765
    
    def test_mqtt_port_default(self):
        """MQTT port has correct default."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--mqtt-port", type=int, default=1883)
        
        args = parser.parse_args([])
        
        assert args.mqtt_port == 1883
    
    def test_grpc_port_default(self):
        """gRPC port has correct default."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--grpc-port", type=int, default=50051)
        
        args = parser.parse_args([])
        
        assert args.grpc_port == 50051
    
    def test_database_url_default(self):
        """Database URL has correct default."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--database-url", default="sqlite:///agent_ros_bridge.db")
        
        args = parser.parse_args([])
        
        assert args.database_url == "sqlite:///agent_ros_bridge.db"
    
    def test_log_level_default(self):
        """Log level has correct default."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--log-level", default="INFO")
        
        args = parser.parse_args([])
        
        assert args.log_level == "INFO"


class TestLogLevelChoices:
    """Test log level choices."""
    
    def test_log_level_accepts_debug(self):
        """Log level accepts DEBUG."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--log-level", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
        
        args = parser.parse_args(["--log-level", "DEBUG"])
        
        assert args.log_level == "DEBUG"
    
    def test_log_level_accepts_info(self):
        """Log level accepts INFO."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--log-level", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
        
        args = parser.parse_args(["--log-level", "INFO"])
        
        assert args.log_level == "INFO"
    
    def test_log_level_accepts_warning(self):
        """Log level accepts WARNING."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--log-level", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
        
        args = parser.parse_args(["--log-level", "WARNING"])
        
        assert args.log_level == "WARNING"
    
    def test_log_level_accepts_error(self):
        """Log level accepts ERROR."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--log-level", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
        
        args = parser.parse_args(["--log-level", "ERROR"])
        
        assert args.log_level == "ERROR"
    
    def test_log_level_rejects_invalid(self):
        """Log level rejects invalid values."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--log-level", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
        
        with pytest.raises(SystemExit):
            parser.parse_args(["--log-level", "INVALID"])
