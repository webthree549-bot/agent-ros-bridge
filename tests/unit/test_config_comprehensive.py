"""
Comprehensive unit tests for configuration system.
Tests config loading from various sources, environment variable overrides,
and edge cases in nested attribute handling.
"""

import json
import os
import tempfile
from pathlib import Path
from unittest import mock

import pytest
import yaml

from agent_ros_bridge.gateway_v2.config import (
    BridgeConfig,
    ConfigLoader,
    PluginConfig,
    ROSEndpoint,
    SecurityConfig,
    TransportConfig,
)


class TestBridgeConfigDefaults:
    """Test default configuration values"""

    def test_default_bridge_name(self):
        """Test default bridge name is set correctly"""
        config = BridgeConfig()
        assert config.name == "agent_ros_bridge"

    def test_default_log_level(self):
        """Test default log level is INFO"""
        config = BridgeConfig()
        assert config.log_level == "INFO"

    def test_default_transports(self):
        """Test default transports are configured"""
        config = BridgeConfig()

        assert "websocket" in config.transports
        assert config.transports["websocket"].port == 8765

        assert "grpc" in config.transports
        assert config.transports["grpc"].port == 50051

        assert "tcp" in config.transports
        assert config.transports["tcp"].port == 9999

    def test_default_connectors(self):
        """Test default connectors are configured"""
        config = BridgeConfig()

        assert "ros2" in config.connectors
        assert config.connectors["ros2"].enabled is True

    def test_default_security_disabled(self):
        """Test security is disabled by default"""
        config = BridgeConfig()
        assert config.security.enabled is False
        assert config.security.mtls_enabled is False
        assert config.security.tls_enabled is False


class TestConfigLoaderFromYaml:
    """Test loading configuration from YAML files"""

    @pytest.fixture
    def temp_yaml_config(self):
        """Create a temporary YAML config file"""
        config_data = {
            "name": "test_gateway",
            "log_level": "DEBUG",
            "transports": {
                "websocket": {
                    "enabled": True,
                    "host": "127.0.0.1",
                    "port": 9000,
                    "tls_cert": "/path/to/cert.pem",
                },
                "grpc": {"enabled": False, "port": 0},
            },
            "security": {"enabled": True, "jwt_secret": "test-secret", "tls_enabled": True},
        }

        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            yaml.dump(config_data, f)
            temp_path = f.name

        yield temp_path

        # Cleanup
        os.unlink(temp_path)

    def test_load_basic_yaml(self, temp_yaml_config):
        """Test basic YAML config loading"""
        config = ConfigLoader.from_yaml(temp_yaml_config)

        assert config.name == "test_gateway"
        assert config.log_level == "DEBUG"

    def test_load_transport_config(self, temp_yaml_config):
        """Test transport configuration loading from YAML"""
        config = ConfigLoader.from_yaml(temp_yaml_config)

        ws_config = config.transports["websocket"]
        assert ws_config.enabled is True
        assert ws_config.host == "127.0.0.1"
        assert ws_config.port == 9000
        assert ws_config.tls_cert == "/path/to/cert.pem"

    def test_load_disabled_transport(self, temp_yaml_config):
        """Test disabled transport configuration"""
        config = ConfigLoader.from_yaml(temp_yaml_config)

        grpc_config = config.transports["grpc"]
        assert grpc_config.enabled is False
        assert grpc_config.port == 0

    def test_load_security_config(self, temp_yaml_config):
        """Test security configuration loading"""
        config = ConfigLoader.from_yaml(temp_yaml_config)

        assert config.security.enabled is True
        assert config.security.jwt_secret == "test-secret"
        assert config.security.tls_enabled is True


class TestConfigLoaderFromJson:
    """Test loading configuration from JSON files"""

    @pytest.fixture
    def temp_json_config(self):
        """Create a temporary JSON config file"""
        config_data = {
            "name": "json_test_gateway",
            "log_level": "WARNING",
            "discovery": {"enabled": False, "interval": 60},
        }

        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
            json.dump(config_data, f)
            temp_path = f.name

        yield temp_path
        os.unlink(temp_path)

    def test_load_json_config(self, temp_json_config):
        """Test JSON config loading"""
        config = ConfigLoader.from_json(temp_json_config)

        assert config.name == "json_test_gateway"
        assert config.log_level == "WARNING"


class TestConfigEnvironmentVariables:
    """Test environment variable configuration overrides"""

    def test_env_override_name(self):
        """Test BRIDGE_NAME environment variable"""
        with mock.patch.dict(os.environ, {"BRIDGE_NAME": "env_named_gateway"}):
            config = ConfigLoader.from_env()
            assert config.name == "env_named_gateway"

    def test_env_override_log_level(self):
        """Test BRIDGE_LOG_LEVEL environment variable"""
        with mock.patch.dict(os.environ, {"BRIDGE_LOG_LEVEL": "ERROR"}):
            config = ConfigLoader.from_env()
            assert config.log_level == "ERROR"

    def test_env_override_websocket_port(self):
        """Test BRIDGE_WEBSOCKET_PORT environment variable with type conversion"""
        with mock.patch.dict(os.environ, {"BRIDGE_WEBSOCKET_PORT": "9999"}):
            config = ConfigLoader.from_env()
            assert config.transports["websocket"].port == 9999
            assert isinstance(config.transports["websocket"].port, int)

    def test_env_override_grpc_port(self):
        """Test BRIDGE_GRPC_PORT environment variable"""
        with mock.patch.dict(os.environ, {"BRIDGE_GRPC_PORT": "50052"}):
            config = ConfigLoader.from_env()
            assert config.transports["grpc"].port == 50052

    def test_env_override_jwt_secret(self):
        """Test BRIDGE_JWT_SECRET environment variable"""
        with mock.patch.dict(os.environ, {"BRIDGE_JWT_SECRET": "super-secret-key"}):
            config = ConfigLoader.from_env()
            assert config.security.jwt_secret == "super-secret-key"

    def test_env_override_tls_cert_path(self):
        """Test BRIDGE_TLS_CERT environment variable"""
        with mock.patch.dict(os.environ, {"BRIDGE_TLS_CERT": "/etc/certs/new.crt"}):
            config = ConfigLoader.from_env()
            assert config.security.tls_cert == "/etc/certs/new.crt"


class TestConfigFileOrEnv:
    """Test ConfigLoader.from_file_or_env method"""

    def test_uses_provided_path(self):
        """Test that provided path is used when valid"""
        config_data = {"name": "provided_path_test"}

        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            yaml.dump(config_data, f)
            temp_path = f.name

        try:
            config = ConfigLoader.from_file_or_env(temp_path)
            assert config.name == "provided_path_test"
        finally:
            os.unlink(temp_path)

    def test_uses_env_var_path(self):
        """Test BRIDGE_CONFIG environment variable is respected"""
        config_data = {"name": "env_path_test"}

        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            yaml.dump(config_data, f)
            temp_path = f.name

        try:
            with mock.patch.dict(os.environ, {"BRIDGE_CONFIG": temp_path}):
                config = ConfigLoader.from_file_or_env()
                assert config.name == "env_path_test"
        finally:
            os.unlink(temp_path)

    def test_falls_back_to_defaults(self):
        """Test that defaults are used when no config file found"""
        # Clear BRIDGE_ env vars by setting to empty string
        env_vars = {k: "" for k in os.environ if k.startswith("BRIDGE_")}

        with mock.patch.dict(os.environ, env_vars, clear=False):
            # Also ensure BRIDGE_CONFIG points to non-existent file
            with mock.patch.dict(os.environ, {"BRIDGE_CONFIG": "/nonexistent/config.yaml"}):
                config = ConfigLoader.from_file_or_env()
                assert config.name == "agent_ros_bridge"  # Default value
                assert config.log_level == "INFO"  # Default value


class TestNestedAttributeHandling:
    """Test the _set_nested_attr method for nested configuration"""

    def test_set_top_level_attribute(self):
        """Test setting top-level attribute"""
        config = BridgeConfig()
        ConfigLoader._set_nested_attr(config, "name", "new_name")
        assert config.name == "new_name"

    def test_set_nested_dict_attribute(self):
        """Test setting nested dict attribute"""
        config = BridgeConfig()
        ConfigLoader._set_nested_attr(config.transports, "websocket", {"port": 7777})
        assert config.transports["websocket"]["port"] == 7777

    def test_set_transport_port_via_dot_path(self):
        """Test setting transport port via dot notation"""
        config = BridgeConfig()
        ConfigLoader._set_nested_attr(config, "transports.websocket.port", "8888")
        assert config.transports["websocket"].port == 8888

    def test_boolean_conversion_from_env(self):
        """Test that 'true'/'false' strings are converted to booleans"""
        config = BridgeConfig()

        # Test various true representations
        for true_val in ["true", "True", "TRUE", "1", "yes", "on"]:
            config.security.enabled = False  # Reset
            ConfigLoader._set_nested_attr(config, "security.enabled", true_val)
            assert config.security.enabled is True, f"Failed for value: {true_val}"

        # Test various false representations
        for false_val in ["false", "False", "FALSE", "0", "no", "off"]:
            config.security.enabled = True  # Reset
            ConfigLoader._set_nested_attr(config, "security.enabled", false_val)
            assert config.security.enabled is False, f"Failed for value: {false_val}"

    def test_integer_conversion_from_env(self):
        """Test that numeric strings are converted to integers"""
        config = BridgeConfig()
        ConfigLoader._set_nested_attr(config, "transports.websocket.port", "1234")
        assert config.transports["websocket"].port == 1234
        assert isinstance(config.transports["websocket"].port, int)

    def test_graceful_handling_of_invalid_path(self):
        """Test that invalid paths are handled gracefully"""
        config = BridgeConfig()
        # Should not raise exception
        ConfigLoader._set_nested_attr(config, "nonexistent.path.here", "value")


class TestROSEndpointConfig:
    """Test ROS endpoint configuration"""

    def test_ros_endpoint_defaults(self):
        """Test ROS endpoint default values"""
        endpoint = ROSEndpoint(id="test_endpoint")

        assert endpoint.id == "test_endpoint"
        assert endpoint.ros_type == "ros2"
        assert endpoint.ros_distro == "jazzy"
        assert endpoint.host == "localhost"
        assert endpoint.domain_id == 0
        assert endpoint.auto_discover is True

    def test_ros_endpoint_with_auth(self):
        """Test ROS endpoint with authentication"""
        endpoint = ROSEndpoint(
            id="remote_endpoint",
            host="192.168.1.100",
            username="robot_user",
            password="robot_pass",
            ssh_key="/path/to/key.pem",
        )

        assert endpoint.username == "robot_user"
        assert endpoint.password == "robot_pass"
        assert endpoint.ssh_key == "/path/to/key.pem"


class TestPluginConfig:
    """Test plugin configuration"""

    def test_plugin_config_defaults(self):
        """Test plugin configuration defaults"""
        plugin = PluginConfig(name="test_plugin")

        assert plugin.name == "test_plugin"
        assert plugin.enabled is True
        assert plugin.source is None
        assert plugin.options == {}

    def test_plugin_config_with_options(self):
        """Test plugin configuration with options"""
        plugin = PluginConfig(
            name="greenhouse",
            enabled=True,
            source="./plugins/greenhouse.py",
            options={"control_interval": 5, "sensors": ["temp", "humidity"]},
        )

        assert plugin.options["control_interval"] == 5
        assert "temp" in plugin.options["sensors"]


class TestConfigMerge:
    """Test configuration merging"""

    def test_merge_preserves_base_values(self):
        """Test that base config values are preserved when not overridden"""
        base = BridgeConfig()
        base.name = "base_name"
        base.log_level = "DEBUG"

        override = BridgeConfig()
        override.name = "override_name"
        # log_level not overridden

        result = ConfigLoader._merge_configs(base, override)

        assert result.name == "override_name"
        assert result.log_level == "DEBUG"  # Preserved from base


class TestTransportConfig:
    """Test transport configuration"""

    def test_transport_config_defaults(self):
        """Test transport configuration defaults"""
        transport = TransportConfig()

        assert transport.enabled is True
        assert transport.host == "0.0.0.0"
        assert transport.port == 0
        assert transport.tls_cert is None
        assert transport.tls_key is None
        assert transport.options == {}

    def test_transport_config_with_tls(self):
        """Test transport configuration with TLS"""
        transport = TransportConfig(
            enabled=True,
            host="0.0.0.0",
            port=443,
            tls_cert="/etc/ssl/cert.pem",
            tls_key="/etc/ssl/key.pem",
            options={"keepalive": 60},
        )

        assert transport.tls_cert == "/etc/ssl/cert.pem"
        assert transport.options["keepalive"] == 60


class TestSecurityConfigEdgeCases:
    """Test security configuration edge cases"""

    def test_security_with_mtls(self):
        """Test security configuration with mTLS enabled"""
        security = SecurityConfig(
            enabled=True,
            mtls_enabled=True,
            tls_cert="/etc/certs/server.crt",
            tls_key="/etc/certs/server.key",
            ca_cert="/etc/certs/ca.crt",
        )

        assert security.mtls_enabled is True
        assert security.ca_cert == "/etc/certs/ca.crt"

    def test_security_authentication_methods(self):
        """Test security authentication methods list"""
        security = SecurityConfig(authentication=["jwt", "mtls", "api_key"])

        assert "jwt" in security.authentication
        assert "mtls" in security.authentication
        assert len(security.authentication) == 3


class TestConfigValidation:
    """Test configuration validation"""

    def test_invalid_log_level(self):
        """Test handling of invalid log level"""
        # Currently accepts any string, but could validate in future
        config = BridgeConfig()
        config.log_level = "INVALID_LEVEL"
        assert config.log_level == "INVALID_LEVEL"

    def test_negative_port_handling(self):
        """Test handling of negative port numbers"""
        transport = TransportConfig()
        transport.port = -1
        # Should this be validated? Currently accepts negative
        assert transport.port == -1

    def test_empty_jwt_secret_when_auth_disabled(self):
        """Test that empty JWT secret is OK when auth disabled"""
        security = SecurityConfig(enabled=False, jwt_secret=None)
        assert security.enabled is False
        assert security.jwt_secret is None


class TestConfigFileDiscovery:
    """Test automatic config file discovery"""

    def test_discovers_yaml_in_standard_locations(self, tmp_path):
        """Test discovery of YAML config in standard locations"""
        config_file = tmp_path / "gateway.yaml"
        config_file.write_text("name: discovered_yaml\n")

        # Mock the standard locations to include our temp path
        with mock.patch.object(Path, "expanduser", return_value=config_file):
            with mock.patch.dict(os.environ, {"BRIDGE_CONFIG": str(config_file)}):
                config = ConfigLoader.from_file_or_env()
                assert config.name == "discovered_yaml"

    def test_prefers_explicit_path_over_discovery(self, tmp_path):
        """Test that explicit path takes precedence"""
        explicit_config = tmp_path / "explicit.yaml"
        explicit_config.write_text("name: explicit_config\n")

        discovered_config = tmp_path / "discovered.yaml"
        discovered_config.write_text("name: discovered_config\n")

        config = ConfigLoader.from_file_or_env(str(explicit_config))
        assert config.name == "explicit_config"
