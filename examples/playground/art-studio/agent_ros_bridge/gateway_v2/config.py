#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Configuration system for OpenClaw Gateway

Supports YAML, JSON, environment variables, and dynamic configuration.
"""

import os
import yaml
import json
import logging
from typing import Dict, Any, Optional, List
from dataclasses import dataclass, field
from pathlib import Path

logger = logging.getLogger("gateway.config")


@dataclass
class TransportConfig:
    """Transport configuration"""
    enabled: bool = True
    host: str = "0.0.0.0"
    port: int = 0  # 0 means disabled
    tls_cert: Optional[str] = None
    tls_key: Optional[str] = None
    options: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ROSEndpoint:
    """Remote ROS endpoint configuration"""
    id: str
    ros_type: str = "ros2"  # ros1, ros2
    ros_distro: str = "jazzy"  # noetic, humble, jazzy
    host: str = "localhost"
    port: Optional[int] = None  # None for default
    domain_id: int = 0
    # Authentication for remote connections
    username: Optional[str] = None
    password: Optional[str] = None
    ssh_key: Optional[str] = None
    # Discovery options
    auto_discover: bool = True
    topics: List[str] = field(default_factory=list)


@dataclass
class ConnectorConfig:
    """Connector configuration"""
    enabled: bool = True
    # Multi-ROS support: list of ROS endpoints
    endpoints: List[ROSEndpoint] = field(default_factory=list)
    # Legacy single-endpoint options (for backward compatibility)
    options: Dict[str, Any] = field(default_factory=dict)


@dataclass
class SecurityConfig:
    """Security configuration"""
    enabled: bool = False
    authentication: List[str] = field(default_factory=lambda: ["jwt"])
    jwt_secret: Optional[str] = None
    tls_enabled: bool = False
    tls_cert: Optional[str] = None
    tls_key: Optional[str] = None
    mtls_enabled: bool = False
    ca_cert: Optional[str] = None


@dataclass
class PluginConfig:
    """Plugin configuration"""
    name: str
    enabled: bool = True
    source: Optional[str] = None  # Path or URL
    options: Dict[str, Any] = field(default_factory=dict)


@dataclass
class BridgeConfig:
    """Main gateway configuration"""
    name: str = "agent_ros_bridge"
    log_level: str = "INFO"
    
    # Transports
    transports: Dict[str, TransportConfig] = field(default_factory=lambda: {
        "websocket": TransportConfig(port=8765),
        "grpc": TransportConfig(port=50051),
        "tcp": TransportConfig(port=9999),
    })
    
    # Connectors
    connectors: Dict[str, ConnectorConfig] = field(default_factory=lambda: {
        "ros2": ConnectorConfig(),
        "mqtt": ConnectorConfig(),
    })
    
    # Security
    security: SecurityConfig = field(default_factory=SecurityConfig)
    
    # Plugins
    plugins: List[PluginConfig] = field(default_factory=list)
    
    # Discovery
    discovery: Dict[str, Any] = field(default_factory=lambda: {
        "enabled": True,
        "methods": ["mdns", "ros2"],
        "interval": 30
    })
    
    # Telemetry
    telemetry: Dict[str, Any] = field(default_factory=lambda: {
        "enabled": True,
        "metrics_port": 9090,
        "tracing": True
    })
    
    # Storage
    storage: Dict[str, Any] = field(default_factory=lambda: {
        "type": "memory",  # memory, sqlite, postgres
        "connection_string": None
    })


class ConfigLoader:
    """Configuration loader with environment variable support"""
    
    ENV_PREFIX = "OPENCLAW_"
    
    @classmethod
    def from_yaml(cls, path: str) -> "BridgeConfig":
        """Load configuration from YAML file"""
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        return cls._dict_to_config(data)
    
    @classmethod
    def from_json(cls, path: str) -> "BridgeConfig":
        """Load configuration from JSON file"""
        with open(path, 'r') as f:
            data = json.load(f)
        return cls._dict_to_config(data)
    
    @classmethod
    def from_env(cls) -> "BridgeConfig":
        """Load configuration from environment variables"""
        config = BridgeConfig()
        
        # Simple env var mapping
        env_mappings = {
            "NAME": "name",
            "LOG_LEVEL": "log_level",
            "WEBSOCKET_PORT": "transports.websocket.port",
            "GRPC_PORT": "transports.grpc.port",
            "TCP_PORT": "transports.tcp.port",
            "JWT_SECRET": "security.jwt_secret",
            "TLS_CERT": "security.tls_cert",
            "TLS_KEY": "security.tls_key",
        }
        
        for env_key, config_key in env_mappings.items():
            env_val = os.getenv(f"{cls.ENV_PREFIX}{env_key}")
            if env_val is not None:
                cls._set_nested_attr(config, config_key, env_val)
        
        return config
    
    @classmethod
    def from_file_or_env(cls, path: Optional[str] = None) -> "BridgeConfig":
        """Load from file or environment"""
        # Try config file locations
        config_paths = [
            path,
            os.getenv("OPENCLAW_CONFIG"),
            "./config/gateway.yaml",
            "./config/gateway.json",
            "~/.openclaw/gateway.yaml",
            "/etc/openclaw/gateway.yaml",
        ]
        
        for p in config_paths:
            if p and Path(p).expanduser().exists():
                if p.endswith('.yaml') or p.endswith('.yml'):
                    config = cls.from_yaml(p)
                    logger.info(f"Loaded config from {p}")
                    break
                elif p.endswith('.json'):
                    config = cls.from_json(p)
                    logger.info(f"Loaded config from {p}")
                    break
        else:
            config = BridgeConfig()
            logger.info("Using default configuration")
        
        # Override with environment variables
        env_config = cls.from_env()
        config = cls._merge_configs(config, env_config)
        
        return config
    
    @classmethod
    def _dict_to_config(cls, data: Dict[str, Any]) -> "BridgeConfig":
        """Convert dictionary to BridgeConfig"""
        # Simplified conversion - full implementation would handle nested structures
        return BridgeConfig(
            name=data.get("name", "agent_ros_bridge"),
            log_level=data.get("log_level", "INFO"),
            transports=cls._parse_transports(data.get("transports", {})),
            connectors=cls._parse_connectors(data.get("connectors", {})),
            security=cls._parse_security(data.get("security", {})),
            plugins=cls._parse_plugins(data.get("plugins", [])),
            discovery=data.get("discovery", {"enabled": True}),
            telemetry=data.get("telemetry", {"enabled": True}),
            storage=data.get("storage", {"type": "memory"}),
        )
    
    @classmethod
    def _parse_transports(cls, data: Dict[str, Any]) -> Dict[str, TransportConfig]:
        """Parse transport configurations"""
        transports = {}
        for name, cfg in data.items():
            transports[name] = TransportConfig(
                enabled=cfg.get("enabled", True),
                host=cfg.get("host", "0.0.0.0"),
                port=cfg.get("port", 0),
                tls_cert=cfg.get("tls_cert"),
                tls_key=cfg.get("tls_key"),
                options=cfg.get("options", {})
            )
        return transports
    
    @classmethod
    def _parse_connectors(cls, data: Dict[str, Any]) -> Dict[str, ConnectorConfig]:
        """Parse connector configurations"""
        connectors = {}
        for name, cfg in data.items():
            connectors[name] = ConnectorConfig(
                enabled=cfg.get("enabled", True),
                options=cfg.get("options", {})
            )
        return connectors
    
    @classmethod
    def _parse_security(cls, data: Dict[str, Any]) -> SecurityConfig:
        """Parse security configuration"""
        return SecurityConfig(
            enabled=data.get("enabled", False),
            authentication=data.get("authentication", ["jwt"]),
            jwt_secret=data.get("jwt_secret"),
            tls_enabled=data.get("tls_enabled", False),
            tls_cert=data.get("tls_cert"),
            tls_key=data.get("tls_key"),
            mTLS_enabled=data.get("mtls_enabled", False),
            ca_cert=data.get("ca_cert")
        )
    
    @classmethod
    def _parse_plugins(cls, data: List[Dict[str, Any]]) -> List[PluginConfig]:
        """Parse plugin configurations"""
        plugins = []
        for p in data:
            plugins.append(PluginConfig(
                name=p.get("name", "unknown"),
                enabled=p.get("enabled", True),
                source=p.get("source"),
                options=p.get("options", {})
            ))
        return plugins
    
    @classmethod
    def _set_nested_attr(cls, obj: Any, path: str, value: Any) -> None:
        """Set nested attribute by dot path"""
        parts = path.split(".")
        for part in parts[:-1]:
            obj = getattr(obj, part)
        setattr(obj, parts[-1], value)
    
    @classmethod
    def _merge_configs(cls, base: "BridgeConfig", override: "BridgeConfig") -> "BridgeConfig":
        """Merge two configurations (override takes precedence)"""
        # Simplified merge - full implementation would handle all fields
        result = BridgeConfig()
        
        # Copy base
        for field_name in base.__dataclass_fields__:
            setattr(result, field_name, getattr(base, field_name))
        
        # Apply overrides (non-default values)
        for field_name in override.__dataclass_fields__:
            override_val = getattr(override, field_name)
            if override_val != getattr(BridgeConfig(), field_name):
                setattr(result, field_name, override_val)
        
        return result


# Example configuration file content
EXAMPLE_CONFIG_YAML = '''
name: "warehouse_gateway_01"
log_level: INFO

transports:
  websocket:
    enabled: true
    host: 0.0.0.0
    port: 8765
    # tls_cert: /etc/certs/gateway.crt
    # tls_key: /etc/certs/gateway.key
  
  grpc:
    enabled: true
    host: 0.0.0.0
    port: 50051
  
  tcp:
    enabled: true
    host: 0.0.0.0
    port: 9999
  
  mqtt:
    enabled: false
    host: localhost
    port: 1883

connectors:
  ros2:
    enabled: true
    options:
      domain_id: 0
  
  mqtt:
    enabled: false
    options:
      broker: mqtt.local
      port: 1883

security:
  enabled: false
  authentication:
    - jwt
  jwt_secret: ${JWT_SECRET}

plugins:
  - name: greenhouse
    enabled: true
    source: ./plugins/greenhouse_plugin.py
    options:
      control_interval: 5
  
  - name: safety_monitor
    enabled: true
    source: ./plugins/safety.py

discovery:
  enabled: true
  methods:
    - mdns
    - ros2
  interval: 30

telemetry:
  enabled: true
  metrics_port: 9090
  tracing: true

storage:
  type: memory
  # type: sqlite
  # connection_string: ./data/gateway.db
'''


if __name__ == "__main__":
    # Example: Create example config file
    with open("config_example.yaml", "w") as f:
        f.write(EXAMPLE_CONFIG_YAML)
    print("Created config_example.yaml")
    
    # Example: Load configuration
    config = ConfigLoader.from_file_or_env()
    print(f"\nGateway name: {config.name}")
    print(f"WebSocket port: {config.transports.get('websocket', TransportConfig()).port}")
    print(f"gRPC port: {config.transports.get('grpc', TransportConfig()).port}")
    print(f"Plugins: {[p.name for p in config.plugins]}")
