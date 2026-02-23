"""Configuration management for Agent ROS Bridge.

Supports configuration via:
- YAML/JSON config files
- Environment variables
- Command-line arguments
- Runtime updates

Example config.yaml for distributed deployment:
    bridge:
      ros_version: 2
      log_level: INFO
    
    transports:
      websocket:
        host: 0.0.0.0
        port: 8765
        tls:
          cert: /etc/ssl/certs/bridge.crt
          key: /etc/ssl/private/bridge.key
        auth:
          enabled: true
          jwt_secret: ${JWT_SECRET}
        cors:
          - https://openclaw.ai
          - https://dashboard.robot.io
      
      grpc:
        host: 0.0.0.0
        port: 50051
        tls:
          cert: /etc/ssl/certs/bridge.crt
          key: /etc/ssl/private/bridge.key
          ca: /etc/ssl/certs/ca.crt  # For mutual TLS
    
    connectors:
      ros2:
        domain_id: 0
        namespace: /agent_bridge
      
      # Or remote ROS via rosbridge_server
      remote_ros:
        type: rosbridge_websocket
        uri: wss://ros-master.robot.local:9090
        auth:
          token: ${ROS_TOKEN}
    
    security:
      rate_limit:
        requests_per_second: 100
        burst_size: 200
      audit_log:
        enabled: true
        destination: /var/log/bridge/audit.log
"""

import os
import logging
from typing import Any, Dict, Optional
from pathlib import Path

logger = logging.getLogger(__name__)


class BridgeConfig:
    """Configuration manager for Agent ROS Bridge."""
    
    # Default configuration
    DEFAULTS = {
        "bridge": {
            "ros_version": 2,
            "log_level": "INFO",
            "node_name": "agent_ros_bridge"
        },
        "transports": {
            "websocket": {
                "enabled": True,
                "host": "0.0.0.0",
                "port": 8765,
                "auth": {
                    "enabled": False
                },
                "ping_interval": 20,
                "ping_timeout": 10
            },
            "grpc": {
                "enabled": True,
                "host": "0.0.0.0",
                "port": 50051,
                "max_workers": 10,
                "auth": {
                    "enabled": False
                }
            },
            "mcp": {
                "enabled": False  # Enabled via Claude Desktop config
            }
        },
        "connectors": {
            "ros2": {
                "enabled": True,
                "domain_id": 0,
                "namespace": "",
                "use_sim_time": False
            }
        },
        "security": {
            "rate_limit": {
                "enabled": True,
                "requests_per_second": 100,
                "burst_size": 200
            },
            "audit_log": {
                "enabled": False,
                "destination": None
            },
            "cors": {
                "enabled": True,
                "origins": ["*"]
            }
        },
        "remote": {
            "openclaw": {
                "enabled": False,
                "endpoint": None,
                "api_key": None
            }
        }
    }
    
    def __init__(self, config_path: Optional[str] = None):
        """Initialize configuration.
        
        Args:
            config_path: Path to YAML/JSON config file
        """
        self._config = self._deep_merge(self.DEFAULTS, {})
        
        if config_path:
            self.load_file(config_path)
        
        # Apply environment variable overrides
        self._apply_env_overrides()
    
    def _deep_merge(self, base: Dict, override: Dict) -> Dict:
        """Deep merge two dictionaries."""
        result = base.copy()
        for key, value in override.items():
            if key in result and isinstance(result[key], dict) and isinstance(value, dict):
                result[key] = self._deep_merge(result[key], value)
            else:
                result[key] = value
        return result
    
    def load_file(self, path: str):
        """Load configuration from file.
        
        Supports .yaml, .yml, and .json files.
        """
        path = Path(path)
        
        if not path.exists():
            raise FileNotFoundError(f"Config file not found: {path}")
        
        if path.suffix in ('.yaml', '.yml'):
            try:
                import yaml
                with open(path) as f:
                    data = yaml.safe_load(f)
            except ImportError:
                raise ImportError("PyYAML required for YAML config: pip install pyyaml")
        elif path.suffix == '.json':
            import json
            with open(path) as f:
                data = json.load(f)
        else:
            raise ValueError(f"Unsupported config format: {path.suffix}")
        
        self._config = self._deep_merge(self._config, data)
        logger.info(f"Loaded config from {path}")
    
    def _apply_env_overrides(self):
        """Apply environment variable overrides."""
        # Map of env vars to config paths
        env_mappings = {
            'BRIDGE_ROS_VERSION': ('bridge', 'ros_version'),
            'BRIDGE_LOG_LEVEL': ('bridge', 'log_level'),
            'BRIDGE_WS_HOST': ('transports', 'websocket', 'host'),
            'BRIDGE_WS_PORT': ('transports', 'websocket', 'port'),
            'BRIDGE_WS_TLS_CERT': ('transports', 'websocket', 'tls', 'cert'),
            'BRIDGE_WS_TLS_KEY': ('transports', 'websocket', 'tls', 'key'),
            'BRIDGE_GRPC_HOST': ('transports', 'grpc', 'host'),
            'BRIDGE_GRPC_PORT': ('transports', 'grpc', 'port'),
            'BRIDGE_GRPC_TLS_CERT': ('transports', 'grpc', 'tls', 'cert'),
            'BRIDGE_GRPC_TLS_KEY': ('transports', 'grpc', 'tls', 'key'),
            'BRIDGE_JWT_SECRET': ('transports', 'websocket', 'auth', 'jwt_secret'),
            'BRIDGE_ROS2_DOMAIN_ID': ('connectors', 'ros2', 'domain_id'),
            'BRIDGE_ROS2_NAMESPACE': ('connectors', 'ros2', 'namespace'),
            'OPENCLAW_ENDPOINT': ('remote', 'openclaw', 'endpoint'),
            'OPENCLAW_API_KEY': ('remote', 'openclaw', 'api_key'),
        }
        
        for env_var, path in env_mappings.items():
            value = os.environ.get(env_var)
            if value is not None:
                # Convert port numbers to int
                if 'PORT' in env_var:
                    try:
                        value = int(value)
                    except ValueError:
                        pass
                
                self._set_nested(self._config, path, value)
                logger.debug(f"Applied env override: {env_var}")
    
    def _set_nested(self, d: Dict, path: tuple, value: Any):
        """Set a nested dictionary value."""
        for key in path[:-1]:
            if key not in d:
                d[key] = {}
            d = d[key]
        d[path[-1]] = value
    
    def get(self, *path: str, default: Any = None) -> Any:
        """Get a configuration value.
        
        Args:
            *path: Path to the config value (e.g., 'transports', 'websocket', 'port')
            default: Default value if not found
            
        Returns:
            Configuration value or default
        """
        value = self._config
        for key in path:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        return value
    
    def set(self, *path: str, value: Any):
        """Set a configuration value."""
        self._set_nested(self._config, path, value)
    
    def get_transport_config(self, name: str) -> Dict[str, Any]:
        """Get configuration for a specific transport."""
        return self.get('transports', name, default={})
    
    def get_connector_config(self, name: str) -> Dict[str, Any]:
        """Get configuration for a specific connector."""
        return self.get('connectors', name, default={})
    
    def is_transport_enabled(self, name: str) -> bool:
        """Check if a transport is enabled."""
        return self.get('transports', name, 'enabled', default=False)
    
    def is_connector_enabled(self, name: str) -> bool:
        """Check if a connector is enabled."""
        return self.get('connectors', name, 'enabled', default=False)
    
    def to_dict(self) -> Dict[str, Any]:
        """Get full configuration as dictionary."""
        return self._config.copy()
    
    @classmethod
    def from_env(cls) -> 'BridgeConfig':
        """Create config from environment variables only."""
        config = cls()
        config._apply_env_overrides()
        return config
    
    @classmethod
    def discover_config(cls) -> Optional['BridgeConfig']:
        """Discover and load config from standard locations.
        
        Searches in order:
        1. BRIDGE_CONFIG env var
        2. ./bridge.yaml
        3. ./config/bridge.yaml
        4. ~/.config/agent-ros-bridge/bridge.yaml
        5. /etc/agent-ros-bridge/bridge.yaml
        """
        paths = [
            os.environ.get('BRIDGE_CONFIG'),
            './bridge.yaml',
            './config/bridge.yaml',
            os.path.expanduser('~/.config/agent-ros-bridge/bridge.yaml'),
            '/etc/agent-ros-bridge/bridge.yaml'
        ]
        
        for path in paths:
            if path and Path(path).exists():
                return cls(path)
        
        # Return default config if no file found
        return cls()


def load_config(path: Optional[str] = None) -> BridgeConfig:
    """Load configuration from file or environment.
    
    Args:
        path: Optional explicit config file path
        
    Returns:
        BridgeConfig instance
    """
    if path:
        return BridgeConfig(path)
    return BridgeConfig.discover_config()


__all__ = ["BridgeConfig", "load_config"]
