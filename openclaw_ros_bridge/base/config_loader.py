#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unified Config Loader - Load YAML/config with env override & validation"""
import os
import yaml
from dotenv import load_dotenv
from typing import Dict, Optional, Any
from openclaw_ros_bridge.base.logger import get_logger

# Load .env file if exists
load_dotenv()

logger = get_logger(__name__)

class ConfigLoader:
    """Singleton config loader with YAML support and environment variable override"""
    _instance: Optional["ConfigLoader"] = None
    _initialized: bool = False

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self.config_cache: Dict[str, Dict] = {}
        self._initialized = True

    def load_yaml(self, config_path: str, cache: bool = True) -> Dict[str, Any]:
        """
        Load YAML config file with error handling and env override
        Env override format: ENV_<SECTION>_<KEY>=value (case-insensitive)
        
        Args:
            config_path: Path to YAML config file
            cache: Cache loaded config to avoid reloading
        
        Returns:
            Loaded and env-overridden config dict
        
        Raises:
            FileNotFoundError: If config file does not exist
            yaml.YAMLError: If YAML parsing fails
        """
        if config_path in self.config_cache and cache:
            return self.config_cache[config_path]

        if not os.path.exists(config_path):
            logger.error(f"Config file not found: {config_path}")
            raise FileNotFoundError(f"Config file not found: {config_path}")

        try:
            with open(config_path, "r", encoding="utf-8") as f:
                config = yaml.safe_load(f) or {}
            logger.debug(f"Loaded raw config from: {config_path}")

            # Apply environment variable overrides
            config = self._apply_env_override(config)
            logger.debug(f"Applied env overrides to config: {config_path}")

            if cache:
                self.config_cache[config_path] = config
            return config
        except yaml.YAMLError as e:
            logger.error(f"YAML parse error in {config_path}: {str(e)}")
            raise
        except Exception as e:
            logger.error(f"Failed to load config {config_path}: {str(e)}", exc_info=True)
            raise

    def _apply_env_override(self, config: Dict[str, Any]) -> Dict[str, Any]:
        """
        Apply environment variable overrides to config
        Recursively process nested dicts
        
        Args:
            config: Raw config dict
        
        Returns:
            Config dict with env overrides applied
        """
        for key, value in config.items():
            env_key = f"ENV_{key.upper()}"
            if isinstance(value, dict):
                config[key] = self._apply_env_override(value)
            elif os.getenv(env_key) is not None:
                # Convert env value to match original type
                env_value = os.getenv(env_key)
                if isinstance(value, bool):
                    config[key] = env_value.lower() in ["true", "1", "yes"]
                elif isinstance(value, int):
                    config[key] = int(env_value)
                elif isinstance(value, float):
                    config[key] = float(env_value)
                elif isinstance(value, list):
                    config[key] = [v.strip() for v in env_value.split(",")]
                else:
                    config[key] = env_value
                logger.info(f"Overridden config {key} with env {env_key}: {config[key]}")
        return config

    def clear_cache(self) -> None:
        """Clear config cache to force reloading"""
        self.config_cache.clear()
        logger.info("Cleared config loader cache")

# Singleton instance
config_loader = ConfigLoader()
