#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Graded Logger - Unified logging with file rotation, remote streaming & ROS support"""
import os
import logging
import logging.handlers
from typing import Optional, Literal

LogLevel = Literal["DEBUG", "INFO", "WARN", "ERROR", "FATAL"]
ROS_LOG_LEVEL_MAP = {
    "DEBUG": logging.DEBUG,
    "INFO": logging.INFO,
    "WARN": logging.WARNING,
    "ERROR": logging.ERROR,
    "FATAL": logging.CRITICAL
}

# Default log config (used before version_manager is available)
DEFAULT_LOG_CONFIG = {
    "level": "INFO",
    "log_dir": "logs/",
    "log_file": "openclaw_ros_bridge.log",
    "max_file_size": 10485760,
    "max_rotated_logs": 5,
    "file_save": True,
    "console_logging": True
}

def get_logger(name: str, log_level: Optional[LogLevel] = None) -> logging.Logger:
    """
    Create a graded logger with file rotation and console output
    Configured from global/debug config (version_manager)
    
    Args:
        name: Logger name (usually __name__)
        log_level: Custom log level (overrides config)
    
    Returns:
        Configured logging.Logger instance
    """
    try:
        from openclaw_ros_bridge.version.version_manager import version_manager
        debug_config = version_manager.debug_config
        global_config = version_manager.global_config
        log_level = log_level or debug_config["logging"]["level"]
        log_dir = global_config["logger"]["log_dir"]
        log_file = global_config["logger"]["log_file"]
        max_file_size = global_config["logger"]["max_file_size"]
        max_rotated_logs = global_config["logger"]["max_rotated_logs"]
        console_logging = debug_config["logging"]["console_logging"]
        file_logging = global_config["logger"]["file_save"]
        log_format = debug_config["logging"]["log_format"]
        date_format = debug_config["logging"]["date_format"]
    except (ImportError, AttributeError):
        # Fallback defaults if version_manager not initialized
        log_level = log_level or DEFAULT_LOG_CONFIG["level"]
        log_dir = DEFAULT_LOG_CONFIG["log_dir"]
        log_file = DEFAULT_LOG_CONFIG["log_file"]
        max_file_size = DEFAULT_LOG_CONFIG["max_file_size"]
        max_rotated_logs = DEFAULT_LOG_CONFIG["max_rotated_logs"]
        console_logging = DEFAULT_LOG_CONFIG["console_logging"]
        file_logging = DEFAULT_LOG_CONFIG["file_save"]
        log_format = "[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s"
        date_format = "%Y-%m-%d %H:%M:%S"

    # Create log directory if not exists
    if file_logging and not os.path.exists(log_dir):
        os.makedirs(log_dir, exist_ok=True)

    # Create logger
    logger = logging.getLogger(name)
    logger.setLevel(ROS_LOG_LEVEL_MAP[log_level])
    logger.propagate = False

    # Clear existing handlers to avoid duplication
    logger.handlers.clear()

    # Log format
    formatter = logging.Formatter(log_format, datefmt=date_format)

    # Console handler
    if console_logging:
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

    # File handler (rotating)
    if file_logging:
        file_path = os.path.join(log_dir, log_file)
        file_handler = logging.handlers.RotatingFileHandler(
            file_path,
            maxBytes=max_file_size,
            backupCount=max_rotated_logs,
            encoding="utf-8"
        )
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    return logger
