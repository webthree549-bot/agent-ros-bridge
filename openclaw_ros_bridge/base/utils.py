#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Utility Functions - Reusable helpers for the entire framework"""
import os
import platform
import psutil
import random
import string
from typing import List, Optional, Dict, Any
from openclaw_ros_bridge.base.logger import get_logger

logger = get_logger(__name__)

def validate_path(path: str, create: bool = False) -> bool:
    """
    Validate a file/directory path exists (create if requested)
    
    Args:
        path: Path to validate
        create: Create directory if it does not exist
    
    Returns:
        True if path is valid/exists, False otherwise
    """
    if os.path.exists(path):
        return True
    if create and not os.path.isfile(path):
        os.makedirs(path, exist_ok=True)
        logger.info(f"Created directory: {path}")
        return True
    logger.error(f"Path does not exist: {path}")
    return False

def convert_to_bool(value: Any) -> bool:
    """
    Convert any value to a boolean (robust for config/args)
    
    Args:
        value: Value to convert (str/bool/int)
    
    Returns:
        Boolean representation of the value
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ["true", "1", "yes", "on"]
    if isinstance(value, int):
        return value == 1
    return False

def get_cpu_cores() -> List[int]:
    """
    Get list of available CPU cores for affinity binding
    
    Returns:
        List of CPU core IDs (0-indexed)
    """
    cores = list(range(psutil.cpu_count(logical=False) or psutil.cpu_count() or 1))
    logger.debug(f"Available CPU cores: {cores}")
    return cores

def get_system_info() -> Dict[str, str]:
    """
    Get basic system information for debugging/deployment
    
    Returns:
        Dict of system info (OS, CPU, RAM, Python, ROS)
    """
    from openclaw_ros_bridge.version.version_manager import version_manager
    return {
        "os": f"{platform.system()} {platform.release()} {platform.machine()}",
        "cpu": platform.processor() or "Unknown",
        "ram": f"{round(psutil.virtual_memory().total / 1024**3, 2)} GB",
        "python": platform.python_version(),
        "ros_type": version_manager.ROS_TYPE,
        "ros_distro": version_manager.ROS_DISTRO,
        "openclaw_version": version_manager.OC_VER
    }

def generate_business_id(prefix: str = "id", length: int = 6) -> str:
    """
    Generate a random unique business ID for plugins
    
    Args:
        prefix: Prefix for the ID
        length: Length of random alphanumeric suffix
    
    Returns:
        Unique business ID (e.g., id_abc123)
    """
    suffix = ''.join(random.choices(string.ascii_lowercase + string.digits, k=length))
    business_id = f"{prefix}_{suffix}"
    logger.debug(f"Generated business ID: {business_id}")
    return business_id

def calculate_latency(start_time: float, end_time: float) -> float:
    """
    Calculate latency in milliseconds from start/end times (time.time())
    
    Args:
        start_time: Start time (float from time.time())
        end_time: End time (float from time.time())
    
    Returns:
        Latency in milliseconds (rounded to 3 decimals)
    """
    latency = (end_time - start_time) * 1000
    return round(latency, 3)

def safe_shutdown(func):
    """
    Decorator for graceful shutdown of nodes/threads
    
    Args:
        func: Function to decorate
    
    Returns:
        Wrapped function with error handling and shutdown logging
    """
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except KeyboardInterrupt:
            logger.info("Received keyboard interrupt - initiating graceful shutdown")
        except Exception as e:
            logger.error(f"Error in wrapped function: {str(e)}", exc_info=True)
        finally:
            logger.info("Graceful shutdown complete")
    return wrapper