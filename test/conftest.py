#!/usr/bin/env python3
"""Pytest fixtures for all tests"""
import pytest
import os

# Set mock mode for all tests
os.environ["MOCK_MODE"] = "true"
os.environ["ROS_DISTRO"] = "jazzy"

@pytest.fixture
def sensor_hal():
    """Fixture for SensorHAL"""
    from openclaw_ros_bridge.hal.sensor_hal import sensor_hal
    sensor_hal.init_hardware()
    return sensor_hal

@pytest.fixture
def actuator_hal():
    """Fixture for ActuatorHAL"""
    from openclaw_ros_bridge.hal.actuator_hal import actuator_hal
    actuator_hal.init_hardware()
    return actuator_hal

@pytest.fixture
def version_manager():
    """Fixture for VersionManager"""
    from openclaw_ros_bridge.version.version_manager import version_manager
    return version_manager

@pytest.fixture
def ros_communicator():
    """Fixture for ROS Communicator"""
    from openclaw_ros_bridge.communication import get_ros_communicator
    return get_ros_communicator()

@pytest.fixture
def greenhouse_plugin():
    """Fixture for GreenhousePlugin"""
    from openclaw_ros_bridge import GreenhousePlugin
    import os
    config_path = os.path.join(os.path.dirname(__file__), "..", "demo", "greenhouse", "gh_config.yaml")
    plugin = GreenhousePlugin(config_path=config_path)
    plugin.init_core()
    plugin.init_plugin()
    return plugin

@pytest.fixture
def arm_plugin():
    """Fixture for ArmManipulationPlugin"""
    from openclaw_ros_bridge import ArmManipulationPlugin
    import os
    config_path = os.path.join(os.path.dirname(__file__), "..", "demo", "arm_manipulation", "arm_config.yaml")
    plugin = ArmManipulationPlugin(config_path=config_path)
    plugin.init_core()
    plugin.init_plugin()
    return plugin
