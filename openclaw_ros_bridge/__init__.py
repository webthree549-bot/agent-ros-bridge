# OpenClaw ROS Bridge
from openclaw_ros_bridge.base import ConfigLoader, get_logger, RealTimeScheduler as RTScheduler
from openclaw_ros_bridge.base.utils import (
    validate_path, convert_to_bool, get_cpu_cores, get_system_info, generate_business_id
)

class Utils:
    """Utility wrapper class"""
    validate_path = staticmethod(validate_path)
    convert_to_bool = staticmethod(convert_to_bool)
    get_cpu_cores = staticmethod(get_cpu_cores)
    get_system_info = staticmethod(get_system_info)
    generate_business_id = staticmethod(generate_business_id)

Logger = get_logger
from openclaw_ros_bridge.version import VersionManager, version_manager
from openclaw_ros_bridge.communication import get_ros_communicator, openclaw_comm
from openclaw_ros_bridge.hal import SensorHAL, ActuatorHAL, sensor_hal, actuator_hal
from openclaw_ros_bridge.converter import DataConverter, data_converter
from openclaw_ros_bridge.fault import RecoveryManager, recovery_manager
from openclaw_ros_bridge.monitor import PerformanceMonitor, StateMonitor, perf_monitor, state_monitor
from openclaw_ros_bridge.plugin_base import BasePlugin

# Demo Plugins
try:
    from demo.greenhouse.greenhouse_plugin import GreenhousePlugin
except ImportError:
    GreenhousePlugin = None

try:
    from demo.arm_manipulation.arm_plugin import ArmManipulationPlugin
except ImportError:
    ArmManipulationPlugin = None

__version__ = '1.0.0'

__all__ = [
    'ConfigLoader', 'get_logger', 'Logger', 'Utils', 'RTScheduler',
    'VersionManager', 'version_manager',
    'get_ros_communicator', 'openclaw_comm',
    'SensorHAL', 'ActuatorHAL', 'sensor_hal', 'actuator_hal',
    'DataConverter', 'data_converter',
    'RecoveryManager', 'recovery_manager',
    'PerformanceMonitor', 'StateMonitor', 'perf_monitor', 'state_monitor',
    'BasePlugin',
]

if GreenhousePlugin is not None:
    __all__.append('GreenhousePlugin')
if ArmManipulationPlugin is not None:
    __all__.append('ArmManipulationPlugin')
