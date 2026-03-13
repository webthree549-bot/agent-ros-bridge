"""
Safety Layer Launch File
Agent ROS Bridge v0.6.1 - Week 2 Implementation

Launches all 4 safety nodes:
- /safety/limits
- /safety/validator
- /safety/emergency_stop
- /safety/watchdog

Usage:
    ros2 launch agent_ros_bridge safety_layer.launch.py
"""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    """Generate launch description for safety layer"""

    # Launch arguments
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='config/safety_limits.yaml',
        description='Path to safety limits configuration file'
    )

    auth_code_arg = DeclareLaunchArgument(
        'auth_code',
        default_value='safety_auth_12345',
        description='Authorization code for clearing emergency stop'
    )

    # Get launch configurations
    config_path = LaunchConfiguration('config_path')
    auth_code = LaunchConfiguration('auth_code')

    # Safety Limits Node
    limits_node = Node(
        package='agent_ros_bridge',
        executable='safety_limits_node',
        name='safety_limits',
        namespace='/safety',
        parameters=[{
            'config_path': config_path
        }],
        output='screen',
        emulate_tty=True
    )

    # Safety Validator Node
    validator_node = Node(
        package='agent_ros_bridge',
        executable='safety_validator_node',
        name='safety_validator',
        namespace='/safety',
        output='screen',
        emulate_tty=True
    )

    # Emergency Stop Node
    emergency_stop_node = Node(
        package='agent_ros_bridge',
        executable='safety_emergency_stop_node',
        name='safety_emergency_stop',
        namespace='/safety',
        parameters=[{
            'auth_code': auth_code
        }],
        output='screen',
        emulate_tty=True
    )

    # Watchdog Node
    watchdog_node = Node(
        package='agent_ros_bridge',
        executable='safety_watchdog_node',
        name='safety_watchdog',
        namespace='/safety',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        config_path_arg,
        auth_code_arg,
        limits_node,
        validator_node,
        emergency_stop_node,
        watchdog_node
    ])
