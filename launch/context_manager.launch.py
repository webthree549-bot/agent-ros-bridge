"""
Launch file for Context Manager Node.

Usage:
    ros2 launch agent_ros_bridge context_manager.launch.py
"""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    """Generate launch description for context manager node."""

    # Launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )

    # Context Manager Node
    context_manager_node = Node(
        package='agent_ros_bridge',
        executable='context_manager',
        name='context_manager',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            {
                # Add parameters here as needed
                'tf_update_rate': 10.0,  # Hz
                'default_frame_id': 'map',
            }
        ]
    )

    return LaunchDescription([
        log_level_arg,
        context_manager_node,
    ])
