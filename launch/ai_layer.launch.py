"""
Launch file for AI Layer (Intent Parser + Context Manager).

Usage:
    ros2 launch agent_ros_bridge ai_layer.launch.py
"""

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for AI layer nodes."""

    # Launch arguments
    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info", description="Logging level (debug, info, warn, error)"
    )

    # Intent Parser Node
    intent_parser_node = Node(
        package="agent_ros_bridge",
        executable="intent_parser",
        name="intent_parser",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            {
                "use_llm_fallback": False,
                "rule_confidence_threshold": 0.95,
            }
        ],
    )

    # Context Manager Node
    context_manager_node = Node(
        package="agent_ros_bridge",
        executable="context_manager",
        name="context_manager",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            {
                "tf_update_rate": 10.0,
                "default_frame_id": "map",
            }
        ],
    )

    return LaunchDescription(
        [
            log_level_arg,
            intent_parser_node,
            context_manager_node,
        ]
    )
