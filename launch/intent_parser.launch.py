"""
Launch file for Intent Parser Node.

Usage:
    ros2 launch agent_ros_bridge intent_parser.launch.py
"""

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for intent parser node."""

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
                # Add parameters here as needed
                "use_llm_fallback": False,  # Week 2: rule-based only
                "rule_confidence_threshold": 0.95,
            }
        ],
    )

    return LaunchDescription(
        [
            log_level_arg,
            intent_parser_node,
        ]
    )
