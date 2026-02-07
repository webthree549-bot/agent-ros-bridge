"""Greenhouse demo launch file."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mock_mode", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="INFO"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(os.path.dirname(__file__), "core.launch.py")
            )
        ),
        Node(
            package="openclaw_ros_bridge",
            executable="greenhouse_plugin",
            name="greenhouse_demo",
            output="screen",
            parameters=[{"mock_mode": LaunchConfiguration("mock_mode")}]
        ),
    ])