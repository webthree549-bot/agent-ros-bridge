"""Core framework launch file."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mock_mode", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="INFO"),
        Node(
            package="openclaw_ros_bridge",
            executable="performance_monitor",
            name="performance_monitor",
            output="screen"
        ),
    ])