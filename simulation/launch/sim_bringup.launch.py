import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():
    """Launch Gazebo simulation with TurtleBot3."""

    # Get paths
    pkg_simulation = get_package_share_directory("simulation")
    world_file = os.path.join(pkg_simulation, "worlds", "empty_warehouse.sdf")

    # Set Gazebo model path
    model_path = os.path.join(pkg_simulation, "models")
    set_model_path = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            world_file,
            "-s",
            "libgazebo_ros_factory.so",
            "-s",
            "libgazebo_ros_init.so",
        ],
        output="screen",
    )

    # Spawn TurtleBot3
    spawn_turtlebot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "turtlebot3_waffle",
            "-file",
            os.path.join(model_path, "turtlebot3_waffle", "model.sdf"),
            "-x",
            "-8.0",
            "-y",
            "-8.0",
            "-z",
            "0.1",
            "-Y",
            "0.0",
        ],
        output="screen",
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": open(
                    os.path.join(model_path, "turtlebot3_waffle", "model.sdf")
                ).read()
            }
        ],
    )

    # Static TF (map -> odom)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    # ROS Bridge (WebSocket)
    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        parameters=[{"port": 9090, "address": "0.0.0.0"}],
        output="screen",
    )

    # ROS Bridge (WebSocket for Agent)
    agent_websocket = Node(
        package="agent_ros_bridge",
        executable="bridge_node",
        name="agent_ros_bridge",
        parameters=[{"websocket_port": 8765, "ros_namespace": "/"}],
        output="screen",
    )

    return LaunchDescription(
        [
            set_model_path,
            gazebo,
            spawn_turtlebot,
            robot_state_publisher,
            static_tf,
            rosbridge,
            agent_websocket,
        ]
    )
