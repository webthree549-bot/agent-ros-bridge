"""Launch file for motion planning nodes.

This launch file starts the motion_planner and execution_monitor nodes
with configuration for Nav2 and MoveIt2 integration.
"""

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    """Generate launch description for motion planning nodes."""

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_nav2_enabled = DeclareLaunchArgument(
        'nav2_enabled',
        default_value='true',
        description='Enable Nav2 integration'
    )

    declare_moveit_enabled = DeclareLaunchArgument(
        'moveit_enabled',
        default_value='true',
        description='Enable MoveIt2 integration'
    )

    declare_safety_validation = DeclareLaunchArgument(
        'safety_validation',
        default_value='true',
        description='Enable safety validation'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_enabled = LaunchConfiguration('nav2_enabled')
    moveit_enabled = LaunchConfiguration('moveit_enabled')
    safety_validation = LaunchConfiguration('safety_validation')

    # Motion Planner Node
    motion_planner_node = Node(
        package='agent_ros_bridge',
        executable='motion_planner',
        name='motion_planner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'nav2_enabled': nav2_enabled,
            'moveit_enabled': moveit_enabled,
            'safety_validation': safety_validation,
            'max_linear_velocity': 1.0,
            'max_angular_velocity': 1.0,
            'planning_timeout': 5.0,
        }],
        remappings=[
            ('/plan_motion', '/ai/plan_motion'),
            ('/motion_plan', '/ai/motion_plan'),
        ]
    )

    # Execution Monitor Node
    execution_monitor_node = Node(
        package='agent_ros_bridge',
        executable='execution_monitor',
        name='execution_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'safety_validation': safety_validation,
            'stuck_time_threshold': 10.0,
            'deviation_distance_threshold': 0.5,
            'obstacle_distance_threshold': 0.5,
            'timeout_factor': 2.0,
            'max_recovery_attempts': 3,
        }],
        remappings=[
            ('/execute_motion', '/ai/execute_motion'),
            ('/execution_status', '/ai/execution_status'),
            ('/telemetry', '/robot/telemetry'),
        ]
    )

    # Include Nav2 launch if enabled
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        condition=LaunchConfiguration('nav2_enabled'),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Include MoveIt2 launch if enabled
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moveit_config'),  # Package with MoveIt config
                'launch',
                'move_group.launch.py'
            ])
        ]),
        condition=LaunchConfiguration('moveit_enabled'),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_nav2_enabled)
    ld.add_action(declare_moveit_enabled)
    ld.add_action(declare_safety_validation)

    # Add nodes
    ld.add_action(motion_planner_node)
    ld.add_action(execution_monitor_node)

    # Add optional integrations
    # ld.add_action(nav2_launch)  # Uncomment when Nav2 is available
    # ld.add_action(moveit_launch)  # Uncomment when MoveIt2 is available

    return ld


# For direct execution
if __name__ == '__main__':
    generate_launch_description()
