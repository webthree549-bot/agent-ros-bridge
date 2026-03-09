#!/usr/bin/env python3
"""
Gazebo E2E Test for Agent ROS Bridge

This test uses actual Gazebo simulation with TurtleBot3.
No mocking - real physics, real ROS2, real robot behavior.

Prerequisites:
    - ROS2 Humble installed and sourced
    - Gazebo Ignition installed
    - TurtleBot3 packages: sudo apt install ros-humble-turtlebot3*
    - Agent ROS Bridge installed

Usage:
    # Terminal 1: Start Gazebo (manual for now)
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

    # Terminal 2: Run this test
    python tests/e2e/test_gazebo_e2e.py
"""

import pytest
import time
import subprocess
import signal
import os
import sys
from pathlib import Path

# Check if ROS2 is available
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

# Skip all tests if ROS2 not available
pytestmark = pytest.mark.skipif(
    not ROS2_AVAILABLE,
    reason="ROS2 not available. Source /opt/ros/humble/setup.bash"
)


class TestGazeboE2E:
    """End-to-end tests using real Gazebo simulation."""
    
    @pytest.fixture(scope="class")
    def ros_node(self):
        """Create ROS2 node for testing."""
        if not ROS2_AVAILABLE:
            pytest.skip("ROS2 not available")
        
        rclpy.init()
        node = Node("gazebo_e2e_test")
        
        yield node
        
        node.destroy_node()
        rclpy.shutdown()
    
    def test_gazebo_running(self, ros_node):
        """Verify Gazebo simulation is running."""
        # Check if /clock topic exists (Gazebo publishes this)
        topic_names = [t for t, _ in ros_node.get_topic_names_and_types()]
        
        # Look for Gazebo-specific topics
        gazebo_topics = [t for t in topic_names if 'gazebo' in t.lower() or 'clock' in t]
        
        # Also check for robot topics
        robot_topics = [t for t in topic_names if 'cmd_vel' in t or 'odom' in t]
        
        print(f"\nDetected topics: {len(topic_names)}")
        print(f"Gazebo topics: {gazebo_topics[:5]}")
        print(f"Robot topics: {robot_topics[:5]}")
        
        # If we have robot topics, Gazebo is likely running
        assert len(robot_topics) > 0, "No robot topics found. Is Gazebo running?"
    
    def test_robot_publishing_odometry(self, ros_node):
        """Verify robot is publishing odometry data."""
        from nav_msgs.msg import Odometry
        
        odom_received = []
        
        def odom_callback(msg):
            odom_received.append(msg)
        
        # Subscribe to odometry
        sub = ros_node.create_subscription(
            Odometry,
            "/odom",
            odom_callback,
            10
        )
        
        # Wait for messages
        start_time = time.time()
        while len(odom_received) < 5 and time.time() - start_time < 5.0:
            rclpy.spin_once(ros_node, timeout_sec=0.1)
        
        sub.destroy()
        
        assert len(odom_received) > 0, "No odometry messages received"
        
        # Check odometry data is valid
        latest = odom_received[-1]
        assert latest.pose.pose.position.x is not None
        assert latest.pose.pose.position.y is not None
        
        print(f"\n✅ Robot odometry working")
        print(f"   Position: x={latest.pose.pose.position.x:.2f}, y={latest.pose.pose.position.y:.2f}")
    
    def test_send_velocity_command(self, ros_node):
        """Test sending velocity commands to robot."""
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        
        # Subscribe to odometry to verify movement
        initial_pose = None
        current_pose = None
        
        def odom_callback(msg):
            nonlocal current_pose
            current_pose = msg.pose.pose
            if initial_pose is None:
                nonlocal initial_pose
                initial_pose = msg.pose.pose
        
        odom_sub = ros_node.create_subscription(
            Odometry,
            "/odom",
            odom_callback,
            10
        )
        
        # Wait for initial pose
        start_time = time.time()
        while initial_pose is None and time.time() - start_time < 3.0:
            rclpy.spin_once(ros_node, timeout_sec=0.1)
        
        assert initial_pose is not None, "Could not get initial pose"
        
        # Publish velocity command
        cmd_pub = ros_node.create_publisher(Twist, "/cmd_vel", 10)
        
        cmd = Twist()
        cmd.linear.x = 0.1  # 0.1 m/s forward
        cmd.angular.z = 0.0
        
        print(f"\nSending velocity command: linear.x={cmd.linear.x} m/s")
        
        # Send command for 2 seconds
        start_time = time.time()
        while time.time() - start_time < 2.0:
            cmd_pub.publish(cmd)
            rclpy.spin_once(ros_node, timeout_sec=0.1)
        
        # Stop
        cmd.linear.x = 0.0
        cmd_pub.publish(cmd)
        
        # Check movement
        assert current_pose is not None
        
        import math
        distance = math.sqrt(
            (current_pose.position.x - initial_pose.position.x)**2 +
            (current_pose.position.y - initial_pose.position.y)**2
        )
        
        print(f"   Initial position: x={initial_pose.position.x:.2f}, y={initial_pose.position.y:.2f}")
        print(f"   Final position: x={current_pose.position.x:.2f}, y={current_pose.position.y:.2f}")
        print(f"   Distance moved: {distance:.2f}m")
        
        odom_sub.destroy()
        cmd_pub.destroy()
        
        # Should have moved roughly 0.2m (0.1 m/s * 2s)
        assert distance > 0.05, f"Robot didn't move enough: {distance:.2f}m"
        assert distance < 0.5, f"Robot moved too much: {distance:.2f}m"
        
        print(f"✅ Robot responds to velocity commands")
    
    def test_nav2_available(self, ros_node):
        """Test if Nav2 navigation stack is available."""
        from nav2_msgs.action import NavigateToPose
        
        nav_client = ActionClient(ros_node, NavigateToPose, "/navigate_to_pose")
        
        # Wait for action server
        server_available = nav_client.wait_for_server(timeout_sec=5.0)
        
        if not server_available:
            pytest.skip("Nav2 not available. Run: ros2 launch nav2_bringup navigation_launch.py")
        
        print("\n✅ Nav2 navigation stack available")
        nav_client.destroy()
    
    def test_navigation_to_goal(self, ros_node):
        """
        E2E Test: Navigate to a goal using Nav2.
        
        This is the full flow:
        1. Get current position
        2. Send navigation goal 1m forward
        3. Wait for completion
        4. Verify robot reached goal
        """
        from nav2_msgs.action import NavigateToPose
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        
        # Create action client
        nav_client = ActionClient(ros_node, NavigateToPose, "/navigate_to_pose")
        
        if not nav_client.wait_for_server(timeout_sec=5.0):
            pytest.skip("Nav2 not available")
        
        # Get current pose
        current_pose = None
        
        def odom_callback(msg):
            nonlocal current_pose
            current_pose = msg.pose.pose
        
        odom_sub = ros_node.create_subscription(
            Odometry,
            "/odom",
            odom_callback,
            10
        )
        
        # Wait for pose
        start_time = time.time()
        while current_pose is None and time.time() - start_time < 3.0:
            rclpy.spin_once(ros_node, timeout_sec=0.1)
        
        assert current_pose is not None, "Could not get robot pose"
        
        initial_x = current_pose.position.x
        initial_y = current_pose.position.y
        
        print(f"\n🚀 Starting navigation E2E test")
        print(f"   Initial position: x={initial_x:.2f}, y={initial_y:.2f}")
        
        # Create goal 1m forward
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = ros_node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = initial_x + 1.0
        goal_msg.pose.pose.position.y = initial_y
        goal_msg.pose.pose.orientation.w = 1.0
        
        print(f"   Goal position: x={goal_msg.pose.pose.position.x:.2f}, y={goal_msg.pose.pose.position.y:.2f}")
        
        # Send goal
        future = nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(ros_node, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        assert goal_handle is not None, "Failed to send goal"
        assert goal_handle.accepted, "Goal was rejected"
        
        print(f"   Goal accepted, waiting for result...")
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        start_time = time.time()
        timeout = 60.0  # 60 second timeout for navigation
        
        while time.time() - start_time < timeout:
            rclpy.spin_once(ros_node, timeout_sec=0.1)
            
            if result_future.done():
                result = result_future.result()
                
                # Status 4 = SUCCEEDED
                if result.status == 4:
                    print(f"   ✅ Navigation succeeded!")
                    break
                else:
                    pytest.fail(f"Navigation failed with status {result.status}")
            
            # Print progress every 5 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0 and current_pose:
                import math
                dist_to_goal = math.sqrt(
                    (current_pose.position.x - (initial_x + 1.0))**2 +
                    (current_pose.position.y - initial_y)**2
                )
                print(f"   ... {elapsed:.0f}s elapsed, distance to goal: {dist_to_goal:.2f}m")
                time.sleep(1)  # Avoid spamming
        else:
            # Cancel goal on timeout
            goal_handle.cancel_goal_async()
            pytest.fail(f"Navigation timed out after {timeout}s")
        
        # Verify final position
        assert current_pose is not None
        
        import math
        final_distance = math.sqrt(
            (current_pose.position.x - (initial_x + 1.0))**2 +
            (current_pose.position.y - initial_y)**2
        )
        
        print(f"   Final position: x={current_pose.position.x:.2f}, y={current_pose.position.y:.2f}")
        print(f"   Distance from goal: {final_distance:.2f}m")
        
        # Should be within 0.3m of goal
        assert final_distance < 0.3, f"Didn't reach goal (distance: {final_distance:.2f}m)"
        
        print(f"\n✅ E2E Navigation test PASSED")
        print(f"   Robot successfully navigated 1m to goal")
        
        odom_sub.destroy()
        nav_client.destroy()


if __name__ == "__main__":
    # Check if Gazebo is running
    try:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )
        if "/clock" not in result.stdout:
            print("⚠️  Warning: Gazebo may not be running")
            print("   Start Gazebo first:")
            print("   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py")
            print("")
    except Exception:
        pass
    
    pytest.main([__file__, "-v", "--tb=short"])
