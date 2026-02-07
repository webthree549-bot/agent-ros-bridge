#!/usr/bin/env python3
"""Unit Tests for ROS Communicator"""

def test_ros_communicator_init(ros_communicator):
    assert ros_communicator.is_initialized is True