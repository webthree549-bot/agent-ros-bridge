#!/usr/bin/env python3
"""Pytest Configuration"""
import os
os.environ["MOCK_MODE"] = "true"
os.environ["ROS_TYPE"] = "ros2"
os.environ["ROS_DISTRO"] = "humble"