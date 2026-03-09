"""
Setup script for agent_ros_bridge package.

This is used by ROS2 colcon build system.
"""

from setuptools import setup, find_packages
import os

package_name = 'agent_ros_bridge'

setup(
    name=package_name,
    version='0.6.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/intent_parser.launch.py',
            'launch/context_manager.launch.py',
            'launch/ai_layer.launch.py',
        ]),
    ],
    install_requires=[
        'setuptools',
        'pydantic>=2.0.0',
        'pyyaml>=6.0',
        'websockets>=11.0',
        'cryptography>=41.0.0',
        'aiohttp>=3.8.0',
        'aiosqlite>=0.19.0',
        'redis>=4.5.0',
    ],
    zip_safe=True,
    maintainer='Agent ROS Bridge Contributors',
    maintainer_email='maintainer@agent-ros-bridge.org',
    description='Universal interface for AI agents to control ROS robots via natural language',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Main CLI
            'agent-ros-bridge = agent_ros_bridge.cli:main',
            'arb = agent_ros_bridge.cli:main',
            
            # AI Layer Nodes (Week 2)
            'intent_parser = agent_ros_bridge.ai.intent_parser:main',
            'context_manager = agent_ros_bridge.ai.context_manager:main',
        ],
    },
)
