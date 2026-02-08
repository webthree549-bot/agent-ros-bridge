from setuptools import setup, find_packages
import os

package_name = 'openclaw_ros_bridge'
here = os.path.abspath(os.path.dirname(__file__))

# Load long description from README.md (gracefully handle missing file)
readme_path = os.path.join(here, 'README.md')
if os.path.exists(readme_path):
    with open(readme_path, encoding='utf-8') as f:
        long_description = f.read()
else:
    long_description = 'Universal ROS1/ROS2 + OpenClaw Adaptive Bridge Framework for Embodied Intelligence'

# Load requirements from requirements.txt (gracefully handle missing file)
requirements_path = os.path.join(here, 'requirements.txt')
if os.path.exists(requirements_path):
    with open(requirements_path, encoding='utf-8') as f:
        requirements = [line.strip() for line in f if line.strip() and not line.startswith('#')]
else:
    requirements = []  # Will use package defaults

# Build data_files safely
launch_files = []
if os.path.isdir('launch'):
    launch_files = [os.path.join('launch', f) for f in os.listdir('launch') if f.endswith('.launch.py')]

config_files = []
if os.path.isdir('config'):
    config_files = [os.path.join('config', f) for f in os.listdir('config') if f.endswith('.yaml')]

greenhouse_files = []
if os.path.isdir('demo/greenhouse'):
    greenhouse_files = [os.path.join('demo/greenhouse', f) for f in os.listdir('demo/greenhouse') if f.endswith(('.py', '.yaml'))]

arm_files = []
if os.path.isdir('demo/arm_manipulation'):
    arm_files = [os.path.join('demo/arm_manipulation', f) for f in os.listdir('demo/arm_manipulation') if f.endswith(('.py', '.yaml'))]

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), launch_files),
        (os.path.join('share', package_name, 'config'), config_files),
        (os.path.join('share', package_name, 'demo/greenhouse'), greenhouse_files),
        (os.path.join('share', package_name, 'demo/arm_manipulation'), arm_files),
    ],
    install_requires=requirements,
    zip_safe=True,
    maintainer='OpenClaw-ROS Dev Team',
    maintainer_email='dev@openclaw-ros.org',
    description='Universal ROS1/ROS2 + OpenClaw Adaptive Bridge Framework for Embodied Intelligence',
    long_description=long_description,
    long_description_content_type='text/markdown',
    license='MIT',
    tests_require=['pytest', 'pytest-cov', 'pytest-benchmark', 'mock'],
    entry_points={
        'console_scripts': [
            # Core Framework
            'version_manager = openclaw_ros_bridge.version.version_manager:main',
            'performance_monitor = openclaw_ros_bridge.monitor.performance_monitor:main',
            'fault_recovery_manager = openclaw_ros_bridge.fault.recovery_manager:main',
            'openclaw_tcp_server = openclaw_ros_bridge.communication.openclaw_tcp_server:main',
            # Demos
            'greenhouse_plugin = demo.greenhouse.greenhouse_plugin:main',
            'arm_manipulation_plugin = demo.arm_manipulation.arm_plugin:main',
        ],
    },
    keywords=['ROS', 'ROS1', 'ROS2', 'OpenClaw', 'Embodied Intelligence', 'HAL', 'Robotics', 'AI'],
    classifiers=[
        'Intended Audience :: Developers',
        'Intended Audience :: Robotics Researchers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.10',
        'Topic :: Robotics',
        'Topic :: Artificial Intelligence',
        'Topic :: Embedded Systems',
    ],
)