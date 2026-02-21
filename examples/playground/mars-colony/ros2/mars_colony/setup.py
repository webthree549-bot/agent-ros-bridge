from setuptools import setup

package_name = 'mars_colony'

setup(
    name=package_name,
    version='0.3.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenClaw',
    maintainer_email='dev@openclaw.ai',
    description='ROS2 Mars Colony - Robot fleet',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'excavator_node = mars_colony.excavator_node:main',
            'solar_drone_node = mars_colony.solar_drone_node:main',
            'builder_node = mars_colony.builder_node:main',
            'scout_node = mars_colony.scout_node:main',
            'command_node = mars_colony.command_node:main',
        ],
    },
)