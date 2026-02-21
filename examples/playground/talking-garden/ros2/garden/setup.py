from setuptools import setup

package_name = 'garden'

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
    description='ROS2 Talking Garden - Plant sensors and poet',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plant_node = garden.plant_node:main',
            'gardener_node = garden.gardener_node:main',
            'oracle_node = garden.oracle_node:main',
        ],
    },
)