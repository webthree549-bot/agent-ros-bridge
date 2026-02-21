from setuptools import setup

package_name = 'art_studio'

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
    description='ROS2 Art Studio - Painting robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'painter_node = art_studio.painter_node:main',
            'canvas_node = art_studio.canvas_node:main',
        ],
    },
)