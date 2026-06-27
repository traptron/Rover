from setuptools import find_packages, setup
import os

package_name = 'teleop_DS4'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/teleop.launch.py']),
        (os.path.join('share', package_name, 'config'), ['config/teleop_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='traptron',
    maintainer_email='traptron@todo.todo',
    description='DualShock 4 teleoperation node for 6-wheel rover (ROS2 Humble)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_node = teleop_DS4.teleop_node:main',
        ],
    },
)
