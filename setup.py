from setuptools import setup
import os
from glob import glob

package_name = 'slalom_simulator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='2D Slalom Simulator with ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator_node = slalom_simulator.simulator_node:main',
            'imu1_localization_node = slalom_simulator.imu1_localization_node:main',
            'imu2_localization_node = slalom_simulator.imu2_localization_node:main',
            'kalman_localization_node = slalom_simulator.kalman_localization_node:main',
            'controller_node = slalom_simulator.controller_node:main',
        ],
    },
)
