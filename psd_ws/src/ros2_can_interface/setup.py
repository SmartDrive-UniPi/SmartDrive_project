from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'ros2_can_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools','ament_index_python'],
    zip_safe=True,
    maintainer='Edoardo Caciorgna',
    maintainer_email='edo.ca1999@gmail.com',
    description='ROS 2 package for converting ROS 2 messages to CAN messages and vice versa.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2can = ros2_can_interface.ros2can:main',
            'can2ros = ros2_can_interface.can2ros:main',
        ],
    },
)
