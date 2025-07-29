from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'link6_control' # Use your actual package name here

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anas Houssaini',
    maintainer_email='elhoussainianas8@gmail.com',
    description='ROS 2 control nodes for the Kinova Link 6 robot, including teleoperation.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbox_teleop_node = link6_control.xbox_teleop_node:main'
        ],
    },
)