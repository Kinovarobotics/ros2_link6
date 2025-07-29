import os
from glob import glob
from setuptools import setup

package_name = 'link6_bringup'

setup(
    name=package_name,
    version='0.0.1',

    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anas Houssaini',
    maintainer_email='elhoussainianas8@gmail.com',
    description='Bringup launch files for the Link6 robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)