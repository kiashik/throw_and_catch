from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ashik',
    maintainer_email='ashikislam232@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'opencv_ros_node = vision.opencv_ros_node:main',
        'opencv_ros_node_rs = vision.opencv_ros_node_rs:main',
        'cal_cam_robot = vision.cal_cam_robot:main',
        ],
    },
)
