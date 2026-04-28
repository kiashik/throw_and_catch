from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'my_motion_planner2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
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
            'servo_test_node=my_motion_planner2.servo_test_node:main',
            'servo_test_pose=my_motion_planner2.servo_test_pose:main',
            'my_servo_keyboard2=my_motion_planner2.my_servo_keyboard2:main'
        ],
    },
)
