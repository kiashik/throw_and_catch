from setuptools import find_packages, setup

import os
from glob import glob


package_name = 'move_arm_to'

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
            "move_arm_to = move_arm_to.move_arm_to:main",
            "my_omy_moveit = move_arm_to.my_omy_moveit:main",
            "omy_f3m_hello_moveit = move_arm_to.omy_f3m_hello_moveit:main",
            "catch_ball_servo = move_arm_to.catch_ball_servo:main",
            "catch_ball_simple = move_arm_to.catch_ball_simple:main",
            "get_tag_pose = move_arm_to.get_tag_pose:main",
            "average_pose_publisher = move_arm_to.average_pose_publisher:main",
            "convert_pose_cam_to_rob = move_arm_to.convert_pose_cam_to_rob:main",
            "track_april_tag = move_arm_to.track_april_tag:main",
            "fake_ball_pose_publisher = move_arm_to.fake_ball_pose_publisher:main",

        ],
    },
)
