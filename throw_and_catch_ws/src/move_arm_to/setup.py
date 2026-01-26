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
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cpsquare',
    maintainer_email='tomagrundler@icloud.com',
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
            "move_to_pose = move_arm_to.move_to_pose:main",

        ],
    },
)
