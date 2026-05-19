import glob

from setuptools import find_packages, setup

package_name = 'ball_catch_predictor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Linear-drag Kalman filter and catch-point predictor for a tossed ball.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'catch_predictor = ball_catch_predictor.catch_predictor_node:main',
            'catch_predictor_v3 = ball_catch_predictor.catch_predictor_nodeV3:main',
        ],
    },
)
