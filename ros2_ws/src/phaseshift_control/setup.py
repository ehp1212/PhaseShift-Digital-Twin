from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'phaseshift_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/control.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2.yaml']),
        ('share/' + package_name + '/config', ['config/slam.yaml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jimmy',
    maintainer_email='eunhyeon1212p@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_node = phaseshift_control.odometry_node:main',
            'slam_test = phaseshift_control.test_slam_controller:main',
        ],
    },
)
