import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'phaseshift_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/phaseshift_test/config', glob('config/*.yaml')),
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
            'scenario_runner_node = phaseshift_test.scenario_runner_node:main',
            'latency_monitor_node = phaseshift_test.latency_monitor_node:main',
        ],
    },
)
