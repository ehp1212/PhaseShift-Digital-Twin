from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'phaseshift_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # urdf files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),

        # demo map
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
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
            'costmap_adapter_node = phaseshift_bringup.costmap_adapter_node:main',
        ],
    },
)
