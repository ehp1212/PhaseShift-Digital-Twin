from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'phaseshift_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),

        (os.path.join('share', package_name, 'models'),
         glob('models/yolov8n.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eun',
    maintainer_email='eun1212p@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detector_node = phaseshift_perception.yolo_detector_node:main',
            'projection_node = phaseshift_perception.projection_node:main',
            'detection_nav_adapter = phaseshift_perception.detection_nav_adapter:main',
        ],
    },
)
