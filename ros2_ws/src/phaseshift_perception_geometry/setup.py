from setuptools import find_packages, setup

package_name = 'phaseshift_perception_geometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception_geometry.launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eun',
    maintainer_email='eunhyeon1212p@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'voxel_change_detection_node = phaseshift_perception_geometry.voxel_change_detection_node:main',
            'voxel_costmap_node = phaseshift_perception_geometry.voxel_costmap_node:main',
        ],
    },
)
