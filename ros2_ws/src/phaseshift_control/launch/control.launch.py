
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_bringup = get_package_share_directory('phaseshift_control')

    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'false',
            'params_file': 
            os.path.join(pkg_bringup, 'config', 'nav2.yaml'),
            'map': ''
        }.items()
    )

    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'localization_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'false',
            'params_file': os.path.join(pkg_bringup, 'config', 'nav2.yaml'),
            'map': ''  # dummy
        }.items()
    )

    return LaunchDescription([
        nav2_navigation_launch,
        nav2_localization_launch,
    ])