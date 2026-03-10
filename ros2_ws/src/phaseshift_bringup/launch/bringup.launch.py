
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_bringup = get_package_share_directory('phaseshift_bringup')

    # ==========================
    # URDF
    # ==========================
    urdf_file = os.path.join(pkg_bringup, 'urdf', 'phaseshift_robot.urdf')
    robot_description = Command(['xacro ', urdf_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )
    
    odom_node = Node(
        package='phaseshift_control',
        executable='odometry_node',
        name='odometry_node',
        output='screen'
    )

    # ==========================
    # Nav2 
    # ==========================
    nav2_launch = IncludeLaunchDescription(
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
            # os.path.join(pkg_bringup, 'config', 'nav2.yaml'
            os.path.join(pkg_bringup, 'config', 'nav2.yaml'),
            'map': ''
        }.items()
    )

    nav2_local_launch = IncludeLaunchDescription(
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

    # ==========================
    # Orchestrator
    # ==========================
    orchestrator = Node(
        package='phaseshift_system',
        executable='orchestrator_node',
        name='orchestrator',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        odom_node,
        nav2_launch,
        nav2_local_launch,
        orchestrator
    ])