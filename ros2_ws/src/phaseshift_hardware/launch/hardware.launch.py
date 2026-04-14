from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    pkg_hardware = get_package_share_directory('phaseshift_hardware')

    robot_description = LaunchConfiguration('robot_description')

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            os.path.join(pkg_hardware, 'config', 'ros2_control.yaml')
        ]
    )

    spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    delayed_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[spawner]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_description',
            default_value=''
        ),
        ros2_control_node,
        delayed_spawner
    ])