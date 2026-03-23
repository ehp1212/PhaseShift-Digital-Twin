
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
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('phaseshift_control'),
                'launch',
                'control.launch.py'
            )
        )
    )

    # ==========================
    # OccupancyGrid Adapter
    # ==========================
    costmap_node = Node(
        package="phaseshift_bringup",
        executable="costmap_adapter_node",
        name="costmap_adapter",
        output="screen"
    )

    # ==========================
    # Perception Layer
    # ==========================
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('phaseshift_perception'),
                'launch',
                'perception.launch.py'
            )
        )
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
        costmap_node,
        nav_launch,
        perception_launch,
        orchestrator
    ])