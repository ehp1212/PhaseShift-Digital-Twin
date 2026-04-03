from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    project_name = LaunchConfiguration("project_name")

    orchestrator = Node(
        package='phaseshift_system',
        executable='orchestrator_node',
        name='orchestrator',
        output='screen',
        parameters=[
            {"project_name": project_name}
        ]
    )

    robot_behaviour_node = Node(
        package='phaseshift_system',
        executable='robot_behaviour_node',
        name='robot_behaviour_node',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "project_name",
            default_value="demo"
        ),

        orchestrator,
        robot_behaviour_node,
        ])