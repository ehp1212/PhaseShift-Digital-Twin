from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_example_map = LaunchConfiguration("use_example_map")
    example_map_name = LaunchConfiguration("example_map_name")

    orchestrator = Node(
        package='phaseshift_system',
        executable='orchestrator_node',
        name='orchestrator',
        output='screen',
        parameters=[
            {"use_example_map": use_example_map},
            {"example_map_name": example_map_name}
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
            "use_example_map",
            default_value="true"
        ),

        DeclareLaunchArgument(
            "example_map_name",
            default_value="demo"
        ),

        orchestrator,
        robot_behaviour_node,
        ])