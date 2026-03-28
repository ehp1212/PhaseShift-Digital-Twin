from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    orchestrator = Node(
        package='phaseshift_system',
        executable='orchestrator_node',
        name='orchestrator',
        output='screen'
    )

    robot_behaviour_node = Node(
        package='phaseshift_system',
        executable='robot_behaviour_node',
        name='robot_behaviour_node',
        output='screen'
    )

    return LaunchDescription([
        orchestrator,
        robot_behaviour_node,
        ])