from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    orchestrator = Node(
        package='phaseshift_system',
        executable='orchestrator_node',
        name='orchestrator',
        output='screen'
    )

    return LaunchDescription([orchestrator])