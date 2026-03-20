from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    yolo_node = Node(
        package='phaseshift_perception',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen'
    )

    projection_node = Node(
        package='phaseshift_perception',
        executable='projection_node',
        name='projection_node',
        output='screen'
    )

    return LaunchDescription([
        yolo_node,
        projection_node,
    ])