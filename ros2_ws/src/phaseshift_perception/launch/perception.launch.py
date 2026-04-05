from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    yolo_detector_node = Node(
        package='phaseshift_perception',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen'
    )

    yolo_tracker_node = Node(
        package='phaseshift_perception',
        executable='yolo_tracker_node',
        name='yolo_tracker_node',
        output='screen'
    )

    projection_node = Node(
        package='phaseshift_perception',
        executable='projection_node',
        name='projection_node',
        output='screen'
    )

    detection_memory_node = Node(
        package='phaseshift_perception',
        executable='detection_memory_node',
        name='detection_memory_node',
        output='screen'
    )

    detection_nav_adapter = Node(
        package='phaseshift_perception',
        executable='detection_nav_adapter',
        name='detection_nav_adapter',
        output='screen'
    )    

    voxel_state_estimator_node = Node(
        package='phaseshift_perception',
        executable='voxel_state_estimator_node',
        name='voxel_state_estimator_node',
        output='screen'
    )    

    return LaunchDescription([
        yolo_detector_node,
        yolo_tracker_node,
        projection_node,
        detection_memory_node,
        detection_nav_adapter,
        voxel_state_estimator_node
    ])