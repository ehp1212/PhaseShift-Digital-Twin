from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    voxel_costmap_node = Node(
        package='phaseshift_perception_geometry',
        executable='voxel_costmap_node.py',
        name='voxel_costmap_node',
        output='screen'
    )

    voxel_perception_node = Node(
        package='phaseshift_perception_geometry',
        executable='voxel_perception_node',
        name='voxel_perception_node',
        output='screen'
    )

    return LaunchDescription([
       voxel_costmap_node,
       voxel_perception_node
    ])