from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    voxel_costmap_node = Node(
        package='phaseshift_perception_geometry',
        executable='voxel_costmap_node',
        name='voxel_costmap_node',
        output='screen'
    )

    return LaunchDescription([
       voxel_costmap_node
       ])