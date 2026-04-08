#include "phaseshift_perception_geometry/voxel_perception_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VoxelPerceptionNode>();
    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}