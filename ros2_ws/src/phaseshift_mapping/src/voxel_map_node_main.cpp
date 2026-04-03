#include "rclcpp/rclcpp.hpp"
#include "phaseshift_mapping/voxel_map_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VoxelMapNode>();

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}