#pragma once

#include <unordered_set>
#include <cmath>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <unordered_set>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class VoxelMapNode : public rclcpp_lifecycle::LifecycleNode
{
public:
        explicit VoxelMapNode();
private:            
        using CallbackReturn =
                rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


        // lifecycle callbacks
        CallbackReturn on_configure(const rclcpp_lifecycle::State &);
        CallbackReturn on_activate(const rclcpp_lifecycle::State &);
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

        // callback
        void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void update_voxel_map(const pcl::PointCloud<pcl::PointXYZ>& cloud);
        bool transform_to_map(const pcl::PointCloud<pcl::PointXYZ>& intput,
        pcl::PointCloud<pcl::PointXYZ>& output,
        const std::string& source_frame);

        // voxel key
        struct VoxelKey
        {
                int x, y, z;

                bool operator==(const VoxelKey& other) const
                {
                return x == other.x && y == other.y && z == other.z;
                }
        };

        struct VoxelKeyHash
        {
                std::size_t operator()(const VoxelKey& k) const
                {
                return ((std::hash<int>()(k.x) ^
                        (std::hash<int>()(k.y) << 1)) >> 1) ^
                        (std::hash<int>()(k.z) << 1);
                }
        };

private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::unordered_set<VoxelKey, VoxelKeyHash> voxel_set_;
        bool active_{false};
};