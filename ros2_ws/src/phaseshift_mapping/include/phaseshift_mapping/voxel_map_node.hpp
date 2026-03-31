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

// Map 
#include "phaseshift_interfaces/srv/save_voxel_map.hpp"

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

        pcl::PointCloud<pcl::PointXYZI> buildFilteredCloud();

        // Map 
        void saveMap(const std::string& path);
        rclcpp::Service<phaseshift_interfaces::srv::SaveVoxelMap>::SharedPtr save_service_;
        void handleSaveMap(const std::shared_ptr<phaseshift_interfaces::srv::SaveVoxelMap::Request> request,
                std::shared_ptr<phaseshift_interfaces::srv::SaveVoxelMap::Response> response);

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

        struct VoxelCell
        {
                uint32_t point_hits = 0;
                uint32_t frame_hits = 0;
                float confidence = 0.0f;
                rclcpp::Time last_seen_time{0, 0, RCL_ROS_TIME};
        };

private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::unordered_map<VoxelKey, VoxelCell, VoxelKeyHash> voxel_map_;
        std::unordered_set<VoxelKey, VoxelKeyHash> current_frame_voxels_;

        bool active_{false};
        double voxel_resolution_{0.15};
        uint32_t stable_frame_threhold_{5};

        VoxelKey pointToVoxelKey(float x, float y, float z)
        {
                return VoxelKey{
                        static_cast<int>(std::floor(x / voxel_resolution_)),
                        static_cast<int>(std::floor(y / voxel_resolution_)),
                        static_cast<int>(std::floor(z / voxel_resolution_))
                };
        }

        bool isHighConfidence(const VoxelCell& cell) const
        {
                return cell.confidence > 0.8f;
        }
};