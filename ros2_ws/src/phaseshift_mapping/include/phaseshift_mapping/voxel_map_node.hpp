#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Map
#include "phaseshift_interfaces/srv/save_voxel_map.hpp"

// Motion-aware, confidence-driven voxel mapping pipeline 
// for stable spatial representation in dynamic environments

// LiDAR → Preprocessing → Motion Gate → Voxel Accumulation → Confidence Filtering → Voxel Map

class VoxelMapNode : public rclcpp_lifecycle::LifecycleNode
{
public:
        explicit VoxelMapNode();
private:            
        using CallbackReturn =
                rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

        // ------------------------------
        // lifecycle callbacks
        // ------------------------------
        CallbackReturn on_configure(const rclcpp_lifecycle::State &);
        CallbackReturn on_activate(const rclcpp_lifecycle::State &);
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

        // ------------------------------
        // callback
        // ------------------------------
        void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        // ------------------------------
        // pipeline stages
        // ------------------------------
        void preprocess_input_cloud(pcl::PointCloud<pcl::PointXYZ> & cloud);

        bool transform_to_map(const pcl::PointCloud<pcl::PointXYZ>& intput,
                pcl::PointCloud<pcl::PointXYZ>& output,
                const std::string& source_frame);
        
        bool is_motion_stable(const builtin_interfaces::msg::Time & stamp);
        void update_voxel_map(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                const rclcpp::Time& stamp);

        pcl::PointCloud<pcl::PointXYZI> buildFilteredCloud();

        // ------------------------------
        // Map 
        // ------------------------------
        void saveMap(const std::string& path);
        void handleSaveMap(const std::shared_ptr<phaseshift_interfaces::srv::SaveVoxelMap::Request> request,
                std::shared_ptr<phaseshift_interfaces::srv::SaveVoxelMap::Response> response);

        // ------------------------------
        // utils
        // ------------------------------
        float getYawFromQuaternion(double x, double y, double z, double w) const;

        int countConfirmedNeighbors(const struct VoxelKey & key) const;

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
                std::size_t operator()(const VoxelKey & k) const
                {
                std::size_t h1 = std::hash<int>()(k.x);
                std::size_t h2 = std::hash<int>()(k.y);
                std::size_t h3 = std::hash<int>()(k.z);
                return h1 ^ (h2 << 1) ^ (h3 << 2);
                }
        };

        struct VoxelCell
        {
                uint32_t point_hits = 0;
                uint32_t frame_hits = 0;

                float confidence = 0.0f;

                rclcpp::Time last_seen_time{0, 0, RCL_ROS_TIME};

                // voxel center position
                float cx = 0.0f;
                float cy = 0.0f;
                float cz = 0.0f;

                bool seen_in_current_frame = false;
        };

        // ------------------------------
        // voxel helpers
        // ------------------------------
        VoxelKey pointToVoxelKey(float x, float y, float z) const
        {
                return VoxelKey{
                static_cast<int>(std::floor(x / voxel_resolution_)),
                static_cast<int>(std::floor(y / voxel_resolution_)),
                static_cast<int>(std::floor(z / voxel_resolution_))
                };
        }

        bool isConfirmed(const VoxelCell & cell) const
        {
                return cell.frame_hits >= stable_frame_threshold_;
        }

        bool isHighConfidence(const VoxelCell & cell) const
        {
                return cell.confidence >= min_confidence_threshold_;
        }

private:
        // ------------------------------
        // ROS interfaces
        // ------------------------------
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
        rclcpp::Service<phaseshift_interfaces::srv::SaveVoxelMap>::SharedPtr save_service_;

        // ------------------------------
        // TF
        // ------------------------------
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // ------------------------------
        // voxel storage
        // ------------------------------
        std::unordered_map<VoxelKey, VoxelCell, VoxelKeyHash> voxel_map_;
        std::unordered_set<VoxelKey, VoxelKeyHash> current_frame_voxels_;

        // ------------------------------
        // state
        // ------------------------------    
        bool active_{false};

        // ------------------------------
        // mapping params
        // ------------------------------
        double voxel_resolution_{0.15};

        uint32_t stable_frame_threhold_{5};

        // preprocess
        double input_leaf_size_{0.05};
        double max_range_{20.0};
        double min_z_{-1.0};
        double max_z_{2.0};

        // motion gating
        bool pose_initialized_{false};
        double last_x_{0.0};
        double last_y_{0.0};
        double last_yaw_{0.0};

        double motion_translation_threshold_{0.03};   // meters
        double motion_yaw_threshold_{0.05};           // radians

        // voxel confirmation / confidence
        uint32_t stable_frame_threshold_{5};
        uint32_t max_point_hits_{30};
        float min_confidence_threshold_{0.3f};

        // postprocess
        int min_neighbor_count_{1};
};