#include "phaseshift_mapping/voxel_map_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

using CallbackReturn =
                rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

VoxelMapNode::VoxelMapNode()
: rclcpp_lifecycle::LifecycleNode("voxel_map_node"),
  active_(false)
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "VoxelMapNode created");
}


// ------------------------------
// CONFIGURE
// ------------------------------
CallbackReturn VoxelMapNode::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "[Lifecycle] Configuring...");

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points",
        rclcpp::SensorDataQoS(),
        std::bind(&VoxelMapNode::pointcloud_callback, this, std::placeholders::_1)
    );

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/voxel_map",
        rclcpp::SystemDefaultsQoS()
    );  

    return CallbackReturn::SUCCESS;
}

// ------------------------------
// ACTIVATE
// ------------------------------
CallbackReturn VoxelMapNode::on_activate(const rclcpp_lifecycle::State &)
{
    pub_->on_activate();
    active_ = true;
    RCLCPP_INFO(get_logger(), "[Lifecycle] Activated");
    return CallbackReturn::SUCCESS;
}


// ------------------------------
// on_deactivate
// ------------------------------
CallbackReturn VoxelMapNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    pub_->on_deactivate();
    active_ = false;
    RCLCPP_INFO(get_logger(), "[Lifecycle] Deactivated");
    return CallbackReturn::SUCCESS;
}

// ------------------------------
// callback
// ------------------------------
void VoxelMapNode::pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!active_) return;

    // ROS -> PCL
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    if (cloud.empty()) return;

    // TF Transform
    pcl::PointCloud<pcl::PointXYZ> transformed;
    if (!transform_to_map(cloud, transformed, msg->header.frame_id))
    {
        RCLCPP_WARN(get_logger(), "TF transform failed");
        return;
    }

    // Voxel map
    update_voxel_map(transformed);

    auto filtered_cloud = buildFilteredCloud();

    // PCL -> ROS
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(filtered_cloud, output_msg);

    output_msg.header.frame_id = "map";
    output_msg.header.stamp = msg->header.stamp;

    pub_->publish(output_msg);
}


// ------------------------------
// VOXEL MAP UPDATE
// ------------------------------
void VoxelMapNode::update_voxel_map(
    const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    rclcpp::Time now = this->get_clock()->now();

    current_frame_voxels_.clear();

    // Get current frame voxel
    for (const auto& pt : cloud.points)
    {
        VoxelKey key = pointToVoxelKey(pt.x, pt.y, pt.z);

        auto& cell = voxel_map_[key];

        cell.point_hits = std::min(cell.point_hits + 1, 100u);
        cell.last_seen_time = now;

        current_frame_voxels_.insert(key);
    }

    // Increase hit count per frame
    for (const auto& key : current_frame_voxels_)
    {
        auto& cell = voxel_map_[key];
        cell.frame_hits = std::min(cell.frame_hits + 1, stable_frame_threhold_);
    }

    // Calculate confidence
    for (auto& [key, cell] : voxel_map_)
    {
        cell.confidence = std::min(
            static_cast<float>(cell.frame_hits) / static_cast<float>(stable_frame_threhold_),
            1.0f
        );
    }

    size_t high_conf = 0;
    for (const auto& [key, cell] : voxel_map_)
    {
        if (cell.confidence > 0.8f)
            high_conf++;
    }


    RCLCPP_INFO(this->get_logger(),
        "Total: %ld | High confidence: %ld",
        voxel_map_.size(), high_conf);
}

bool VoxelMapNode::transform_to_map(
const pcl::PointCloud<pcl::PointXYZ>& input,
pcl::PointCloud<pcl::PointXYZ>& output,
const std::string& source_frame)
{
    geometry_msgs::msg::TransformStamped transform;

    try
    {
        if (!tf_buffer_->canTransform(
                "map",
                source_frame,
                tf2::TimePointZero,
                tf2::durationFromSec(0.1)))
        {
            RCLCPP_WARN(get_logger(), "TF not ready yet");
            return false;
        }

        // rclcpp::Time now = this->get_clock()->now();
        transform = tf_buffer_->lookupTransform(
            "map",
            source_frame,
            // now
            tf2::TimePointZero,
            tf2::durationFromSec(0.1)  
        );
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(get_logger(), "TF error: %s", ex.what());
        return false;
    }
    
    // ------------------------
    // transform matrix
    // ------------------------
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();

    const auto& t = transform.transform.translation;
    const auto& q = transform.transform.rotation;

    Eigen::Quaternionf quat(q.w, q.x, q.y, q.z);
    Eigen::Matrix3f rot = quat.toRotationMatrix();

    mat.block<3,3>(0,0) = rot;
    mat(0,3) = t.x;
    mat(1,3) = t.y;
    mat(2,3) = t.z;

    // ------------------------
    // point
    // ------------------------
    pcl::transformPointCloud(input, output, mat);

    return true;
}

pcl::PointCloud<pcl::PointXYZ> VoxelMapNode::buildFilteredCloud()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (const auto& [key, cell] : voxel_map_)
    {
        if (!isHighConfidence(cell)) continue;

        pcl::PointXYZ pt;

        pt.x = key.x * voxel_resolution_;
        pt.y = key.y * voxel_resolution_;
        pt.z = key.z * voxel_resolution_;

        cloud.points.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;

    return cloud;
}