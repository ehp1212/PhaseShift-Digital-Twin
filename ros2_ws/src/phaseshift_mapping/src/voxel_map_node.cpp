#include "phaseshift_mapping/voxel_map_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

    save_service_ = create_service<phaseshift_interfaces::srv::SaveVoxelMap>(
        "/system/save_voxel_map",
        std::bind(
            &VoxelMapNode::handleSaveMap,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
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

    // ------------------------------
    // Preprocess
    // ------------------------------
    preprocess_input_cloud(cloud);
    if (cloud.empty()) return;

    // ------------------------------
    // TF Transform
    // ------------------------------
    pcl::PointCloud<pcl::PointXYZ> transformed;
    if (!transform_to_map(cloud, transformed, msg->header.frame_id))
    {
        RCLCPP_WARN(get_logger(), "TF transform failed");
        return;
    }

    // ------------------------------
    // Pose / Motion gating
    // ------------------------------
    if (!is_motion_stable(msg->header.stamp))
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Skip scan: unstable motion");
            return;
    }

    // ------------------------------
    // Voxel map
    // ------------------------------
    update_voxel_map(transformed, rclcpp::Time(msg->header.stamp));

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
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const rclcpp::Time& stamp)
{

    current_frame_voxels_.clear();

    // ------------------------
    // accumulate points
    // ------------------------
    for (const auto& pt : cloud.points)
    {
        VoxelKey key = pointToVoxelKey(pt.x, pt.y, pt.z);

        auto& cell = voxel_map_[key];

        // add point hit count
        cell.point_hits = std::min(cell.point_hits + 1, 100u);
        cell.last_seen_time = stamp;

        // centroid calculation
        cell.cx = (key.x + 0.5f) * voxel_resolution_;
        cell.cy = (key.y + 0.5f) * voxel_resolution_;
        cell.cz = (key.z + 0.5f) * voxel_resolution_;

        current_frame_voxels_.insert(key);
    }

    // ------------------------
    // frame hit
    // ------------------------
    for (const auto& key : current_frame_voxels_)
    {
        auto& cell = voxel_map_[key];
        cell.frame_hits = std::min(cell.frame_hits + 1, stable_frame_threhold_);
    }

    // ------------------------
    // confidence
    // ------------------------
    for (auto& [key, cell] : voxel_map_)
    {
        cell.confidence = std::min(
            static_cast<float>(cell.frame_hits) / static_cast<float>(stable_frame_threhold_),
            1.0f
        );
    }
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

        transform = tf_buffer_->lookupTransform(
            "map",
            source_frame,
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

pcl::PointCloud<pcl::PointXYZI> VoxelMapNode::buildFilteredCloud()
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.reserve(voxel_map_.size());

    for (const auto& [key, cell] : voxel_map_)
    {
        // minimum frame hit
        if (cell.frame_hits <stable_frame_threhold_) continue;

        // minimum confidence
        if (cell.confidence < 0.3f) continue;

        // neighbor filtering
        int neighbor_count = 0;

        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dz = -1; dz <= 1; ++dz)
                {
                    if (dx == 0 && dy == 0 && dz == 0) continue;

                    VoxelKey nk{key.x + dx, key.y + dy, key.z + dz};

                    auto it = voxel_map_.find(nk);
                    if (it != voxel_map_.end() &&
                        it->second.frame_hits >= stable_frame_threhold_)
                    {
                        neighbor_count++;
                    }
                }
            }
        }

        if (neighbor_count < 1) continue;

        pcl::PointXYZI pt;

        pt.x = cell.cx;
        pt.y = cell.cy;
        pt.z = cell.cz;

        pt.intensity = cell.confidence;

        cloud.points.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;

    return cloud;
}

void VoxelMapNode::preprocess_input_cloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    // NAN 
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cloud, cloud, indices);

    if (cloud.empty()) return;

    // Input downsampling
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud.makeShared());
    vg.setLeafSize(0.05f, 0.05f, 0.05f);

    pcl::PointCloud<pcl::PointXYZ> down;
    vg.filter(down);

    cloud = down;
    if (cloud.empty()) return;

    // Z filtering
    pcl::PointCloud<pcl::PointXYZ> filtered;
    filtered.reserve(cloud.size());

    for (const auto& p : cloud.points)
    {
        double r = std::sqrt(p.x * p.x + p.y * p.y);
        if (r > 20.0) continue;
        if (p.z < -1.0 || p.z > 2.0) continue;

        filtered.push_back(p);
    }

    cloud = filtered;
}

bool VoxelMapNode::is_motion_stable(const builtin_interfaces::msg::Time& stamp)
{
    geometry_msgs::msg::TransformStamped tf;

    try
    {
        tf = tf_buffer_->lookupTransform(
            "map",
            "base_link",
            stamp,
            tf2::durationFromSec(0.1));
    }
    catch(tf2::TransformException& ex)
    {
        RCLCPP_WARN(get_logger(), "Motion TF fall: %s", ex.what());
        return false;
    }

    double x = tf.transform.translation.x;
    double y = tf.transform.translation.y;

    auto& q = tf.transform.rotation;
    double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    );

    static bool initialized = false;
    static double last_x, last_y, last_yaw;

    if (!initialized)
    {
        last_x = x;
        last_y = y;
        last_yaw = yaw;
        initialized = true;
        return true;
    }

    double dx = x - last_x;
    double dy = y - last_y;
    double dist = std::sqrt(dx * dx + dy * dy);

    double dyaw = std::fabs(yaw - last_yaw);

    last_x = x;
    last_y = y;
    last_yaw = yaw;

    if (dist > 0.03) return false;
    if (dyaw > 0.05) return false;

    return true;
}

// ------------------------------
// MAP MANAGEMENT
// ------------------------------
void VoxelMapNode::saveMap(const std::string& path)
{
    auto cloud = buildFilteredCloud();

    pcl::io::savePCDFileBinary(path, cloud);

    RCLCPP_INFO(this->get_logger(),
        "Save voxel map: %s", path.c_str());
}

void VoxelMapNode::handleSaveMap(
    const std::shared_ptr<phaseshift_interfaces::srv::SaveVoxelMap::Request> request,
    std::shared_ptr<phaseshift_interfaces::srv::SaveVoxelMap::Response> response)
{
    try
    {
        saveMap(request->path);
        response->success = true;

        RCLCPP_INFO(this->get_logger(),
            "Map saved via service: %s", request->path.c_str());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),
            "Failed to save map: %s", e.what());

        response->success = false;
    }
}