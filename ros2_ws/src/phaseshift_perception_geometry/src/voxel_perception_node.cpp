#include "phaseshift_perception_geometry/voxel_perception_node.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ============================================================
// Constructor
// ============================================================

VoxelPerceptionNode::VoxelPerceptionNode()
: rclcpp_lifecycle::LifecycleNode("voxel_perception_node")
{
    RCLCPP_INFO(get_logger(), "[VoxelPerceptionNode] Created");

    // -----------------------------
    // Parameters
    // -----------------------------
    this->declare_parameter("voxel_resolution", 0.15);
    this->declare_parameter("voxel_map_path", "");
    this->declare_parameter("max_detection_range", 5.0);

    this->declare_parameter<std::string>("input_topic", "/velodyne_points");
    this->declare_parameter<std::string>("change_points_topic", "/perception_geometry/change_points");
    this->declare_parameter<std::string>("change_points_info_topic", "/perception_geometry/change_points_info");
    this->declare_parameter<std::string>("live_voxel_topic", "/perception_geometry/live_voxel_points");
    this->declare_parameter<std::string>("target_frame", "map");

    this->declare_parameter("persistence_threshold", 3);
    this->declare_parameter("static_confidence_threshold", 0.6);

    // TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

// ============================================================
// Utility: load static voxel confidence map
// ============================================================

bool VoxelPerceptionNode::load_voxel_map(const std::string & path)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(path, cloud) == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to load PCD: %s", path.c_str());
        return false;
    }

    static_confidence_map_.clear();
    static_confidence_map_.reserve(cloud.points.size());

    for (const auto & pt : cloud.points) {
        VoxelKey key = make_voxel_key(pt.x, pt.y, pt.z);
        float conf = std::clamp(pt.intensity, 0.0f, 1.0f);
        static_confidence_map_[key] = conf;
    }

    RCLCPP_INFO(
        get_logger(),
        "Loaded static confidence map: %zu voxels",
        static_confidence_map_.size());

    return true;
}

// ============================================================
// Utility: static confidence lookup (3x3x3 neighborhood max)
// ============================================================

float VoxelPerceptionNode::get_static_confidence(const VoxelKey & key) const
{
    float best_conf = 0.0f;

    for (int dx = -1; dx <= 1; dx++)
    for (int dy = -1; dy <= 1; dy++)
    for (int dz = -1; dz <= 1; dz++) {
        VoxelKey neighbor{key.x + dx, key.y + dy, key.z + dz};

        auto it = static_confidence_map_.find(neighbor);
        if (it != static_confidence_map_.end()) {
            best_conf = std::max(best_conf, it->second);
        }
    }

    return best_conf;
}

// ============================================================
// Utility: neighborhood existence
// ============================================================

bool VoxelPerceptionNode::has_neighbour(
    const VoxelKey & key,
    const std::unordered_set<VoxelKey, VoxelKeyHash> & set) const
{
    for (int dx = -1; dx <= 1; dx++)
    for (int dy = -1; dy <= 1; dy++)
    for (int dz = -1; dz <= 1; dz++) {
        if (dx == 0 && dy == 0 && dz == 0) {
            continue;
        }

        VoxelKey neighbor{key.x + dx, key.y + dy, key.z + dz};
        if (set.find(neighbor) != set.end()) {
            return true;
        }
    }

    return false;
}

// ============================================================
// Utility: count neighbors
// ============================================================

uint16_t VoxelPerceptionNode::count_neighbors(
    const VoxelKey & key,
    const std::unordered_set<VoxelKey, VoxelKeyHash> & set) const
{
    uint16_t count = 0;

    for (int dx = -1; dx <= 1; dx++)
    for (int dy = -1; dy <= 1; dy++)
    for (int dz = -1; dz <= 1; dz++) {
        if (dx == 0 && dy == 0 && dz == 0) {
            continue;
        }

        VoxelKey neighbor{key.x + dx, key.y + dy, key.z + dz};
        if (set.find(neighbor) != set.end()) {
            count++;
        }
    }

    return count;
}

// ============================================================
// Utility: point <-> voxel
// ============================================================

VoxelPerceptionNode::VoxelKey
VoxelPerceptionNode::make_voxel_key(float x, float y, float z) const
{
    return VoxelKey{
        static_cast<int>(std::floor(x / voxel_resolution_)),
        static_cast<int>(std::floor(y / voxel_resolution_)),
        static_cast<int>(std::floor(z / voxel_resolution_))
    };
}

pcl::PointXYZI VoxelPerceptionNode::voxel_key_to_point(const VoxelKey & key) const
{
    return voxel_key_to_point(key, 1.0f);
}

pcl::PointXYZI VoxelPerceptionNode::voxel_key_to_point(const VoxelKey & key, float score) const
{
    pcl::PointXYZI pt;
    pt.x = (static_cast<float>(key.x) + 0.5f) * voxel_resolution_;
    pt.y = (static_cast<float>(key.y) + 0.5f) * voxel_resolution_;
    pt.z = (static_cast<float>(key.z) + 0.5f) * voxel_resolution_;
    pt.intensity = score;
    return pt;
}

// ============================================================
// Utility: dynamic score
// ============================================================

float VoxelPerceptionNode::compute_dynamic_score(
    float static_confidence,
    uint16_t persistence,
    uint16_t neighbor_count) const
{
    float temporal_stability =
        std::min(1.0f,
            static_cast<float>(persistence) /
            static_cast<float>(persistence_threshold_));

    float neighbor_score =
        std::clamp(static_cast<float>(neighbor_count) / 26.0f, 0.0f, 1.0f);

    float novelty_score =
        std::clamp(1.0f - static_confidence, 0.0f, 1.0f);

    // weighted sum
    return 0.5f * temporal_stability +
           0.3f * neighbor_score +
           0.2f * novelty_score;
}

// ============================================================
// Lifecycle: configure
// ============================================================

VoxelPerceptionNode::CallbackReturn
VoxelPerceptionNode::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "[Lifecycle] Configuring...");

    voxel_resolution_ = this->get_parameter("voxel_resolution").as_double();
    voxel_map_path_ = this->get_parameter("voxel_map_path").as_string();
    max_detection_range_ = this->get_parameter("max_detection_range").as_double();

    input_topic_ = this->get_parameter("input_topic").as_string();
    change_points_topic_ = this->get_parameter("change_points_topic").as_string();
    change_points_info_topic_ = this->get_parameter("change_points_info_topic").as_string();
    live_voxel_topic_ = this->get_parameter("live_voxel_topic").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();

    persistence_threshold_ = this->get_parameter("persistence_threshold").as_int();
    static_confidence_threshold_ =
        static_cast<float>(this->get_parameter("static_confidence_threshold").as_double());

    if (!load_voxel_map(voxel_map_path_)) {
        return CallbackReturn::FAILURE;
    }

    // Publishers
    live_voxel_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        live_voxel_topic_, 10);

    change_detect_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        change_points_topic_, 10);

    change_detect_info_pub_ =
        this->create_publisher<phaseshift_interfaces::msg::VoxelChangeArray>(
            change_points_info_topic_, 10);

    return CallbackReturn::SUCCESS;
}

// ============================================================
// Lifecycle: activate
// ============================================================

VoxelPerceptionNode::CallbackReturn
VoxelPerceptionNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "[Lifecycle] Activating...");

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_,
        rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&VoxelPerceptionNode::points_callback, this, std::placeholders::_1));

    live_voxel_pub_->on_activate();
    change_detect_pub_->on_activate();
    change_detect_info_pub_->on_activate();

    return CallbackReturn::SUCCESS;
}

// ============================================================
// Lifecycle: deactivate
// ============================================================

VoxelPerceptionNode::CallbackReturn
VoxelPerceptionNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "[Lifecycle] Deactivating...");

    if (sub_) {
        sub_.reset();
    }

    live_voxel_pub_->on_deactivate();
    change_detect_pub_->on_deactivate();
    change_detect_info_pub_->on_deactivate();

    return CallbackReturn::SUCCESS;
}

// ============================================================
// Lifecycle: cleanup
// ============================================================

VoxelPerceptionNode::CallbackReturn
VoxelPerceptionNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "[Lifecycle] Cleaning up...");

    sub_.reset();
    live_voxel_pub_.reset();
    change_detect_pub_.reset();
    change_detect_info_pub_.reset();

    voxel_persistence_.clear();

    return CallbackReturn::SUCCESS;
}

// ============================================================
// Main callback
// ============================================================

void VoxelPerceptionNode::points_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    auto t0 = std::chrono::steady_clock::now();

    // -----------------------------
    // latency
    // -----------------------------
    rclcpp::Time now = get_clock()->now();
    double latency_ms = (now - msg->header.stamp).seconds() * 1000.0;

    if (!change_detect_pub_ || !live_voxel_pub_ || !change_detect_info_pub_) return;
    if (!change_detect_pub_->is_activated() ||
        !live_voxel_pub_->is_activated() ||
        !change_detect_info_pub_->is_activated()) return;

    // -----------------------------
    // ROS → PCL
    // -----------------------------
    pcl::PointCloud<pcl::PointXYZI> input_cloud;
    pcl::fromROSMsg(*msg, input_cloud);
    if (input_cloud.empty()) return;

    // -----------------------------
    // range filter
    // -----------------------------
    pcl::PointCloud<pcl::PointXYZI> range_filtered;
    range_filtered.reserve(input_cloud.points.size());

    const float max_range_sq = max_detection_range_ * max_detection_range_;

    for (const auto & pt : input_cloud.points)
    {
        float d2 = pt.x*pt.x + pt.y*pt.y + pt.z*pt.z;
        if (d2 <= max_range_sq)
            range_filtered.points.push_back(pt);
    }

    if (range_filtered.empty()) return;

    auto t_range = std::chrono::steady_clock::now();

    // -----------------------------
    // lidar voxelization
    // -----------------------------
    std::unordered_set<VoxelKey, VoxelKeyHash> live_voxel_set;
    live_voxel_set.reserve(range_filtered.points.size());

    for (const auto & pt : range_filtered.points)
        live_voxel_set.insert(make_voxel_key(pt.x, pt.y, pt.z));

    auto t_lidar_voxel = std::chrono::steady_clock::now();

    // -----------------------------
    // TF
    // -----------------------------
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(
            target_frame_,
            msg->header.frame_id,
            tf2::TimePointZero
        );
    }
    catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "TF lookup failed: %s",
            ex.what()
        );
        return;
    }

    auto t_tf = std::chrono::steady_clock::now();

    // -----------------------------
    // map voxel
    // -----------------------------
    std::unordered_set<VoxelKey, VoxelKeyHash> map_voxel_set;
    map_voxel_set.reserve(live_voxel_set.size());

    geometry_msgs::msg::PointStamped p_in, p_out;

    for (const auto & key : live_voxel_set)
    {
        p_in.point.x = (key.x + 0.5f) * voxel_resolution_;
        p_in.point.y = (key.y + 0.5f) * voxel_resolution_;
        p_in.point.z = (key.z + 0.5f) * voxel_resolution_;

        tf2::doTransform(p_in, p_out, tf);

        map_voxel_set.insert(make_voxel_key(
            p_out.point.x,
            p_out.point.y,
            p_out.point.z
        ));
    }

    // -----------------------------
    // persistence update
    // -----------------------------
    for (const auto & key : map_voxel_set)
        voxel_persistence_[key]++;

    for (auto it = voxel_persistence_.begin(); it != voxel_persistence_.end();)
    {
        if (map_voxel_set.find(it->first) == map_voxel_set.end())
        {
            it->second--;
            if (it->second <= 0)
            {
                it = voxel_persistence_.erase(it);
                continue;
            }
        }
        ++it;
    }

    // -----------------------------
    // RID 
    // -----------------------------
    int min_x = INT_MAX, min_y = INT_MAX, min_z = INT_MAX;
    int max_x = INT_MIN, max_y = INT_MIN, max_z = INT_MIN;

    for (const auto & key : map_voxel_set)
    {
        min_x = std::min(min_x, key.x);
        min_y = std::min(min_y, key.y);
        min_z = std::min(min_z, key.z);

        max_x = std::max(max_x, key.x);
        max_y = std::max(max_y, key.y);
        max_z = std::max(max_z, key.z);
    }

    int size_x = max_x - min_x + 1;
    int size_y = max_y - min_y + 1;
    int size_z = max_z - min_z + 1;

    std::vector<uint8_t> grid(size_x * size_y * size_z, 0);

    auto idx = [&](int x, int y, int z) -> int {
        return (x - min_x) +
               size_x * ((y - min_y) +
               size_y * (z - min_z));
    };

    for (const auto & key : map_voxel_set)
        grid[idx(key.x, key.y, key.z)] = 1;

    // -----------------------------
    // feature vector
    // -----------------------------
    struct VoxelFeature {
        VoxelKey key;
        float static_conf;
        uint16_t persistence;
        uint8_t neighbor;
    };

    std::vector<VoxelFeature> features;
    features.reserve(map_voxel_set.size());

    for (const auto & key : map_voxel_set)
    {
        VoxelFeature f;
        f.key = key;

        auto it_conf = static_confidence_map_.find(key);
        f.static_conf = (it_conf != static_confidence_map_.end()) ? it_conf->second : 0.0f;

        auto it_p = voxel_persistence_.find(key);
        f.persistence = (it_p != voxel_persistence_.end()) ? it_p->second : 0;

        f.neighbor = 0;

        features.push_back(f);
    }

    auto t_map_voxel = std::chrono::steady_clock::now();

    // -----------------------------
    // neighbor (grid)
    // -----------------------------
    for (auto & f : features)
    {
        int x = f.key.x;
        int y = f.key.y;
        int z = f.key.z;

        for (int dx=-1; dx<=1; dx++)
        for (int dy=-1; dy<=1; dy++)
        for (int dz=-1; dz<=1; dz++)
        {
            if (dx==0 && dy==0 && dz==0) continue;

            int nx = x + dx;
            int ny = y + dy;
            int nz = z + dz;

            if (nx < min_x || nx > max_x ||
                ny < min_y || ny > max_y ||
                nz < min_z || nz > max_z)
                continue;

            if (grid[idx(nx, ny, nz)])
                f.neighbor++;
        }
    }

    // -----------------------------
    // filtering
    // -----------------------------
    std::unordered_set<VoxelKey, VoxelKeyHash> filtered_changed;
    filtered_changed.reserve(features.size());

    for (const auto & f : features)
    {
        if (f.static_conf >= static_confidence_threshold_) continue;
        if (f.neighbor == 0) continue;
        if (f.persistence < persistence_threshold_) continue;

        filtered_changed.insert(f.key);
    }

    auto t_filter = std::chrono::steady_clock::now();

    // -----------------------------
    // build change info msg
    // -----------------------------
    phaseshift_interfaces::msg::VoxelChangeArray change_info_msg;
    change_info_msg.header = msg->header;
    change_info_msg.header.frame_id = target_frame_;

    change_info_msg.voxels.reserve(filtered_changed.size());

    // -----------------------------
    // publish cloud
    // -----------------------------
    pcl::PointCloud<pcl::PointXYZI> live_cloud;
    pcl::PointCloud<pcl::PointXYZI> changed_cloud;

    for (const auto & f : features)
        live_cloud.points.push_back(voxel_key_to_point(f.key, 1.0f));

    for (const auto & f : features)
    {
        if (filtered_changed.find(f.key) == filtered_changed.end())
            continue;

        // VoxelChange
        phaseshift_interfaces::msg::VoxelChange v;

        // position (voxel center)
        auto pt = voxel_key_to_point(f.key);
        v.position.x = pt.x;
        v.position.y = pt.y;
        v.position.z = pt.z;

        // raw features
        v.static_confidence = f.static_conf;
        v.persistence = f.persistence;
        v.neighbor_count = f.neighbor;

        float normalized_persistence =
            std::min(1.0f,
                static_cast<float>(f.persistence) /
                static_cast<float>(persistence_threshold_));

        float temporal_stability = normalized_persistence;

        float dynamic_score =
        normalized_persistence * (1.0f - f.static_conf);

        v.dynamic_score = dynamic_score;
        v.temporal_stability = temporal_stability;  
        change_info_msg.voxels.push_back(v);

        // changed cloud 
        float score =
            0.5f * (float)f.persistence / persistence_threshold_ +
            0.3f * (f.neighbor / 26.0f) +
            0.2f * (1.0f - f.static_conf);

        changed_cloud.points.push_back(voxel_key_to_point(f.key, score));
    }

    sensor_msgs::msg::PointCloud2 live_msg, changed_msg;
    pcl::toROSMsg(live_cloud, live_msg);
    pcl::toROSMsg(changed_cloud, changed_msg);

    live_msg.header = msg->header;
    changed_msg.header = msg->header;

    live_msg.header.frame_id = target_frame_;
    changed_msg.header.frame_id = target_frame_;

    live_voxel_pub_->publish(live_msg);
    change_detect_pub_->publish(changed_msg);
    change_detect_info_pub_->publish(change_info_msg);

    auto t_end = std::chrono::steady_clock::now();

    // -----------------------------
    // timing log
    // -----------------------------
    double total =
        std::chrono::duration<double, std::milli>(t_end - t0).count();

    double range_ms =
        std::chrono::duration<double, std::milli>(t_range - t0).count();

    double lidar_voxel_ms =
        std::chrono::duration<double, std::milli>(t_lidar_voxel - t_range).count();

    double tf_ms =
        std::chrono::duration<double, std::milli>(t_tf - t_lidar_voxel).count();

    double map_voxel_ms =
        std::chrono::duration<double, std::milli>(t_map_voxel - t_tf).count();

    double filter_ms =
        std::chrono::duration<double, std::milli>(t_filter - t_map_voxel).count();

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Total=%.2f ms | Latency=%.2f | Range=%.2f | LidarVoxel=%.2f | TF=%.2f | MapVoxel=%.2f | Filter=%.2f",
        total,
        latency_ms,
        range_ms,
        lidar_voxel_ms,
        tf_ms,
        map_voxel_ms,
        filter_ms
    );
}