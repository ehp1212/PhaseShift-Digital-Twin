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
    // [1] end-to-end latency
    // -----------------------------
    rclcpp::Time now = get_clock()->now();
    double latency_ms = (now - msg->header.stamp).seconds() * 1000.0;

    if (!change_detect_pub_ || !live_voxel_pub_ || !change_detect_info_pub_) {
        return;
    }

    if (!change_detect_pub_->is_activated() ||
        !live_voxel_pub_->is_activated() ||
        !change_detect_info_pub_->is_activated()) {
        return;
    }

    // -----------------------------
    // [2] ROS -> PCL
    // -----------------------------
    pcl::PointCloud<pcl::PointXYZI> input_cloud;
    pcl::fromROSMsg(*msg, input_cloud);

    if (input_cloud.empty()) {
        return;
    }

    // -----------------------------
    // [3] distance filter (lidar frame)
    // -----------------------------
    pcl::PointCloud<pcl::PointXYZI> range_filtered_cloud;
    range_filtered_cloud.reserve(input_cloud.points.size());

    const double max_range_sq = max_detection_range_ * max_detection_range_;

    for (const auto & pt : input_cloud.points) {
        const double dist_sq =
            static_cast<double>(pt.x) * pt.x +
            static_cast<double>(pt.y) * pt.y +
            static_cast<double>(pt.z) * pt.z;

        if (dist_sq <= max_range_sq) {
            range_filtered_cloud.points.push_back(pt);
        }
    }

    if (range_filtered_cloud.empty()) {
        return;
    }

    auto t_ros_to_pcl_end = std::chrono::steady_clock::now();

    // -----------------------------
    // [4] lidar-frame voxelization
    // -----------------------------
    std::unordered_set<VoxelKey, VoxelKeyHash> live_voxel_set;
    live_voxel_set.reserve(range_filtered_cloud.points.size());

    for (const auto & pt : range_filtered_cloud.points) {
        live_voxel_set.insert(make_voxel_key(pt.x, pt.y, pt.z));
    }

    auto t_lidar_voxel_end = std::chrono::steady_clock::now();

    // -----------------------------
    // [5] TF lookup
    // -----------------------------
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(
            target_frame_,
            msg->header.frame_id,
            tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "TF lookup failed: %s", ex.what());
        return;
    }

    auto t_tf_lookup_end = std::chrono::steady_clock::now();

    // -----------------------------
    // voxel-center transform only
    //     lidar voxel -> map voxel
    // -----------------------------
    std::unordered_set<VoxelKey, VoxelKeyHash> map_voxel_set;
    std::unordered_set<VoxelKey, VoxelKeyHash> changed_voxel_set;
    map_voxel_set.reserve(live_voxel_set.size());
    changed_voxel_set.reserve(live_voxel_set.size());

    geometry_msgs::msg::PointStamped p_in;
    geometry_msgs::msg::PointStamped p_out;

    for (const auto & key : live_voxel_set) {
        // voxel center in lidar frame
        p_in.point.x = (static_cast<float>(key.x) + 0.5f) * voxel_resolution_;
        p_in.point.y = (static_cast<float>(key.y) + 0.5f) * voxel_resolution_;
        p_in.point.z = (static_cast<float>(key.z) + 0.5f) * voxel_resolution_;

        // transform point
        tf2::doTransform(p_in, p_out, tf);

        // remap to map-frame voxel
        VoxelKey map_key = make_voxel_key(
            static_cast<float>(p_out.point.x),
            static_cast<float>(p_out.point.y),
            static_cast<float>(p_out.point.z));

        map_voxel_set.insert(map_key);
    }

    // static confidence cache
    std::unordered_map<VoxelKey, float, VoxelKeyHash> static_conf_cache;
    static_conf_cache.reserve(map_voxel_set.size());

    for (const auto & key : map_voxel_set)
    {
        auto it = static_confidence_map_.find(key);

        float static_conf = (it != static_confidence_map_.end())
            ? it->second
            : 0.0f;

        static_conf_cache[key] = static_conf;

        if (static_conf < static_confidence_threshold_) {
            changed_voxel_set.insert(key);
        }
    }

    auto t_map_voxel_end = std::chrono::steady_clock::now();

    // -----------------------------
    // temporal persistence update
    // -----------------------------
    for (const auto & key : map_voxel_set) {
        voxel_persistence_[key]++;
    }

    for (auto it = voxel_persistence_.begin(); it != voxel_persistence_.end();) {
        if (map_voxel_set.find(it->first) == map_voxel_set.end()) {
            it->second--;
            if (it->second <= 0) {
                it = voxel_persistence_.erase(it);
                continue;
            }
        }
        ++it;
    }

    // neighbor count cache
    std::unordered_map<VoxelKey, uint16_t, VoxelKeyHash> neighbor_count_cache;
    neighbor_count_cache.reserve(map_voxel_set.size());

    for (const auto & key : map_voxel_set) {
        neighbor_count_cache[key] = count_neighbors(key, map_voxel_set);
    }

    // filtered changed voxels
    std::unordered_set<VoxelKey, VoxelKeyHash> filtered_changed_voxel_set;
    filtered_changed_voxel_set.reserve(changed_voxel_set.size());

    for (const auto & key : changed_voxel_set) {
        auto it = voxel_persistence_.find(key);
        if (it == voxel_persistence_.end()) {
            continue;
        }

        if (neighbor_count_cache[key] == 0) {
            continue;
        }

        if (it->second >= persistence_threshold_) {
            filtered_changed_voxel_set.insert(key);
        }
    }

    auto t_filter_end = std::chrono::steady_clock::now();

    // -----------------------------
    // [8] feature cache for changed voxels
    // -----------------------------
    std::unordered_map<VoxelKey, ChangedFeature, VoxelKeyHash> changed_feature_map;
    changed_feature_map.reserve(filtered_changed_voxel_set.size());

    for (const auto & key : filtered_changed_voxel_set) {
        float static_conf = static_conf_cache[key];

        auto it = voxel_persistence_.find(key);
        uint16_t persistence =
            (it != voxel_persistence_.end()) ? static_cast<uint16_t>(it->second) : 0;

        uint16_t neighbor_count = neighbor_count_cache[key];

        float temporal_stability =
            std::min(1.0f, static_cast<float>(persistence) /
                              static_cast<float>(persistence_threshold_));

        float dynamic_score =
            compute_dynamic_score(static_conf, persistence, neighbor_count);

        changed_feature_map[key] = ChangedFeature{
            static_conf,
            persistence,
            neighbor_count,
            temporal_stability,
            dynamic_score
        };
    }

    // -----------------------------
    // custom feature msg
    // -----------------------------
    phaseshift_interfaces::msg::VoxelChangeArray info_msg;
    info_msg.header.frame_id = target_frame_;
    info_msg.header.stamp = msg->header.stamp;
    info_msg.voxels.reserve(filtered_changed_voxel_set.size());

    for (const auto & key : filtered_changed_voxel_set) {
        const auto & feature = changed_feature_map[key];
        auto pt = voxel_key_to_point(key);

        phaseshift_interfaces::msg::VoxelChange voxel;
        voxel.position.x = pt.x;
        voxel.position.y = pt.y;
        voxel.position.z = pt.z;
        voxel.static_confidence = feature.static_confidence;
        voxel.persistence = feature.persistence;
        voxel.neighbor_count = feature.neighbor_count;
        voxel.temporal_stability = feature.temporal_stability;
        voxel.dynamic_score = feature.dynamic_score;

        info_msg.voxels.push_back(voxel);
    }

    auto t_info_msg_end = std::chrono::steady_clock::now();

    // -----------------------------
    // build point clouds
    //      live: simple intensity=1.0
    //      changed: intensity=dynamic_score
    // -----------------------------
    pcl::PointCloud<pcl::PointXYZI> live_voxel_cloud;
    pcl::PointCloud<pcl::PointXYZI> changed_voxel_cloud;
    live_voxel_cloud.reserve(map_voxel_set.size());
    changed_voxel_cloud.reserve(filtered_changed_voxel_set.size());

    for (const auto & key : map_voxel_set) {
        live_voxel_cloud.points.push_back(voxel_key_to_point(key, 1.0f));
    }

    for (const auto & key : filtered_changed_voxel_set) {
        const auto & feature = changed_feature_map[key];
        changed_voxel_cloud.points.push_back(
            voxel_key_to_point(key, feature.dynamic_score));
    }

    live_voxel_cloud.width = static_cast<uint32_t>(live_voxel_cloud.points.size());
    live_voxel_cloud.height = 1;
    live_voxel_cloud.is_dense = true;

    changed_voxel_cloud.width = static_cast<uint32_t>(changed_voxel_cloud.points.size());
    changed_voxel_cloud.height = 1;
    changed_voxel_cloud.is_dense = true;

    sensor_msgs::msg::PointCloud2 live_msg;
    sensor_msgs::msg::PointCloud2 changed_msg;

    pcl::toROSMsg(live_voxel_cloud, live_msg);
    pcl::toROSMsg(changed_voxel_cloud, changed_msg);

    live_msg.header.frame_id = target_frame_;
    changed_msg.header.frame_id = target_frame_;
    live_msg.header.stamp = msg->header.stamp;
    changed_msg.header.stamp = msg->header.stamp;

    auto t_cloud_build_end = std::chrono::steady_clock::now();

    // -----------------------------
    // [11] publish
    // -----------------------------
    live_voxel_pub_->publish(live_msg);
    change_detect_pub_->publish(changed_msg);
    change_detect_info_pub_->publish(info_msg);

    auto t_publish_end = std::chrono::steady_clock::now();

    // -----------------------------
    // [12] timing logs
    // -----------------------------
    double ros_to_pcl_ms =
        std::chrono::duration<double, std::milli>(t_ros_to_pcl_end - t0).count();
    double lidar_voxel_ms =
        std::chrono::duration<double, std::milli>(t_lidar_voxel_end - t_ros_to_pcl_end).count();
    double tf_ms =
        std::chrono::duration<double, std::milli>(t_tf_lookup_end - t_lidar_voxel_end).count();
    double map_voxel_ms =
        std::chrono::duration<double, std::milli>(t_map_voxel_end - t_tf_lookup_end).count();
    double filter_ms =
        std::chrono::duration<double, std::milli>(t_filter_end - t_map_voxel_end).count();
    double info_msg_ms =
        std::chrono::duration<double, std::milli>(t_info_msg_end - t_filter_end).count();
    double cloud_build_ms =
        std::chrono::duration<double, std::milli>(t_cloud_build_end - t_info_msg_end).count();
    double publish_ms =
        std::chrono::duration<double, std::milli>(t_publish_end - t_cloud_build_end).count();
    double total_ms =
        std::chrono::duration<double, std::milli>(t_publish_end - t0).count();

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Timing | Total=%.2f | ROS+Range=%.2f | LidarVoxel=%.2f | TF=%.2f | MapVoxel=%.2f | Filter=%.2f | InfoMsg=%.2f | CloudBuild=%.2f | Publish=%.2f | Latency=%.2f",
        total_ms,
        ros_to_pcl_ms,
        lidar_voxel_ms,
        tf_ms,
        map_voxel_ms,
        filter_ms,
        info_msg_ms,
        cloud_build_ms,
        publish_ms,
        latency_ms
    );
}