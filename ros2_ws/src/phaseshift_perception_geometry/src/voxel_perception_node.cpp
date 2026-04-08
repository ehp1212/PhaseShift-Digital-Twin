#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "phaseshift_interfaces/msg/voxel_change.hpp"
#include "phaseshift_interfaces/msg/voxel_change_array.hpp"

#include <unordered_set>
#include <unordered_map>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class VoxelPerceptionNode : public rclcpp_lifecycle::LifecycleNode
{

private: 
    double voxel_resolution_;
    std::string voxel_map_path_;

    std::string input_topic_;
    std::string change_points_topic_;
    std::string live_voxel_topic_;
    std::string change_points_info_topic_;

    std::string target_frame_;

    int persistence_threshold_;

    struct VoxelKey{
        int x, y, z;

        bool operator==(const VoxelKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelKeyHash{
        std::size_t operator()(const VoxelKey& k) const{
            return ((std::hash<int>()(k.x) ^
                (std::hash<int>()(k.y) << 1)) >> 1) ^
                (std::hash<int>()(k.z) << 1);
        }
    };

    bool load_voxel_map(const std::string& path)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud;

        if (pcl::io::loadPCDFile<pcl::PointXYZI>(path, cloud) == -1)
        {
            RCLCPP_ERROR(get_logger(), "Failed to load PCD");
            return false;
        }

        for (const auto& pt : cloud.points)
            static_voxel_set_.insert(make_voxel_key(pt.x, pt.y, pt.z));

        RCLCPP_INFO(get_logger(), "Static voxel set size: %zu", static_voxel_set_.size());
        return true;
    }

public:
    VoxelPerceptionNode()
    : rclcpp_lifecycle::LifecycleNode("voxel_perception_node")
    {
        RCLCPP_INFO(get_logger(), "[VoxelPerceptionNode] Created");

        this->declare_parameter("voxel_resolution", 0.15);
        this->declare_parameter("voxel_map_path", "");

        this->declare_parameter<std::string>("input_topic", "/velodyne_points");
        this->declare_parameter<std::string>("change_points_topic", "/perception_geometry/change_points");
        this->declare_parameter<std::string>("change_points_info_topic", "/perception_geometry/change_points_info");
        this->declare_parameter<std::string>("live_voxel_topic", "/perception_geometry/live_voxel_points");
        this->declare_parameter<std::string>("target_frame", "map");

        this->declare_parameter("persistence_threshold", 3);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    // -----------------------------
    // CONFIGURE
    // -----------------------------
    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "[Lifecycle] Configuring...");

        voxel_resolution_ = this->get_parameter("voxel_resolution").as_double();
        voxel_map_path_ = this->get_parameter("voxel_map_path").as_string();

        RCLCPP_INFO(get_logger(), "Resolution: %.2f", voxel_resolution_);
        RCLCPP_INFO(get_logger(), "PCD Path: %s", voxel_map_path_.c_str());

        input_topic_ = this->get_parameter("input_topic").as_string();
        change_points_topic_ = this->get_parameter("change_points_topic").as_string();
        change_points_info_topic_ = this->get_parameter("change_points_info_topic").as_string();

        live_voxel_topic_ = this->get_parameter("live_voxel_topic").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();
        persistence_threshold_ = this->get_parameter("persistence_threshold").as_int();
        
        if (!load_voxel_map(voxel_map_path_))
        {
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(get_logger(), "Loading VoxelMap : %s", voxel_map_path_.c_str());

        live_voxel_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            live_voxel_topic_,
            10
        );

        change_detect_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            change_points_topic_,
            10
        );

        change_detect_info_pub_ = create_publisher<phaseshift_interfaces::msg::VoxelChangeArray>(
            change_points_info_topic_,
            10
        );

        return CallbackReturn::SUCCESS;
    }

    // -----------------------------
    // ACTIVATE
    // -----------------------------
    CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "[Lifecycle] Activating...");

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_,
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&VoxelPerceptionNode::points_callback, this, std::placeholders::_1)
        );

        live_voxel_pub_->on_activate();
        change_detect_pub_->on_activate();
        change_detect_info_pub_->on_activate();

        return CallbackReturn::SUCCESS;
    }

    // -----------------------------
    // DEACTIVATE
    // -----------------------------
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "[Lifecycle] Deactivating...");

        live_voxel_pub_->on_deactivate();
        change_detect_pub_->on_deactivate();
        change_detect_info_pub_->on_deactivate();

        return CallbackReturn::SUCCESS;
    }

    // -----------------------------
    // CLEANUP
    // -----------------------------
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "[Lifecycle] Cleaning up...");

        sub_.reset();
        live_voxel_pub_.reset();
        change_detect_pub_.reset();
        change_detect_info_pub_.reset();

        return CallbackReturn::SUCCESS;
    }

private:
std::unordered_set<VoxelKey, VoxelKeyHash> static_voxel_set_;
std::unordered_map<VoxelKey, int, VoxelKeyHash> voxel_persistence_;

rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr live_voxel_pub_;
rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr change_detect_pub_;

rclcpp_lifecycle::LifecyclePublisher<phaseshift_interfaces::msg::VoxelChangeArray>::SharedPtr change_detect_info_pub_;

std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

bool has_neighbour(const VoxelKey& key, const std::unordered_set<VoxelKey, VoxelKeyHash>& set)
{
    for (int dx = -1; dx <= 1; dx++)
    for (int dy = -1; dy <= 1; dy++)
    for (int dz = -1; dz <= 1; dz++)
    {
        if (dx==0 && dy==0 && dz==0) continue;

        if (set.find({key.x+dx, key.y+dy, key.z+dz}) != set.end())
            return true;
    }

    return false;
}

// -----------------------------
// CALLBACK
// -----------------------------
void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // -----------------------------
    // latency
    // -----------------------------
    rclcpp::Time now = get_clock()->now();
    double latency_ms = (now - msg->header.stamp).seconds() * 1000.0;

    auto t_start = std::chrono::steady_clock::now();

    if (!change_detect_pub_ || !live_voxel_pub_ || !change_detect_info_pub_) return;
    if (!change_detect_pub_->is_activated() || !live_voxel_pub_->is_activated()
        || !change_detect_info_pub_->is_activated()) return;

    // -----------------------------
    // ROS → PCL (lidar frame)
    // -----------------------------
    pcl::PointCloud<pcl::PointXYZI> input_cloud;
    pcl::fromROSMsg(*msg, input_cloud);

    if (input_cloud.empty()) return;

    // -----------------------------
    // voxelization (lidar frame)
    // -----------------------------
    std::unordered_set<VoxelKey, VoxelKeyHash> live_voxel_set;
    live_voxel_set.reserve(input_cloud.points.size());

    for (const auto & pt : input_cloud.points)
    {
        VoxelKey key = {
            static_cast<int>(std::floor(pt.x / voxel_resolution_)),
            static_cast<int>(std::floor(pt.y / voxel_resolution_)),
            static_cast<int>(std::floor(pt.z / voxel_resolution_))
        };
        live_voxel_set.insert(key);
    }

    // -----------------------------
    // voxel → pointcloud
    // -----------------------------
    pcl::PointCloud<pcl::PointXYZI> voxel_cloud;

    for (const auto & key : live_voxel_set) {
        voxel_cloud.points.push_back(voxel_key_to_point(key));
    }

    voxel_cloud.width = voxel_cloud.points.size();
    voxel_cloud.height = 1;
    voxel_cloud.is_dense = true;

    // -----------------------------
    // TF (voxel only)
    // -----------------------------
    auto tf_start = std::chrono::steady_clock::now();

    sensor_msgs::msg::PointCloud2 voxel_msg;
    pcl::toROSMsg(voxel_cloud, voxel_msg);
    voxel_msg.header = msg->header;

    geometry_msgs::msg::TransformStamped tf;

    try {
        tf = tf_buffer_->lookupTransform(
            target_frame_,
            msg->header.frame_id,
            tf2::TimePointZero
        );
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
        return;
    }

    sensor_msgs::msg::PointCloud2 transformed_voxel_msg;
    tf2::doTransform(voxel_msg, transformed_voxel_msg, tf);

    auto tf_end = std::chrono::steady_clock::now();
    double tf_ms = std::chrono::duration<double, std::milli>(tf_end - tf_start).count();

    // -----------------------------
    // voxelization (map frame)
    // -----------------------------
    pcl::PointCloud<pcl::PointXYZI> map_cloud;
    pcl::fromROSMsg(transformed_voxel_msg, map_cloud);

    std::unordered_set<VoxelKey, VoxelKeyHash> map_voxel_set;
    std::unordered_set<VoxelKey, VoxelKeyHash> changed_voxel_set;

    map_voxel_set.reserve(map_cloud.points.size());
    changed_voxel_set.reserve(map_cloud.points.size());

    for (const auto & pt : map_cloud.points)
    {
        VoxelKey key = {
            static_cast<int>(std::floor(pt.x / voxel_resolution_)),
            static_cast<int>(std::floor(pt.y / voxel_resolution_)),
            static_cast<int>(std::floor(pt.z / voxel_resolution_))
        };

        map_voxel_set.insert(key);

        if (static_voxel_set_.find(key) == static_voxel_set_.end()) {
            changed_voxel_set.insert(key);
        }
    }

    // -----------------------------
    // temporal filtering
    // -----------------------------
    for (const auto& key : map_voxel_set)
    {
        voxel_persistence_[key]++;
    }

    for (auto it = voxel_persistence_.begin(); it != voxel_persistence_.end(); )
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
    // neighbor + temporal filtering
    // -----------------------------
    std::unordered_set<VoxelKey, VoxelKeyHash> filtered_changed_voxel_set;

    for (const auto& key : changed_voxel_set)
    {
        // neighbor filter
        if (!has_neighbour(key, map_voxel_set))
            continue;

        // temporal filter
        auto it = voxel_persistence_.find(key);

        if (it != voxel_persistence_.end() && it->second >= persistence_threshold_)
        {
            filtered_changed_voxel_set.insert(key);
        }
    }

    // -----------------------------
    // change detect info
    // -----------------------------
    phaseshift_interfaces::msg::VoxelChangeArray info_msg;
    info_msg.header.frame_id = target_frame_;
    info_msg.header.stamp = msg->header.stamp;

    for (const auto & key : filtered_changed_voxel_set)
    {
        phaseshift_interfaces::msg::VoxelChange voxel;

        auto pt = voxel_key_to_point(key);
        voxel.position.x = pt.x;
        voxel.position.y = pt.y;
        voxel.position.z = pt.z;

        // static confidence
        // 나중에 confidence map 연결하면 여기 교체.
        voxel.static_confidence =
            (static_voxel_set_.find(key) != static_voxel_set_.end()) ? 1.0f : 0.0f;

        // persistence
        auto it = voxel_persistence_.find(key);
        uint16_t persistence = (it != voxel_persistence_.end()) ? static_cast<uint16_t>(it->second) : 0;
        voxel.persistence = persistence;

        // neighbor count
        uint16_t neighbor_count = 0;
        for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++)
        for (int dz = -1; dz <= 1; dz++)
        {
            if (dx == 0 && dy == 0 && dz == 0) continue;

            VoxelKey neighbor{key.x + dx, key.y + dy, key.z + dz};
            if (map_voxel_set.find(neighbor) != map_voxel_set.end()) {
                neighbor_count++;
            }
        }
        voxel.neighbor_count = neighbor_count;

        // temporal_stability: persistence normalized
        float temporal_stability =
            std::min(1.0f, static_cast<float>(persistence) / static_cast<float>(persistence_threshold_));

        voxel.temporal_stability = temporal_stability;

        // dynamic_score: temporal_stability
        voxel.dynamic_score = temporal_stability;

        info_msg.voxels.push_back(voxel);
    }

    // -----------------------------
    // publish
    // -----------------------------
    pcl::PointCloud<pcl::PointXYZI> live_voxel_cloud;
    pcl::PointCloud<pcl::PointXYZI> changed_voxel_cloud;

    for (const auto & key : map_voxel_set) {
        live_voxel_cloud.points.push_back(voxel_key_to_point(key));
    }

    for (const auto & key : filtered_changed_voxel_set) {
        changed_voxel_cloud.points.push_back(voxel_key_to_point(key));
    }

    live_voxel_cloud.width = live_voxel_cloud.points.size();
    live_voxel_cloud.height = 1;
    live_voxel_cloud.is_dense = true;

    changed_voxel_cloud.width = changed_voxel_cloud.points.size();
    changed_voxel_cloud.height = 1;
    changed_voxel_cloud.is_dense = true;

    sensor_msgs::msg::PointCloud2 live_msg;
    sensor_msgs::msg::PointCloud2 changed_msg;

    pcl::toROSMsg(live_voxel_cloud, live_msg);
    pcl::toROSMsg(changed_voxel_cloud, changed_msg);

    live_msg.header.frame_id = target_frame_;
    changed_msg.header.frame_id = target_frame_;

    live_voxel_pub_->publish(live_msg);
    change_detect_pub_->publish(changed_msg);
    change_detect_info_pub_->publish(info_msg);

    // -----------------------------
    // latency
    // -----------------------------
    auto t_end = std::chrono::steady_clock::now();
    double proc_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Latency=%.2f ms | TF=%.2f ms | Proc=%.2f ms | Points=%zu → Voxels=%zu | Changed=%zu → Filtered=%zu",
        latency_ms,
        tf_ms,
        proc_ms,
        input_cloud.points.size(),
        map_voxel_set.size(),
        changed_voxel_set.size(),
        filtered_changed_voxel_set.size()
    );
}

VoxelKey make_voxel_key(float x, float y, float z) const
{
    return VoxelKey{
        static_cast<int>(std::floor(x / voxel_resolution_)),
        static_cast<int>(std::floor(y / voxel_resolution_)),
        static_cast<int>(std::floor(z / voxel_resolution_))
    };
}

pcl::PointXYZI voxel_key_to_point(const VoxelKey & key) const
{
    pcl::PointXYZI pt;
    pt.x = (static_cast<float>(key.x) + 0.5f) * voxel_resolution_;
    pt.y = (static_cast<float>(key.y) + 0.5f) * voxel_resolution_;
    pt.z = (static_cast<float>(key.z) + 0.5f) * voxel_resolution_;
    pt.intensity = 1.0f;
    return pt;
}

};

// -----------------------------
// MAIN
// -----------------------------
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VoxelPerceptionNode>();

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}