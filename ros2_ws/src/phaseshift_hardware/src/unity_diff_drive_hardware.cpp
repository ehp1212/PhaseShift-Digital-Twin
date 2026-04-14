#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "pluginlib/class_list_macros.hpp"

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class UnityDiffDriveHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
      return CallbackReturn::ERROR;

    size_t n = info_.joints.size();

    hw_positions_.resize(n, 0.0);
    hw_velocities_.resize(n, 0.0);
    hw_commands_.resize(n, 0.0);

    node_ = rclcpp::Node::make_shared("unity_diff_drive_hardware");

    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    last_time_ = node_->now();

    return CallbackReturn::SUCCESS;
  }

  // ------------------------
  // Interfaces
  // ------------------------
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> states;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      states.emplace_back(info_.joints[i].name,
                          hardware_interface::HW_IF_POSITION,
                          &hw_positions_[i]);

      states.emplace_back(info_.joints[i].name,
                          hardware_interface::HW_IF_VELOCITY,
                          &hw_velocities_[i]);
    }

    return states;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> cmds;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      cmds.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_VELOCITY,
                        &hw_commands_[i]);
    }

    return cmds;
  }

  // ------------------------
  // READ (state update)
  // ------------------------
  hardware_interface::return_type read(const rclcpp::Time & time,
                                       const rclcpp::Duration & period) override
  {
    double dt = period.seconds();

    // 1. wheel velocity update
    for (size_t i = 0; i < hw_velocities_.size(); ++i)
    {
      hw_velocities_[i] = hw_commands_[i];
      hw_positions_[i] += hw_velocities_[i] * dt;
    }

    // 2. left/right
    double left = (hw_velocities_[0] + hw_velocities_[1]) * 0.5;
    double right = (hw_velocities_[2] + hw_velocities_[3]) * 0.5;

    // 3. diff kinematics
    double v = (left + right) * 0.5;
    double w = (right - left) / wheel_separation_;

    // 4. integrate pose
    theta_ += w * dt;
    x_ += v * cos(theta_) * dt;
    y_ += v * sin(theta_) * dt;
    
    if ((time - last_time_).seconds() > 0.02)
    {
      publish_odom(time, v, w);
      last_time_ = time;
    }

    return hardware_interface::return_type::OK;
  }

  // ------------------------
  // WRITE (command)
  // ------------------------
  hardware_interface::return_type write(const rclcpp::Time &,
                                        const rclcpp::Duration &) override
  {
    // diff_drive_controller → hw_commands_ 
    return hardware_interface::return_type::OK;
  }

private:

  void publish_odom(const rclcpp::Time & now, double v, double w)
  {
    nav_msgs::msg::Odometry msg;

    msg.header.stamp = now;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_footprint";

    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);

    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    msg.twist.twist.linear.x = v;
    msg.twist.twist.angular.z = w;

    odom_pub_->publish(msg);

    // TF broadcast
    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = now;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";

    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;

    tf.transform.rotation = msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf);
  }

private:

  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Joint data
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  // Robot state
  double x_ = -0.5;
  double y_ = 0.0;
  double theta_ = 3.141592;

  double wheel_separation_ = 0.55; // Husky

  rclcpp::Time last_time_;
};

PLUGINLIB_EXPORT_CLASS(
  UnityDiffDriveHardware,
  hardware_interface::SystemInterface)