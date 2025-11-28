//
// Created by park on 24. 4. 3.
//

#pragma once

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "slam_pkg/type/types.hpp"

namespace Slam::ros2 {

inline builtin_interfaces::msg::Time to_ros(Time const& data) { return rclcpp::Time(data.value); }

inline std_msgs::msg::Header to_ros(Header const& data) {
  std_msgs::msg::Header output;
  output.stamp = to_ros(data.stamp);
  output.frame_id = data.ref_frame_id;
  return output;
}

template <typename ROS_T = geometry_msgs::msg::Point>
inline ROS_T to_ros(Vector const& data);

template <>
inline geometry_msgs::msg::Point to_ros(Vector const& data) {
  geometry_msgs::msg::Point output;
  output.x = data.x();
  output.y = data.y();
  output.z = data.z();
  return output;
}

template <>
inline geometry_msgs::msg::Vector3 to_ros(Vector const& data) {
  geometry_msgs::msg::Vector3 output;
  output.x = data.x();
  output.y = data.y();
  output.z = data.z();
  return output;
}

inline geometry_msgs::msg::Quaternion to_ros(Quaternion const& data) {
  geometry_msgs::msg::Quaternion output;
  output.w = data.w();
  output.x = data.x();
  output.y = data.y();
  output.z = data.z();
  return output;
}

inline geometry_msgs::msg::Pose to_ros(Pose const& data) {
  geometry_msgs::msg::Pose output;
  output.position = to_ros(data.translation());
  output.orientation = to_ros(data.rotation());
  return output;
}

inline geometry_msgs::msg::PoseStamped to_ros(PoseStamped const& data) {
  geometry_msgs::msg::PoseStamped output;
  output.header = to_ros(data.header);
  output.pose = to_ros(data.pose);
  return output;
}

inline geometry_msgs::msg::Twist to_ros(Twist const& data) {
  geometry_msgs::msg::Twist output;
  output.angular = to_ros<geometry_msgs::msg::Vector3>(data.angular);
  output.linear = to_ros<geometry_msgs::msg::Vector3>(data.linear);
  return output;
}

template <typename SlamT>
std::function<void(std::string const&, SlamT const&)> pub_function_builder(rclcpp::Node& node) {
  return [&node](std::string const& topic, SlamT const& data) {
    using RosT = decltype(to_ros(std::declval<SlamT>()));
    static auto map_pub = std::unordered_map<std::string, std::shared_ptr<rclcpp::PublisherBase>>();

    if (map_pub.find(topic) == map_pub.end()) {
      map_pub[topic] = node.create_publisher<RosT>(topic);
    }
    auto pub_data = std::static_pointer_cast<rclcpp::Publisher<RosT>>(map_pub[topic]);
    pub_data->publish(to_ros(data));
  };
}

}  // namespace Slam::ros2
