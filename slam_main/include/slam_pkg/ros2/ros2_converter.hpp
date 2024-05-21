//
// Created by park on 24. 4. 3.
//

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include "slam_pkg/type/types.hpp"

namespace Slam::ros2 {

#pragma region to_ros

inline builtin_interfaces::msg::Time to_ros(Time const& data) { return rclcpp::Time(data.value); }

inline std_msgs::msg::Header to_ros(Header const& data) {
  std_msgs::msg::Header output;
  output.stamp = to_ros(data.stamp);
  output.frame_id = data.ref_frame_id;
  return output;
}

inline geometry_msgs::msg::Point to_ros(Point const& data) {
  geometry_msgs::msg::Point output;
  output.x = data.x;
  output.y = data.y;
  output.z = data.z;
  return output;
}

inline geometry_msgs::msg::Quaternion to_ros(Quaternion const& data) {
  geometry_msgs::msg::Quaternion output;
  output.w = data.w;
  output.x = data.x;
  output.y = data.y;
  output.z = data.z;
  return output;
}

inline geometry_msgs::msg::PoseStamped to_ros(PoseStamped const& data) {
  geometry_msgs::msg::PoseStamped output;
  output.header = to_ros(data.header);
  output.pose.position = to_ros(data.pose.p);
  output.pose.orientation = to_ros(data.pose.q);
  return output;
}

inline geometry_msgs::msg::Vector3 to_ros(Vector const& data) {
  geometry_msgs::msg::Vector3 output;
  output.x = data.x;
  output.y = data.y;
  output.z = data.z;
  return output;
}

inline geometry_msgs::msg::Twist to_ros(Twist const& data) {
  geometry_msgs::msg::Twist output;
  output.angular = to_ros(data.angular);
  output.linear = to_ros(data.linear);
  return output;
}

#pragma endregion to_ros

#pragma region from_ros

inline Header from_ros(std_msgs::msg::Header const& data) {
  Header output;
  output.ref_frame_id = data.frame_id;
  output.stamp = rclcpp::Time(data.stamp).nanoseconds();
  return output;
}

inline LaserScan from_ros(sensor_msgs::msg::LaserScan const& data) {
  LaserScan output;
  output.header = from_ros(data.header);
  output.ranges = data.ranges;
  output.intensities = data.intensities;
  ScanInfo& info = output.info;
  info.angle_increment = data.angle_increment;
  info.angle_max = data.angle_max;
  info.angle_min = data.angle_min;
  info.range_max = data.range_max;
  info.range_min = data.range_min;
  info.scan_time = data.scan_time;
  info.time_increment = data.time_increment;
  return output;
}

inline Point from_ros(geometry_msgs::msg::Point32 const& data) {
  Point output;
  output.x = data.x;
  output.y = data.y;
  output.z = data.z;
  return output;
}

inline PointCloud from_ros(sensor_msgs::msg::PointCloud const& data) {
  PointCloud output;
  output.header = from_ros(data.header);
  output.points.reserve(data.points.size());
  for (const auto& p : data.points) {
    output.points.push_back(from_ros(p));
  }
  return output;
}

#pragma endregion from_ros

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
