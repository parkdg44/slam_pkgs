//
// Created by park on 24. 4. 2.
//

#pragma once

#include <memory>

#include "slam_pkg/ros2/ros2_converter.hpp"
#include "slam_pkg/ros2/ros2_utils.hpp"
#include "slam_pkg/util/string_helper/string_helper.hpp"

namespace Slam::ros2 {

class SlamNode : public rclcpp::Node {
  template <typename T>
  using SubPtr = typename rclcpp::Subscription<T>::SharedPtr;

 public:
  SlamNode() : rclcpp::Node("slam_main") {
    using Slam::util::split_str;

    auto laser_topics =
        split_str(util::declare_and_get_parameter<std::string>(this, "laser_topics"), ' ');
    auto pcd_topics =
        split_str(util::declare_and_get_parameter<std::string>(this, "pcd_topics"), ' ');

    for (const auto& topic : laser_topics) {
      RCLCPP_INFO(get_logger(), "subscribe [%s] LaserScan topic", topic.c_str());
      laser_map_[topic] = create_subscription<sensor_msgs::msg::LaserScan>(
          topic, rclcpp::SensorDataQoS(),
          [this, topic](const sensor_msgs::msg::LaserScan& data) { laser_callback(topic, data); });
    }

    for (const auto& topic : pcd_topics) {
      RCLCPP_INFO(get_logger(), "subscribe [%s] PointCloud topic", topic.c_str());
      pcd_map_[topic] = create_subscription<sensor_msgs::msg::PointCloud2>(
          topic, rclcpp::SensorDataQoS(),
          [this, topic](const sensor_msgs::msg::PointCloud2& data) { pcd_callback(topic, data); });
    }
  }

  void laser_callback(const std::string& topic, const sensor_msgs::msg::LaserScan& data) {
    LaserScan scan = from_ros(data);
  }

  void pcd_callback(const std::string& topic, const sensor_msgs::msg::PointCloud2& data) {
    PointCloud pcd = from_ros(data);
  }

 private:
  std::unordered_map<std::string, SubPtr<sensor_msgs::msg::LaserScan>> laser_map_{};
  std::unordered_map<std::string, SubPtr<sensor_msgs::msg::PointCloud2>> pcd_map_{};
  LaserScan scan_{};
};

}  // namespace Slam::ros2
