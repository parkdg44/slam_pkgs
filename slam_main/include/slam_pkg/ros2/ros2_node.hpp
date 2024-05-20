//
// Created by park on 24. 4. 2.
//

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "slam_pkg/ros2/ros2_converter.hpp"
#include "slam_pkg/util/string_helper/string_helper.hpp"

namespace Slam::ros2 {

class Node : public rclcpp::Node {
  template <typename T>
  using SubPtr = typename rclcpp::Subscription<T>::SharedPtr;

 public:
  Node() : rclcpp::Node("slam_main") {
    declare_parameter("laser_topics", "");
    declare_parameter("pcd_topics", "");

    auto laser_topics = util::split_str(get_parameter("laser_topics").as_string(), ' ');
    auto pcd_topics = util::split_str(get_parameter("pcd_topics").as_string(), ' ');

    for (const auto& topic : laser_topics) {
      laser_map_[topic] = create_subscription<sensor_msgs::msg::LaserScan>(
          topic, rclcpp::SensorDataQoS(),
          [this, topic](const sensor_msgs::msg::LaserScan& data) { laser_callback(topic, data); });
    }

    for (const auto& topic : laser_topics) {
      laser_map_[topic] = create_subscription<sensor_msgs::msg::LaserScan>(
          topic, rclcpp::SensorDataQoS(),
          [this, topic](const sensor_msgs::msg::LaserScan& data) { laser_callback(topic, data); });
    }
  }

  void laser_callback(const std::string& topic, const sensor_msgs::msg::LaserScan& data) {
    auto scan = from_ros(data);
  }

  void laser_callback(const std::string& topic, const sensor_msgs::msg::PointCloud& data) {
    auto pcd = from_ros(data);
  }

 private:
  std::unordered_map<std::string, SubPtr<sensor_msgs::msg::LaserScan>> laser_map_{};
  std::unordered_map<std::string, SubPtr<sensor_msgs::msg::PointCloud>> pcd_map_{};
  LaserScan scan_{};
};

}  // namespace Slam::ros2
