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
    using namespace util;

    auto laser_topics = split_str(declare_and_get_parameter<std::string>("laser_topics"), ' ');
    auto pcd_topics = split_str(declare_and_get_parameter<std::string>("pcd_topics"), ' ');

    for (const auto& topic : laser_topics) {
      RCLCPP_INFO(get_logger(), "subscribe [%s] LaserScan topic", topic.c_str());
      laser_map_[topic] = create_subscription<sensor_msgs::msg::LaserScan>(
          topic, rclcpp::SensorDataQoS(),
          [this, topic](const sensor_msgs::msg::LaserScan& data) { laser_callback(topic, data); });
    }

    for (const auto& topic : pcd_topics) {
      RCLCPP_INFO(get_logger(), "subscribe [%s] PointCloud topic", topic.c_str());
      pcd_map_[topic] = create_subscription<sensor_msgs::msg::PointCloud>(
          topic, rclcpp::SensorDataQoS(),
          [this, topic](const sensor_msgs::msg::PointCloud& data) { pcd_callback(topic, data); });
    }
  }

  void laser_callback(const std::string& topic, const sensor_msgs::msg::LaserScan& data) {
    auto scan = from_ros(data);
  }

  void pcd_callback(const std::string& topic, const sensor_msgs::msg::PointCloud& data) {
    auto pcd = from_ros(data);
  }

 private:
  template <typename T>
  T declare_and_get_parameter(std::string const& name, T default_value = {}) {
    declare_parameter(name, default_value);

    if constexpr (std::is_same_v<T, std::string>) {
      return get_parameter(name).as_string();
    }
    if constexpr (std::is_same_v<T, bool>) {
      return get_parameter(name).as_bool();
    }
    if constexpr (std::is_same_v<T, int>) {
      return get_parameter(name).as_int();
    }
    if constexpr (std::is_same_v<T, double>) {
      return get_parameter(name).as_double();
    }
    if constexpr (std::is_same_v<T, std::vector<std::string>>) {
      return get_parameter(name).as_string_array();
    }
    if constexpr (std::is_same_v<T, std::vector<bool>>) {
      return get_parameter(name).as_bool_array();
    }
    if constexpr (std::is_same_v<T, std::vector<int>>) {
      return get_parameter(name).as_integer_array();
    }
    if constexpr (std::is_same_v<T, std::vector<double>>) {
      return get_parameter(name).as_double_array();
    }
    throw std::runtime_error(std::string(typeid(T).name()) + " type can't be used.");
  }

  std::unordered_map<std::string, SubPtr<sensor_msgs::msg::LaserScan>> laser_map_{};
  std::unordered_map<std::string, SubPtr<sensor_msgs::msg::PointCloud>> pcd_map_{};
  LaserScan scan_{};
};

}  // namespace Slam::ros2
