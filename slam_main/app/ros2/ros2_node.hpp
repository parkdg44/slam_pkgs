//
// Created by park on 24. 4. 2.
//

#pragma once

#include "ros2_converter.hpp"
#include "slam_pkg/module/feature_extractor/orb_extractor.hpp"
#include "ros2_utils.hpp"
#include "slam_pkg/util/string_helper/string_helper.hpp"

namespace Slam::ros2 {

class SlamNode : public rclcpp::Node {
  template <typename RosT>
  using SubPtr = typename rclcpp::Subscription<RosT>::SharedPtr;
  template <typename RosT>
  using CbFunc = std::function<void(RosT const&)>;

 public:
  SlamNode();

  void ros_callback(const std::string& topic, const sensor_msgs::msg::LaserScan& data);

  void ros_callback(const std::string& topic, const sensor_msgs::msg::PointCloud2& data);

  void ros_callback(const std::string& topic, const nav_msgs::msg::Odometry& data);

 private:
  std::unordered_map<std::string, SubPtr<sensor_msgs::msg::LaserScan>> laser_map_{};
  std::unordered_map<std::string, SubPtr<sensor_msgs::msg::PointCloud2>> pcd_map_{};
  std::unordered_map<std::string, SubPtr<sensor_msgs::msg::Image>> img_map_{};
  SubPtr<nav_msgs::msg::Odometry> sub_odom_{};
};

}  // namespace Slam::ros2
