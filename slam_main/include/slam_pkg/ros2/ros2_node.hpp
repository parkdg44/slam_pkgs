//
// Created by park on 24. 4. 2.
//

#pragma once

#include "slam_pkg/module/feature_extractor/orb_extractor.hpp"
#include "slam_pkg/ros2/ros2_converter.hpp"
#include "slam_pkg/ros2/ros2_utils.hpp"
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

  void ros_callback(const std::string& topic, const sensor_msgs::msg::Image& data);

  void ros_callback(const std::string& topic, const sensor_msgs::msg::CameraInfo& data);

 private:
  std::unordered_map<std::string, SubPtr<sensor_msgs::msg::LaserScan>> laser_map_{};
  std::unordered_map<std::string, SubPtr<sensor_msgs::msg::PointCloud2>> pcd_map_{};
  std::unordered_map<std::string, SubPtr<sensor_msgs::msg::Image>> img_map_{};
  std::unordered_map<std::string, SubPtr<sensor_msgs::msg::CameraInfo>> cam_info_map_{};
  LaserScan scan_{};

  module::OrbExtractor orb_extractor_;
};

}  // namespace Slam::ros2
