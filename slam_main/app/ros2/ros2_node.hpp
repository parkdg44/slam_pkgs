//
// Created by park on 24. 4. 2.
//

#pragma once

#include "slam_pkg/util/string_helper.hpp"

// ros2
#include "ros2_converter.hpp"
#include "ros2_utils.hpp"

namespace Slam::ros2 {

class SlamNode : public rclcpp::Node {
  template <typename RosT>
  using Subscriber = typename rclcpp::Subscription<RosT>::SharedPtr;

 public:
  SlamNode() : Node("slam_main") {
    using Slam::util::split_str;

    auto laser_topics =
        split_str(util::declare_and_get_parameter<std::string>(this, "laser_topics"), ' ');
    auto pcd_topics =
        split_str(util::declare_and_get_parameter<std::string>(this, "pcd_topics"), ' ');
    auto color_cam_topics =
        split_str(util::declare_and_get_parameter<std::string>(this, "color_cam_topics"), ' ');

    auto odom_topic = util::declare_and_get_parameter<std::string>(this, "odom_topic", "odom");

    // for (const auto& topic : laser_topics) {
    //   RCLCPP_INFO(get_logger(), "subscribe [%s] LaserScan topic", topic.c_str());
    //   laser_map_[topic] = create_subscription<sensor_msgs::msg::LaserScan>(
    //       topic, rclcpp::SensorDataQoS(),
    //       [this, topic](const sensor_msgs::msg::LaserScan& data) { ros_callback(topic, data); });
    // }
    //
    // for (const auto& topic : pcd_topics) {
    //   RCLCPP_INFO(get_logger(), "subscribe [%s] PointCloud topic", topic.c_str());
    //   pcd_map_[topic] = create_subscription<sensor_msgs::msg::PointCloud2>(
    //       topic, rclcpp::SensorDataQoS(),
    //       [this, topic](const sensor_msgs::msg::PointCloud2& data) { ros_callback(topic, data);
    //       });
    // }
    //
    // RCLCPP_INFO(get_logger(), "subscribe [%s] Odometry topic", odom_topic.c_str());
    // sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    //     odom_topic, rclcpp::SensorDataQoS(),
    //     [this, odom_topic](const nav_msgs::msg::Odometry& data) { ros_callback(odom_topic, data);
    //     });
  }

 private:
  // std::unordered_map<std::string, Subscriber<sensor_msgs::msg::LaserScan>> laser_map_{};
  // std::unordered_map<std::string, Subscriber<sensor_msgs::msg::PointCloud2>> pcd_map_{};
  // std::unordered_map<std::string, Subscriber<sensor_msgs::msg::Image>> img_map_{};
  Subscriber<nav_msgs::msg::Odometry> sub_odom_{};
};

}  // namespace Slam::ros2
