//
// Created by park on 24. 4. 2.
//

#include "slam_pkg/ros2/ros2_node.hpp"

namespace Slam::ros2 {

SlamNode::SlamNode() : Node("slam_main") {
  using Slam::util::split_str;

  auto laser_topics =
      split_str(util::declare_and_get_parameter<std::string>(this, "laser_topics"), ' ');
  auto pcd_topics =
      split_str(util::declare_and_get_parameter<std::string>(this, "pcd_topics"), ' ');
  auto color_cam_topics =
      split_str(util::declare_and_get_parameter<std::string>(this, "color_cam_topics"), ' ');

  auto odom_topic = util::declare_and_get_parameter<std::string>(this, "odom_topic", "odom");

  for (const auto& topic : laser_topics) {
    RCLCPP_INFO(get_logger(), "subscribe [%s] LaserScan topic", topic.c_str());
    laser_map_[topic] = create_subscription<sensor_msgs::msg::LaserScan>(
        topic, rclcpp::SensorDataQoS(),
        [this, topic](const sensor_msgs::msg::LaserScan& data) { ros_callback(topic, data); });
  }

  for (const auto& topic : pcd_topics) {
    RCLCPP_INFO(get_logger(), "subscribe [%s] PointCloud topic", topic.c_str());
    pcd_map_[topic] = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS(),
        [this, topic](const sensor_msgs::msg::PointCloud2& data) { ros_callback(topic, data); });
  }

  for (const auto& topic : color_cam_topics) {
    RCLCPP_INFO(get_logger(), "subscribe [%s/color/image] Image topic", topic.c_str());
    img_map_[topic] = create_subscription<sensor_msgs::msg::Image>(
        topic + "/color/image", rclcpp::SensorDataQoS(),
        [this, topic](const sensor_msgs::msg::Image& data) { ros_callback(topic, data); });

    RCLCPP_INFO(get_logger(), "subscribe [%s/color/camera_info]  CameraInfo topic", topic.c_str());
    cam_info_map_[topic] = create_subscription<sensor_msgs::msg::CameraInfo>(
        topic + "/color/camera_info", rclcpp::SensorDataQoS(),
        [this, topic](const sensor_msgs::msg::CameraInfo& data) { ros_callback(topic, data); });
  }

  RCLCPP_INFO(get_logger(), "subscribe [%s] Odometry topic", odom_topic.c_str());
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this, odom_topic](const nav_msgs::msg::Odometry& data) { ros_callback(odom_topic, data); });
}

void SlamNode::ros_callback(const std::string& topic, const sensor_msgs::msg::LaserScan& data) {
  LaserScan scan = from_ros(data);
}

void SlamNode::ros_callback(const std::string& topic, const sensor_msgs::msg::PointCloud2& data) {
  PointCloud pcd = from_ros(data);
}

void SlamNode::ros_callback(const std::string& topic, const sensor_msgs::msg::Image& data) {
  orb_extractors_[topic].receive_data(topic, from_ros(data));
}

void SlamNode::ros_callback(const std::string& topic, const sensor_msgs::msg::CameraInfo& data) {
  orb_extractors_[topic].receive_data(topic, from_ros(data));
}

void SlamNode::ros_callback(const std::string& topic, const nav_msgs::msg::Odometry& data) {
  for (auto& [topic, extractor] : orb_extractors_) {
    extractor.receive_data(topic, from_ros(data));
  }
}

}  // namespace Slam::ros2