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
  auto img_topics =
      split_str(util::declare_and_get_parameter<std::string>(this, "img_topics"), ' ');
  auto cam_info_topics =
      split_str(util::declare_and_get_parameter<std::string>(this, "cam_info_topics"), ' ');

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

  for (const auto& topic : img_topics) {
    RCLCPP_INFO(get_logger(), "subscribe [%s] Image topic", topic.c_str());
    img_map_[topic] = create_subscription<sensor_msgs::msg::Image>(
        topic, rclcpp::SensorDataQoS(),
        [this, topic](const sensor_msgs::msg::Image& data) { ros_callback(topic, data); });
  }

  for (const auto& topic : cam_info_topics) {
    RCLCPP_INFO(get_logger(), "subscribe [%s] CameraInfo topic", topic.c_str());
    cam_info_map_[topic] = create_subscription<sensor_msgs::msg::CameraInfo>(
        topic, rclcpp::SensorDataQoS(),
        [this, topic](const sensor_msgs::msg::CameraInfo& data) { ros_callback(topic, data); });
  }
}

void SlamNode::ros_callback(const std::string& topic, const sensor_msgs::msg::LaserScan& data) {
  LaserScan scan = from_ros(data);
}

void SlamNode::ros_callback(const std::string& topic, const sensor_msgs::msg::PointCloud2& data) {
  PointCloud pcd = from_ros(data);
}
void SlamNode::ros_callback(const std::string& topic, const sensor_msgs::msg::Image& data) {
  orb_extractor_.receive_image(from_ros(data));
}
void SlamNode::ros_callback(const std::string& topic, const sensor_msgs::msg::CameraInfo& data) {
  orb_extractor_.receive_camera_info(from_ros(data));
}

}  // namespace Slam::ros2