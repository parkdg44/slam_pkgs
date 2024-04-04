//
// Created by park on 24. 4. 3.
//

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "type/types.hpp"

namespace Slam::ros2 {

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

inline geometry_msgs::msg::PoseStamped to_ros(Pose const& data) {
  geometry_msgs::msg::PoseStamped output;
  output.header = to_ros(data.header);
  output.pose.position = to_ros(data.p);
  output.pose.orientation = to_ros(data.q);
  return output;
}

template <typename SlamT>
std::function<void(std::string const&, SlamT const&)> ros2_function_builder(rclcpp::Node& node) {
  return [&node](std::string const& topic, SlamT const& data) {
    using RosT = decltype(to_ros(data));
    static auto map_pub = std::unordered_map<std::string, std::shared_ptr<rclcpp::PublisherBase>>();

    if (map_pub.find(topic) == map_pub.end()) {
      map_pub[topic] = node.create_publisher<RosT>(topic);
    }
    auto pub_data = std::static_pointer_cast<rclcpp::Publisher<RosT>>(map_pub[topic]);

    pub_data->publish(to_ros(data));
  };
}

}  // namespace Slam::ros2
