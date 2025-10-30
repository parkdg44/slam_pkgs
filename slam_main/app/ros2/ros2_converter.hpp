//
// Created by park on 24. 4. 3.
//

#pragma once

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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
  output.x = data.x();
  output.y = data.y();
  output.z = data.z();
  return output;
}

inline geometry_msgs::msg::Quaternion to_ros(Quaternion const& data) {
  geometry_msgs::msg::Quaternion output;
  output.w = data.w();
  output.x = data.x();
  output.y = data.y();
  output.z = data.z();
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
  output.x = data.x();
  output.y = data.y();
  output.z = data.z();
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

inline Header from_ros(std_msgs::msg::Header const& msg) {
  Header output;
  output.ref_frame_id = msg.frame_id;
  output.stamp = rclcpp::Time(msg.stamp).nanoseconds();
  return output;
}

inline LaserScan from_ros(sensor_msgs::msg::LaserScan const& msg) {
  LaserScan output;
  output.header = from_ros(msg.header);
  output.ranges = msg.ranges;
  output.intensities = msg.intensities;
  ScanInfo& info = output.info;
  info.angle_increment = msg.angle_increment;
  info.angle_max = msg.angle_max;
  info.angle_min = msg.angle_min;
  info.range_max = msg.range_max;
  info.range_min = msg.range_min;
  info.scan_time = msg.scan_time;
  info.time_increment = msg.time_increment;
  return output;
}

inline Point from_ros(geometry_msgs::msg::Point32 const& msg) {
  Point output{msg.x, msg.y, msg.z};
  return output;
}

inline PointCloud from_ros(sensor_msgs::msg::PointCloud const& msg) {
  PointCloud output;
  output.header = from_ros(msg.header);
  output.points.reserve(msg.points.size());
  for (const auto& p : msg.points) {
    output.points.push_back(from_ros(p));
  }
  return output;
}

inline PointCloud from_ros(sensor_msgs::msg::PointCloud2 const& msg) {
  PointCloud output;

  int width = static_cast<int>(msg.width);
  int height = static_cast<int>(msg.width);
  int step = static_cast<int>(msg.point_step);
  uint datatype = msg.fields[0].datatype;

  int p_size = width * height;
  output.points.resize(p_size);

  // float 16 (float)
  if (datatype == 7) {
    float xyz[3];
    for (int i = 0; i < width * height; i++) {
      memcpy(&xyz, &msg.data[i * step], 3 * sizeof(float));
      output.points[i] = {xyz[0], xyz[1], xyz[2]};
    }
  }
  // float 32 (double)
  else if (datatype == 8) {
    double xyz[3];
    for (int i = 0; i < width * height; i++) {
      memcpy(&xyz, &msg.data[i * step], 3 * sizeof(double));
      output.points[i] = {xyz[0], xyz[1], xyz[2]};
    }
  }

  output.header = from_ros(msg.header);
  output.info.width = width;
  output.info.height = height;

  return output;
}

inline Image from_ros(sensor_msgs::msg::Image const& msg) {
  Image output;

  auto cv_output = cv_bridge::toCvCopy(msg, msg.encoding);
  output.header = from_ros(cv_output->header);
  output.encoding = cv_output->encoding;
  output.value = std::move(cv_output->image);

  return output;
}

inline CameraInfo from_ros(sensor_msgs::msg::CameraInfo const& msg) {
  CameraInfo output;

  output.header = from_ros(msg.header);
  output.height = msg.height;
  output.width = msg.width;
  output.distortion_model = msg.distortion_model;
  output.d = msg.d;
  output.k = decltype(output.k){msg.k.data()};
  output.r = decltype(output.r){msg.r.data()};
  output.p = decltype(output.p){msg.p.data()};

  return output;
}

inline Point from_ros(geometry_msgs::msg::Point const& data) {
  Point output;
  output.x() = data.x;
  output.y() = data.y;
  output.z() = data.z;
  return output;
}

inline Quaternion from_ros(geometry_msgs::msg::Quaternion const& data) {
  Quaternion output;
  output.w() = data.w;
  output.x() = data.x;
  output.y() = data.y;
  output.z() = data.z;
  return output;
}

inline Pose from_ros(geometry_msgs::msg::Pose const& data) {
  Pose output;
  output.p = from_ros(data.position);
  output.q = from_ros(data.orientation);
  return output;
}

inline PoseWithCovariance from_ros(geometry_msgs::msg::PoseWithCovariance const& data) {
  PoseWithCovariance output;
  output.pose = from_ros(data.pose);
  memcpy(output.cov.value.data(), &data.covariance, sizeof(double) * 36);
  return output;
}

inline PoseStamped from_ros(geometry_msgs::msg::PoseStamped const& data) {
  PoseStamped output;
  output.header = from_ros(data.header);
  output.pose = from_ros(data.pose);
  return output;
}

inline Vector from_ros(geometry_msgs::msg::Vector3 const& data) {
  Vector output;
  output.value = {data.x, data.y, data.z};
  return output;
}

inline Twist from_ros(geometry_msgs::msg::Twist const& data) {
  Twist output;
  output.angular = from_ros(data.angular);
  output.linear = from_ros(data.linear);
  return output;
}

inline TwistWithCovariance from_ros(geometry_msgs::msg::TwistWithCovariance const& data) {
  TwistWithCovariance output;
  output.twist = from_ros(data.twist);
  memcpy(output.cov.value.data(), &data.covariance, sizeof(double) * 36);
  return output;
}

inline Odometry from_ros(nav_msgs::msg::Odometry const& data) {
  Odometry output;
  output.header = from_ros(data.header);
  output.pose_w_cov = from_ros(data.pose);
  output.twist_w_cov = from_ros(data.twist);
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
